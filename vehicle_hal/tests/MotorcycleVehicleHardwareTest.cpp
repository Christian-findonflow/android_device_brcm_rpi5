/*
 * Copyright (C) 2024 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-side unit tests for the motorcycle Vehicle HAL decode and config
 * logic. These run on the development machine (atest --host
 * motorcycle_vhal_test) with no device and no CAN bus: frames are fed
 * directly into the decode path through a test peer.
 */

#include "MotorcycleVehicleHardware.h"

#include <gtest/gtest.h>

#include <cutils/properties.h>
#include <utils/SystemClock.h>

#include <linux/can.h>

#include <mutex>
#include <optional>
#include <vector>

namespace android::hardware::automotive::vehicle::motorcycle {

using ::aidl::android::hardware::automotive::vehicle::VehicleGear;
using ::aidl::android::hardware::automotive::vehicle::VehicleProperty;

// Friended by MotorcycleVehicleHardware: exposes the private decode/config
// entry points to the tests.
class MotorcycleVehicleHardwareTestPeer {
  public:
    explicit MotorcycleVehicleHardwareTestPeer(MotorcycleVehicleHardware* hw) : mHw(hw) {}

    void processCanFrame(const struct can_frame& frame) { mHw->processCanFrame(frame); }

    StatusCode applyConfigValue(const VehiclePropValue& value) {
        return mHw->applyConfigValue(value);
    }

    void checkLinkTimeouts(int64_t nowNs) { mHw->checkLinkTimeouts(nowNs); }

    // Range-model entry points take explicit timestamps, so tests can drive
    // them with synthetic time instead of depending on wall-clock spacing.
    void accumulateDistance(float speedMps, int64_t ts) { mHw->accumulateDistance(speedMps, ts); }
    void accumulateEnergy(float voltage, float current, float speedMps, int64_t ts) {
        mHw->accumulateEnergy(voltage, current, speedMps, ts);
    }
    void publishRange(int64_t ts) { mHw->publishRange(ts); }
    void setLastSoc(float soc) { mHw->mLastSocPercent = soc; }

  private:
    MotorcycleVehicleHardware* mHw;
};

namespace {

constexpr int32_t PROP_ENGINE_RPM = static_cast<int32_t>(VehicleProperty::ENGINE_RPM);
constexpr int32_t PROP_SPEED = static_cast<int32_t>(VehicleProperty::PERF_VEHICLE_SPEED);
constexpr int32_t PROP_CURRENT_GEAR = static_cast<int32_t>(VehicleProperty::CURRENT_GEAR);
constexpr int32_t PROP_CHARGE_RATE =
        static_cast<int32_t>(VehicleProperty::EV_BATTERY_INSTANTANEOUS_CHARGE_RATE);
constexpr int32_t PROP_COOLANT = static_cast<int32_t>(VehicleProperty::ENGINE_COOLANT_TEMP);
constexpr int32_t PROP_OIL = static_cast<int32_t>(VehicleProperty::ENGINE_OIL_TEMP);
constexpr int32_t PROP_ODOMETER = static_cast<int32_t>(VehicleProperty::PERF_ODOMETER);
constexpr int32_t PROP_EV_BATTERY_LEVEL =
        static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL);
constexpr int32_t PROP_RANGE = static_cast<int32_t>(VehicleProperty::RANGE_REMAINING);
constexpr int32_t PROP_BATTERY_CAPACITY =
        static_cast<int32_t>(VehicleProperty::INFO_EV_BATTERY_CAPACITY);

struct can_frame makeFrame(uint32_t id, bool extended, std::initializer_list<uint8_t> bytes) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = id | (extended ? CAN_EFF_FLAG : 0);
    frame.can_dlc = 8;
    size_t i = 0;
    for (uint8_t b : bytes) {
        frame.data[i++] = b;
    }
    return frame;
}

VehiclePropValue makeFloatValue(int32_t propId, float value) {
    VehiclePropValue v;
    v.prop = propId;
    v.areaId = 0;
    v.value.floatValues = {value};
    return v;
}

VehiclePropValue makeIntValue(int32_t propId, int32_t value) {
    VehiclePropValue v;
    v.prop = propId;
    v.areaId = 0;
    v.value.int32Values = {value};
    return v;
}

class MotorcycleVehicleHardwareTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // The range model loads its learned consumption and pack config from
        // persist properties in the constructor, and property_set persists
        // in-process on the host - clear them BEFORE construction.
        property_set("persist.vendor.motodash.whperkm", "");
        property_set("persist.vendor.motodash.cfg.pack_energy_wh", "");
        // Nonexistent interface: the reader thread stays in its retry loop and
        // never interferes; frames are injected through the peer instead.
        mHardware = std::make_unique<MotorcycleVehicleHardware>("vcan-test-none");
        mHardware->registerOnPropertyChangeEvent(
                std::make_unique<const IVehicleHardware::PropertyChangeCallback>(
                        [this](std::vector<VehiclePropValue> values) {
                            std::lock_guard<std::mutex> lock(mEventsMutex);
                            for (auto& v : values) {
                                mEvents.push_back(std::move(v));
                            }
                        }));
        mPeer = std::make_unique<MotorcycleVehicleHardwareTestPeer>(mHardware.get());

        // Config persists through property_set even on the host (in-process),
        // so reset to defaults for test isolation.
        mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_WHEEL_CIRCUMFERENCE, 1.894f));
        mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_GEAR_RATIO, 4.0f));
        mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_ID_CONTROLLER_STATUS,
                                             static_cast<int32_t>(CAN_ID_CONTROLLER_STATUS)));
        mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_ID_CONTROLLER_TEMPS,
                                             static_cast<int32_t>(CAN_ID_CONTROLLER_TEMPS)));
        mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_ID_BMS,
                                             static_cast<int32_t>(CAN_ID_BMS)));
        clearEvents();
    }

    std::optional<VehiclePropValue> lastEvent(int32_t propId) {
        std::lock_guard<std::mutex> lock(mEventsMutex);
        for (auto it = mEvents.rbegin(); it != mEvents.rend(); ++it) {
            if (it->prop == propId) return *it;
        }
        return std::nullopt;
    }

    size_t countEvents(int32_t propId) {
        std::lock_guard<std::mutex> lock(mEventsMutex);
        size_t n = 0;
        for (const auto& e : mEvents) {
            if (e.prop == propId) n++;
        }
        return n;
    }

    void clearEvents() {
        std::lock_guard<std::mutex> lock(mEventsMutex);
        mEvents.clear();
    }

    std::unique_ptr<MotorcycleVehicleHardware> mHardware;
    std::unique_ptr<MotorcycleVehicleHardwareTestPeer> mPeer;
    std::mutex mEventsMutex;
    std::vector<VehiclePropValue> mEvents;
};

// Controller status 0x10261022: [errors, flags|gear, rpmL, rpmH, vL, vH, aL, aH]
TEST_F(MotorcycleVehicleHardwareTest, ControllerStatusDecodesRpmSpeedGearPower) {
    // gear D (bits 4-7 = 3), rpm 3000, 72.0 V, 25.5 A discharge
    auto frame = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                           {0x00, 0x30, 0xB8, 0x0B, 0xD0, 0x02, 0xFF, 0x00});
    mPeer->processCanFrame(frame);

    auto rpm = lastEvent(PROP_ENGINE_RPM);
    ASSERT_TRUE(rpm.has_value());
    EXPECT_FLOAT_EQ(rpm->value.floatValues[0], 3000.0f);

    // Default config: 1.894 m circumference, ratio 4.0
    auto speed = lastEvent(PROP_SPEED);
    ASSERT_TRUE(speed.has_value());
    EXPECT_NEAR(speed->value.floatValues[0], 3000.0f * 1.894f / (4.0f * 60.0f), 0.01f);

    auto gear = lastEvent(PROP_CURRENT_GEAR);
    ASSERT_TRUE(gear.has_value());
    EXPECT_EQ(gear->value.int32Values[0], static_cast<int32_t>(VehicleGear::GEAR_DRIVE));

    auto volts = lastEvent(VENDOR_BATTERY_VOLTAGE);
    ASSERT_TRUE(volts.has_value());
    EXPECT_FLOAT_EQ(volts->value.floatValues[0], 72.0f);

    auto amps = lastEvent(VENDOR_BATTERY_CURRENT);
    ASSERT_TRUE(amps.has_value());
    EXPECT_FLOAT_EQ(amps->value.floatValues[0], 25.5f);

    // AOSP semantics: milliwatts, positive = charging. 25.5A discharge at
    // 72V is therefore -1,836,000 mW.
    auto power = lastEvent(PROP_CHARGE_RATE);
    ASSERT_TRUE(power.has_value());
    EXPECT_NEAR(power->value.floatValues[0], -(72.0f * 25.5f) * 1000.0f, 500.0f);
}

TEST_F(MotorcycleVehicleHardwareTest, RegenCurrentIsSigned) {
    // current raw = -100 (0xFF9C little-endian) => -10.0 A (regen)
    auto frame = makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                           {0x00, 0x20, 0x00, 0x00, 0xD0, 0x02, 0x9C, 0xFF});
    mPeer->processCanFrame(frame);

    auto amps = lastEvent(VENDOR_BATTERY_CURRENT);
    ASSERT_TRUE(amps.has_value());
    EXPECT_FLOAT_EQ(amps->value.floatValues[0], -10.0f);

    // Regen: charging, so the AOSP charge-rate property goes positive
    auto power = lastEvent(PROP_CHARGE_RATE);
    ASSERT_TRUE(power.has_value());
    EXPECT_GT(power->value.floatValues[0], 0.0f);
}

// Controller temps 0x10261023: [ctrlTemp, motorTemp, -, hall, throttle, -, err, -]
TEST_F(MotorcycleVehicleHardwareTest, ControllerTempsDecode) {
    auto frame = makeFrame(CAN_ID_CONTROLLER_TEMPS, true,
                           {45, 60, 0, 0, 55, 0, 0, 0});
    mPeer->processCanFrame(frame);

    auto ctrl = lastEvent(PROP_COOLANT);
    ASSERT_TRUE(ctrl.has_value());
    EXPECT_FLOAT_EQ(ctrl->value.floatValues[0], 45.0f);

    auto motor = lastEvent(PROP_OIL);
    ASSERT_TRUE(motor.has_value());
    EXPECT_FLOAT_EQ(motor->value.floatValues[0], 60.0f);

    auto throttle = lastEvent(VENDOR_THROTTLE_POSITION);
    ASSERT_TRUE(throttle.has_value());
    EXPECT_FLOAT_EQ(throttle->value.floatValues[0], 55.0f);
}

// BMS broadcast 0x6B1, observed layout: SOC = byte 3, temp = byte 7 - 40
TEST_F(MotorcycleVehicleHardwareTest, BmsBroadcastDecodesSocAndTemp) {
    auto frame = makeFrame(CAN_ID_BMS, /*extended=*/false,
                           {0, 99, 0, 18, 3, 2, 0, 51});
    mPeer->processCanFrame(frame);

    auto soc = lastEvent(PROP_EV_BATTERY_LEVEL);
    ASSERT_TRUE(soc.has_value());
    EXPECT_FLOAT_EQ(soc->value.floatValues[0], 18.0f);

    auto temp = lastEvent(VENDOR_PACK_TEMP_AVG);
    ASSERT_TRUE(temp.has_value());
    EXPECT_FLOAT_EQ(temp->value.floatValues[0], 11.0f);  // 51 - 40
}

// OBD2 Mode 0x22 response: [len, 0x62, pidHi, pidLo, data...]
TEST_F(MotorcycleVehicleHardwareTest, Obd2SohResponse) {
    auto frame = makeFrame(CAN_ID_OBD2_RESPONSE, false,
                           {0x04, 0x62, 0xF0, 0x13, 95, 0, 0, 0});
    mPeer->processCanFrame(frame);

    auto soh = lastEvent(VENDOR_PACK_SOH);
    ASSERT_TRUE(soh.has_value());
    EXPECT_FLOAT_EQ(soh->value.floatValues[0], 95.0f);
}

TEST_F(MotorcycleVehicleHardwareTest, Obd2CellVoltageScaling) {
    // PID 0xF032 low cell voltage, raw 36500 (0x8E94, LE in payload) => 3.65 V
    auto frame = makeFrame(CAN_ID_OBD2_RESPONSE, false,
                           {0x05, 0x62, 0xF0, 0x32, 0x94, 0x8E, 0, 0});
    mPeer->processCanFrame(frame);

    auto low = lastEvent(VENDOR_CELL_VOLTAGE_LOW);
    ASSERT_TRUE(low.has_value());
    EXPECT_NEAR(low->value.floatValues[0], 3.65f, 0.0001f);
}

TEST_F(MotorcycleVehicleHardwareTest, ConfigSpeedParamsApplyLive) {
    EXPECT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_WHEEL_CIRCUMFERENCE, 2.0f)),
              StatusCode::OK);
    EXPECT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_GEAR_RATIO, 5.0f)),
              StatusCode::OK);

    // Config writes notify subscribers (keeps settings UIs in sync)
    auto cfg = lastEvent(VENDOR_CFG_GEAR_RATIO);
    ASSERT_TRUE(cfg.has_value());
    EXPECT_FLOAT_EQ(cfg->value.floatValues[0], 5.0f);

    // rpm 3000 with new config: 3000 / 5 wheel rpm * 2.0 m / 60 = 20 m/s
    auto frame = makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                           {0x00, 0x20, 0xB8, 0x0B, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(frame);

    auto speed = lastEvent(PROP_SPEED);
    ASSERT_TRUE(speed.has_value());
    EXPECT_NEAR(speed->value.floatValues[0], 20.0f, 0.01f);
}

TEST_F(MotorcycleVehicleHardwareTest, ConfigCanIdReroutesDecode) {
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_ID_CONTROLLER_STATUS, 0x123)),
              StatusCode::OK);
    clearEvents();

    // Old ID is now unknown: no RPM event
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x20, 0xB8, 0x0B, 0x00, 0x00, 0x00, 0x00}));
    EXPECT_EQ(countEvents(PROP_ENGINE_RPM), 0u);

    // New ID decodes (interface flag doesn't matter, only the ID)
    mPeer->processCanFrame(makeFrame(0x123, false,
                                     {0x00, 0x20, 0xD2, 0x04, 0x00, 0x00, 0x00, 0x00}));
    auto rpm = lastEvent(PROP_ENGINE_RPM);
    ASSERT_TRUE(rpm.has_value());
    EXPECT_FLOAT_EQ(rpm->value.floatValues[0], 1234.0f);
}

TEST_F(MotorcycleVehicleHardwareTest, ConfigValidationRejectsBadValues) {
    // Out of range
    EXPECT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_WHEEL_CIRCUMFERENCE, 9.0f)),
              StatusCode::INVALID_ARG);
    EXPECT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_GEAR_RATIO, 0.1f)),
              StatusCode::INVALID_ARG);
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_ID_BMS, 0)),
              StatusCode::INVALID_ARG);
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_GPIO_LEFT_TURN, 99)),
              StatusCode::INVALID_ARG);
    // Wrong value type
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_WHEEL_CIRCUMFERENCE, 2)),
              StatusCode::INVALID_ARG);
    // Non-config properties stay read-only
    EXPECT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_BATTERY_VOLTAGE, 80.0f)),
              StatusCode::ACCESS_DENIED);
}

// Fault bits: 0x10261022 byte 0 (motor/hall/throttle/controller/brake/limp)
// and 0x10261023 byte 6 (over-current/voltage/temperature).
TEST_F(MotorcycleVehicleHardwareTest, ControllerFaultBitsAreSurfaced) {
    // byte0 = motor fault | throttle fault
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x05, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));

    auto faults = lastEvent(VENDOR_FAULT_FLAGS);
    ASSERT_TRUE(faults.has_value());
    EXPECT_EQ(faults->value.int32Values[0], FAULT_MOTOR | FAULT_THROTTLE);
}

TEST_F(MotorcycleVehicleHardwareTest, TemperatureFaultBitsUseTheSecondByte) {
    // temps frame byte6 = controller over-temp (bit 3) | motor over-temp (bit 4)
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_TEMPS, true,
                                     {90, 120, 0, 0, 0, 0, 0x18, 0}));

    auto faults = lastEvent(VENDOR_FAULT_FLAGS);
    ASSERT_TRUE(faults.has_value());
    EXPECT_EQ(faults->value.int32Values[0],
              FAULT_CONTROLLER_OVER_TEMP | FAULT_MOTOR_OVER_TEMP);
}

TEST_F(MotorcycleVehicleHardwareTest, FaultsFromBothFramesCombineAndClearIndependently) {
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_TEMPS, true,
                                     {90, 120, 0, 0, 0, 0, 0x10, 0}));
    auto faults = lastEvent(VENDOR_FAULT_FLAGS);
    ASSERT_TRUE(faults.has_value());
    EXPECT_EQ(faults->value.int32Values[0], FAULT_MOTOR | FAULT_MOTOR_OVER_TEMP);

    // Clearing the controller frame's byte must not clear the temperature bits
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
    faults = lastEvent(VENDOR_FAULT_FLAGS);
    EXPECT_EQ(faults->value.int32Values[0], FAULT_MOTOR_OVER_TEMP);
}

TEST_F(MotorcycleVehicleHardwareTest, FaultsNotifyOnlyOnChange) {
    auto frame = makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                           {0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    mPeer->processCanFrame(frame);
    mPeer->processCanFrame(frame);
    mPeer->processCanFrame(frame);
    EXPECT_EQ(countEvents(VENDOR_FAULT_FLAGS), 1u);
}

// Odometer integrates speed over time; both meters advance together and the
// trip meter is resettable while the odometer keeps counting.
TEST_F(MotorcycleVehicleHardwareTest, OdometerAccumulatesWithDistance) {
    auto rolling = makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                             {0x00, 0x20, 0xB8, 0x0B, 0x00, 0x00, 0x00, 0x00});
    // First frame only establishes the time reference (no elapsed interval yet)
    mPeer->processCanFrame(rolling);
    auto start = lastEvent(PROP_ODOMETER);
    ASSERT_TRUE(start.has_value());
    float startKm = start->value.floatValues[0];

    // Subsequent frames accumulate; the exact distance depends on wall-clock
    // spacing, so assert monotonic non-negative growth rather than a value.
    for (int i = 0; i < 5; i++) {
        mPeer->processCanFrame(rolling);
    }
    auto after = lastEvent(PROP_ODOMETER);
    ASSERT_TRUE(after.has_value());
    EXPECT_GE(after->value.floatValues[0], startKm);

    auto trip = lastEvent(VENDOR_TRIP_DISTANCE);
    ASSERT_TRUE(trip.has_value());
    EXPECT_GE(trip->value.floatValues[0], 0.0f);
}

TEST_F(MotorcycleVehicleHardwareTest, StationaryBikeDoesNotAccumulateDistance) {
    auto stopped = makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                             {0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    mPeer->processCanFrame(stopped);
    float before = lastEvent(PROP_ODOMETER)->value.floatValues[0];
    for (int i = 0; i < 10; i++) {
        mPeer->processCanFrame(stopped);
    }
    EXPECT_FLOAT_EQ(lastEvent(PROP_ODOMETER)->value.floatValues[0], before);
}

TEST_F(MotorcycleVehicleHardwareTest, TripResetLeavesOdometerIntact) {
    auto rolling = makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                             {0x00, 0x20, 0xB8, 0x0B, 0x00, 0x00, 0x00, 0x00});
    for (int i = 0; i < 4; i++) {
        mPeer->processCanFrame(rolling);
    }
    float odoBefore = lastEvent(PROP_ODOMETER)->value.floatValues[0];

    VehiclePropValue reset;
    reset.prop = VENDOR_TRIP_DISTANCE;
    reset.areaId = 0;
    reset.value.floatValues = {0.0f};
    EXPECT_EQ(mPeer->applyConfigValue(reset), StatusCode::OK);

    // The reset is applied by the reader thread on the next frame
    mPeer->processCanFrame(rolling);

    EXPECT_GE(lastEvent(PROP_ODOMETER)->value.floatValues[0], odoBefore);
    EXPECT_LT(lastEvent(VENDOR_TRIP_DISTANCE)->value.floatValues[0], 0.001f);
}

// Status bits (0x10261022 data1 low nibble): lock, brake, cruise, side stand
TEST_F(MotorcycleVehicleHardwareTest, StatusFlagsSurfaceSideStandAndBrake) {
    // data1 = 0x2A: gear N (0x20) + brake (bit1) + side stand (bit3)
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
    auto status = lastEvent(VENDOR_STATUS_FLAGS);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->value.int32Values[0], STATUS_BRAKE | STATUS_SIDE_STAND);

    // Clearing them notifies again
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
    status = lastEvent(VENDOR_STATUS_FLAGS);
    EXPECT_EQ(status->value.int32Values[0], 0);
}

TEST_F(MotorcycleVehicleHardwareTest, StatusFlagsNotifyOnlyOnChange) {
    auto frame = makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                           {0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    mPeer->processCanFrame(frame);
    mPeer->processCanFrame(frame);
    EXPECT_EQ(countEvents(VENDOR_STATUS_FLAGS), 1u);
}

// Link status: frames set the bit; silence (via the watchdog check with an
// explicit "now") clears it, so the UI can grey out instead of freezing.
TEST_F(MotorcycleVehicleHardwareTest, LinkDropsWhenControllerGoesSilent) {
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x20, 0xB8, 0x0B, 0x00, 0x00, 0x00, 0x00}));
    auto link = lastEvent(VENDOR_LINK_STATUS);
    ASSERT_TRUE(link.has_value());
    EXPECT_EQ(link->value.int32Values[0] & LINK_CONTROLLER, LINK_CONTROLLER);

    // 2s later with no frames: controller link must drop
    int64_t now = ::android::elapsedRealtimeNano() + 2000000000LL;
    mPeer->checkLinkTimeouts(now);
    link = lastEvent(VENDOR_LINK_STATUS);
    ASSERT_TRUE(link.has_value());
    EXPECT_EQ(link->value.int32Values[0] & LINK_CONTROLLER, 0);

    // Frames resume: link restored
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x20, 0xB8, 0x0B, 0x00, 0x00, 0x00, 0x00}));
    link = lastEvent(VENDOR_LINK_STATUS);
    EXPECT_EQ(link->value.int32Values[0] & LINK_CONTROLLER, LINK_CONTROLLER);
}

TEST_F(MotorcycleVehicleHardwareTest, BmsLinkIndependentOfController) {
    mPeer->processCanFrame(makeFrame(CAN_ID_BMS, false, {0, 99, 0, 18, 3, 2, 0, 51}));
    auto link = lastEvent(VENDOR_LINK_STATUS);
    ASSERT_TRUE(link.has_value());
    EXPECT_EQ(link->value.int32Values[0], LINK_BMS);

    // Controller timeout must not clear the BMS bit
    int64_t now = ::android::elapsedRealtimeNano() + 2000000000LL;
    mPeer->checkLinkTimeouts(now);
    link = lastEvent(VENDOR_LINK_STATUS);
    EXPECT_EQ(link->value.int32Values[0] & LINK_BMS, LINK_BMS);
}

// Charging: standstill + charge current. Regen (moving, negative current)
// must NOT register as charging.
TEST_F(MotorcycleVehicleHardwareTest, ChargingDetectedAtStandstillOnly) {
    // rpm 0, current -8.0A (0xFFB0 -> raw -80)
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x00, 0x00, 0x00, 0xD0, 0x02, 0xB0, 0xFF}));
    auto charging = lastEvent(VENDOR_CHARGING);
    ASSERT_TRUE(charging.has_value());
    EXPECT_EQ(charging->value.int32Values[0], 1);

    // Moving with regen current: not charging
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x20, 0xB8, 0x0B, 0xD0, 0x02, 0xB0, 0xFF}));
    charging = lastEvent(VENDOR_CHARGING);
    EXPECT_EQ(charging->value.int32Values[0], 0);

    // Standstill drawing current (accessories/idle): not charging
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x00, 0x00, 0x00, 0xD0, 0x02, 0x14, 0x00}));
    charging = lastEvent(VENDOR_CHARGING);
    EXPECT_EQ(charging->value.int32Values[0], 0);
}

TEST_F(MotorcycleVehicleHardwareTest, GearChangeNotifiesOnlyOnChange) {
    auto driveFrame = makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                {0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    mPeer->processCanFrame(driveFrame);
    mPeer->processCanFrame(driveFrame);
    EXPECT_EQ(countEvents(PROP_CURRENT_GEAR), 1u);

    // Back to neutral (bits 4-7 = 2)
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
    EXPECT_EQ(countEvents(PROP_CURRENT_GEAR), 2u);
    auto gear = lastEvent(PROP_CURRENT_GEAR);
    EXPECT_EQ(gear->value.int32Values[0], static_cast<int32_t>(VehicleGear::GEAR_NEUTRAL));
}

// ============================================================================
// Range model. Driven with synthetic timestamps through the peer: 100ms steps
// at 20 m/s advance 2m per step, and 72V * 27.78A = 2000W consumes
// 0.0556 Wh per step, i.e. 27.78 Wh/km - a plausible motorway figure.
// ============================================================================

namespace {
constexpr int64_t kStepNs = 100000000LL;  // 100ms
constexpr float kTestSpeedMps = 20.0f;
constexpr float kTestVoltage = 72.0f;
constexpr float kTestCurrentA = 27.78f;   // 2000W at 72V -> 27.78 Wh/km at 20 m/s
}  // namespace

TEST_F(MotorcycleVehicleHardwareTest, RangeStaysUnknownUntilConsumptionLearned) {
    mPeer->setLastSoc(80.0f);
    mPeer->publishRange(1);
    auto range = lastEvent(PROP_RANGE);
    ASSERT_TRUE(range.has_value());
    EXPECT_FLOAT_EQ(range->value.floatValues[0], 0.0f);  // 0 = unknown, UI shows "--"
}

TEST_F(MotorcycleVehicleHardwareTest, RangeLearnsConsumptionAndProjects) {
    mPeer->setLastSoc(50.0f);
    // 500m at steady 2000W: two full 200m chunks complete and agree, so the
    // EMA equals the true consumption regardless of seeding order.
    int64_t t = 1;
    mPeer->accumulateDistance(kTestSpeedMps, t);  // establish time reference
    mPeer->accumulateEnergy(kTestVoltage, kTestCurrentA, kTestSpeedMps, t);
    for (int i = 0; i < 250; i++) {
        t += kStepNs;
        mPeer->accumulateDistance(kTestSpeedMps, t);
        mPeer->accumulateEnergy(kTestVoltage, kTestCurrentA, kTestSpeedMps, t);
    }
    auto range = lastEvent(PROP_RANGE);
    ASSERT_TRUE(range.has_value());
    // 50% of the 5292Wh default pack at 27.78 Wh/km = 95.2 km
    EXPECT_NEAR(range->value.floatValues[0], 95250.0f, 2500.0f);
}

TEST_F(MotorcycleVehicleHardwareTest, StandstillChargingDoesNotPoisonConsumption) {
    mPeer->setLastSoc(50.0f);
    int64_t t = 1;
    mPeer->accumulateDistance(kTestSpeedMps, t);
    mPeer->accumulateEnergy(kTestVoltage, kTestCurrentA, kTestSpeedMps, t);
    for (int i = 0; i < 250; i++) {
        t += kStepNs;
        mPeer->accumulateDistance(kTestSpeedMps, t);
        mPeer->accumulateEnergy(kTestVoltage, kTestCurrentA, kTestSpeedMps, t);
    }
    float learnedRange = lastEvent(PROP_RANGE)->value.floatValues[0];

    // Park and charge at -10A for 10 minutes: no distance, heavy negative
    // power. Must not leak into the consumption average.
    for (int i = 0; i < 6000; i++) {
        t += kStepNs;
        mPeer->accumulateDistance(0.0f, t);
        mPeer->accumulateEnergy(kTestVoltage, -10.0f, 0.0f, t);
    }
    // Ride one more chunk at the same consumption; the average must be intact.
    for (int i = 0; i < 105; i++) {
        t += kStepNs;
        mPeer->accumulateDistance(kTestSpeedMps, t);
        mPeer->accumulateEnergy(kTestVoltage, kTestCurrentA, kTestSpeedMps, t);
    }
    EXPECT_NEAR(lastEvent(PROP_RANGE)->value.floatValues[0], learnedRange,
                learnedRange * 0.05f);
}

TEST_F(MotorcycleVehicleHardwareTest, RegenChunkExtendsRange) {
    mPeer->setLastSoc(50.0f);
    int64_t t = 1;
    mPeer->accumulateDistance(kTestSpeedMps, t);
    mPeer->accumulateEnergy(kTestVoltage, kTestCurrentA, kTestSpeedMps, t);
    for (int i = 0; i < 250; i++) {
        t += kStepNs;
        mPeer->accumulateDistance(kTestSpeedMps, t);
        mPeer->accumulateEnergy(kTestVoltage, kTestCurrentA, kTestSpeedMps, t);
    }
    float drivingRange = lastEvent(PROP_RANGE)->value.floatValues[0];

    // A long downhill: regen while rolling. The net-negative chunk clamps to
    // zero consumption and pulls the EMA (and so the range) up, never down.
    for (int i = 0; i < 105; i++) {
        t += kStepNs;
        mPeer->accumulateDistance(kTestSpeedMps, t);
        mPeer->accumulateEnergy(kTestVoltage, -10.0f, kTestSpeedMps, t);
    }
    EXPECT_GT(lastEvent(PROP_RANGE)->value.floatValues[0], drivingRange);
}

TEST_F(MotorcycleVehicleHardwareTest, PackEnergyConfigAppliesAndValidates) {
    EXPECT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_PACK_ENERGY_WH, 4900.0f)),
              StatusCode::OK);
    auto cap = lastEvent(PROP_BATTERY_CAPACITY);
    ASSERT_TRUE(cap.has_value());
    EXPECT_FLOAT_EQ(cap->value.floatValues[0], 4900.0f);

    EXPECT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_PACK_ENERGY_WH, 100.0f)),
              StatusCode::INVALID_ARG);
    EXPECT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_PACK_ENERGY_WH, 99999.0f)),
              StatusCode::INVALID_ARG);
}

}  // namespace
}  // namespace android::hardware::automotive::vehicle::motorcycle
