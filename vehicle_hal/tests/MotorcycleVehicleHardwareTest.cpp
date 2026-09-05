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
#include "imu/SyntheticImu.h"

#include <gtest/gtest.h>

#include <cutils/properties.h>
#include <utils/SystemClock.h>

#include <linux/can.h>

#include <sys/socket.h>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include <optional>
#include <regex>
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
    void setCaptureDir(const std::string& dir) { mHw->mCaptureDir = dir; }
    void updateChargingState(int rpm, float current, int64_t ts) {
        mHw->updateChargingState(rpm, current, ts);
    }
    void setChargingDwell(int64_t ns) { mHw->mChargingDwellNs = ns; }
    void endRideIfDue(int64_t nowNs, bool linkDead) { mHw->endRideIfDue(nowNs, linkDead); }
    // IMU path without the thread: the caller is the clock and the sensor.
    void processImuSample(const imu::ImuSample& s, float tempC, int64_t ts) {
        mHw->processImuSample(s, tempC, ts);
    }
    void setImuMounting(const imu::Mounting& m) { mHw->mLean.setMounting(m); }
    void setImuPresent() { mHw->mImuSourceBits = IMU_STATUS_PRESENT; }
    const std::vector<VehiclePropConfig>& propertyConfigs() const { return mHw->mPropertyConfigs; }
    const VehiclePropValue& currentValue(int32_t prop) { return mHw->mCurrentValues[prop]; }
    void setLiveSpeed(float mps, int64_t ts) {
        mHw->mLastSpeedMps = mps;
        mHw->mLastControllerFrameNs = ts;
    }
    // Wire a socketpair in place of the CAN socket so tests can read frames
    // the HAL transmits (display reports).
    void setCanSocket(int fd) { mHw->mCanSocket = fd; }

  private:
    MotorcycleVehicleHardware* mHw;
};

namespace {

constexpr int32_t PROP_ENGINE_RPM = static_cast<int32_t>(VehicleProperty::ENGINE_RPM);
constexpr int32_t PROP_SPEED = static_cast<int32_t>(VehicleProperty::PERF_VEHICLE_SPEED);
constexpr int32_t PROP_CURRENT_GEAR = static_cast<int32_t>(VehicleProperty::CURRENT_GEAR);
constexpr int32_t PROP_PARKING_BRAKE =
        static_cast<int32_t>(VehicleProperty::PARKING_BRAKE_ON);
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
        property_set("persist.vendor.motodash.ride.seq", "");
        property_set("persist.vendor.motodash.cfg.pack_energy_wh", "");
        property_set("persist.vendor.motodash.cfg.units_speed", "");
        property_set("persist.vendor.motodash.cfg.units_distance", "");
        property_set("persist.vendor.motodash.cfg.units_temp", "");
        property_set("persist.vendor.motodash.cfg.can_capture", "");
        property_set("persist.vendor.motodash.cfg.gear_base", "");
        property_set("persist.vendor.motodash.cfg.pack_max_v", "");
        property_set("persist.vendor.motodash.imu.up", "");
        property_set("persist.vendor.motodash.imu.fwd", "");
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

TEST_F(MotorcycleVehicleHardwareTest, ParkingBrakeFollowsGear) {
    // The bike has no parking-brake switch: the HAL derives PARKING_BRAKE_ON
    // from the gear so CarDrivingStateService can initialize. P = on,
    // anything else = off; boot default is on (unknown = parked).

    // Gear D -> brake released (changes from the boot default of ON).
    auto drive = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                           {0x00, 0x30, 0x00, 0x00, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(drive);
    auto brake = lastEvent(PROP_PARKING_BRAKE);
    ASSERT_TRUE(brake.has_value());
    EXPECT_EQ(brake->value.int32Values[0], 0);

    // Gear P (upper nibble 0) -> brake on again.
    auto park = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                          {0x00, 0x00, 0x00, 0x00, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(park);
    brake = lastEvent(PROP_PARKING_BRAKE);
    ASSERT_TRUE(brake.has_value());
    EXPECT_EQ(brake->value.int32Values[0], 1);

    // Neutral -> off. N equals no boot-default change for the GEAR property,
    // but the brake must still drop: it is published outside the gear-change
    // branch precisely for this case.
    auto neutral = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                             {0x00, 0x20, 0x00, 0x00, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(neutral);
    brake = lastEvent(PROP_PARKING_BRAKE);
    ASSERT_TRUE(brake.has_value());
    EXPECT_EQ(brake->value.int32Values[0], 0);
}

TEST_F(MotorcycleVehicleHardwareTest, DisplayReportEncodesOdoTripSpeedAndCounter) {
    // The stock display answers the controller at 250 ms on 0x1026105A;
    // we must look the same or the controller flags a missing display.
    // Wire a socketpair in as the CAN socket and read back what the HAL
    // transmits when a controller frame arrives.
    int sv[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv), 0);
    mPeer->setCanSocket(sv[1]);

    // rpm 20000 -> 105 km/h unclamped? No: 20000*1.894/(4*60)=157.8 m/s
    // is 568 km/h, so the report's speed byte must clamp to 199.
    auto fast = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                          {0x00, 0x30, 0x20, 0x4E, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(fast);

    // The BMS poll thread also writes OBD2 requests into this socket, so
    // reads must filter for the display-report ID.
    auto readDisplayReport = [&](struct can_frame* out) {
        struct can_frame f;
        while (recv(sv[0], &f, sizeof(f), MSG_DONTWAIT) == (ssize_t)sizeof(f)) {
            if (f.can_id == (CAN_ID_DISPLAY_REPORT | CAN_EFF_FLAG)) {
                *out = f;
                return true;
            }
        }
        return false;
    };

    struct can_frame report;
    ASSERT_TRUE(readDisplayReport(&report));
    EXPECT_EQ(report.can_dlc, 8);
    // Fresh test instance: odometer and trip are zero.
    EXPECT_EQ(report.data[0], 0);  // odo low
    EXPECT_EQ(report.data[2], 0);  // odo high
    EXPECT_EQ(report.data[3], 0);  // trip low
    EXPECT_EQ(report.data[4], 0);  // trip high
    EXPECT_EQ(report.data[5], 199);  // speed clamped to the display's max
    uint8_t firstCounter = report.data[6];

    // A frame inside the 250 ms window must NOT produce a report.
    mPeer->processCanFrame(fast);
    struct can_frame none;
    EXPECT_FALSE(readDisplayReport(&none));

    // After the window, the next frame reports again with the counter bumped.
    usleep(260000);
    auto slow = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                          {0x00, 0x30, 0x54, 0x01, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(slow);  // rpm 340 -> 2.68 m/s -> 9 km/h
    ASSERT_TRUE(readDisplayReport(&report));
    EXPECT_EQ(report.data[5], 9);
    EXPECT_EQ(report.data[6], (uint8_t)(firstCounter + 1));

    mPeer->setCanSocket(-1);
    close(sv[0]);
    close(sv[1]);
}

TEST_F(MotorcycleVehicleHardwareTest, CaptureToggleClosesAndReopensFiles) {
    // The Rider settings switch flips VENDOR_CFG_CAN_CAPTURE live. Off must
    // close the log (so a pull mid-session gets complete data); on again
    // must start a NEW file rather than corrupt the old one.
    char dirTemplate[] = "/tmp/motocapXXXXXX";
    ASSERT_NE(mkdtemp(dirTemplate), nullptr);
    mPeer->setCaptureDir(dirTemplate);

    auto countLogs = [&]() {
        int n = 0;
        DIR* d = opendir(dirTemplate);
        if (!d) return -1;
        while (auto* e = readdir(d)) {
            if (strncmp(e->d_name, "can-", 4) == 0) n++;
        }
        closedir(d);
        return n;
    };

    ASSERT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_CAPTURE, 1)),
              StatusCode::OK);
    auto frame = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                           {0x00, 0x30, 0xB8, 0x0B, 0xD0, 0x02, 0xFF, 0x00});
    mPeer->processCanFrame(frame);
    EXPECT_EQ(countLogs(), 1);

    ASSERT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_CAPTURE, 0)),
              StatusCode::OK);
    mPeer->processCanFrame(frame);  // capture off: frame must not reopen a file
    EXPECT_EQ(countLogs(), 1);

    sleep(1);  // candump filenames carry epoch seconds; force a distinct name
    ASSERT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_CAPTURE, 1)),
              StatusCode::OK);
    mPeer->processCanFrame(frame);
    EXPECT_EQ(countLogs(), 2);
}

TEST_F(MotorcycleVehicleHardwareTest, GearBaseOffsetRemapsGears) {
    // Spec ambiguity: the bike may report 1=P..4=D instead of 0=P..3=D.
    // The Workshop-settable gear base must remap live, and the parking
    // brake must follow the REMAPPED gear.
    ASSERT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_GEAR_BASE, 1)),
              StatusCode::OK);

    // Raw nibble 4 with base 1 -> D.
    auto d = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                       {0x00, 0x40, 0x00, 0x00, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(d);
    auto gear = lastEvent(PROP_CURRENT_GEAR);
    ASSERT_TRUE(gear.has_value());
    EXPECT_EQ(gear->value.int32Values[0], static_cast<int32_t>(VehicleGear::GEAR_DRIVE));
    auto brake = lastEvent(PROP_PARKING_BRAKE);
    ASSERT_TRUE(brake.has_value());
    EXPECT_EQ(brake->value.int32Values[0], 0);

    // Raw nibble 1 with base 1 -> P, and the brake comes on.
    auto p = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                       {0x00, 0x10, 0x00, 0x00, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(p);
    gear = lastEvent(PROP_CURRENT_GEAR);
    ASSERT_TRUE(gear.has_value());
    EXPECT_EQ(gear->value.int32Values[0], static_cast<int32_t>(VehicleGear::GEAR_PARK));
    brake = lastEvent(PROP_PARKING_BRAKE);
    ASSERT_TRUE(brake.has_value());
    EXPECT_EQ(brake->value.int32Values[0], 1);

    // Out-of-range base is refused.
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_GEAR_BASE, 2)),
              StatusCode::INVALID_ARG);
}

TEST_F(MotorcycleVehicleHardwareTest, RawGearStatusBytePublished) {
    // The Workshop screen shows byte1 verbatim so gear/mode discrepancies
    // can be identified on the bike without a laptop.
    auto frame = makeFrame(CAN_ID_CONTROLLER_STATUS, /*extended=*/true,
                           {0x00, 0x35, 0x00, 0x00, 0xD0, 0x02, 0x00, 0x00});
    mPeer->processCanFrame(frame);
    auto raw = lastEvent(VENDOR_RAW_GEAR_STATUS);
    ASSERT_TRUE(raw.has_value());
    EXPECT_EQ(raw->value.int32Values[0], 0x35);
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

// BMS broadcast 0x6B1 = Orion's default second message: discharge/charge
// current limits, high/low cell temperature, checksum. Raw frame as seen on
// the bike 2026-09-05.
TEST_F(MotorcycleVehicleHardwareTest, DecodesOrionBmsBroadcast) {
    mPeer->processCanFrame(makeFrame(CAN_ID_BMS, false, {0x00, 0x60, 0x00, 0x1B, 0x13, 0x11, 0x00, 0x58}));
    EXPECT_FLOAT_EQ(lastEvent(VENDOR_DISCHARGE_LIMIT)->value.floatValues[0], 96.0f);
    EXPECT_FLOAT_EQ(lastEvent(VENDOR_CHARGE_LIMIT)->value.floatValues[0], 27.0f);
    EXPECT_FLOAT_EQ(lastEvent(VENDOR_PACK_TEMP_HIGH)->value.floatValues[0], 19.0f);
    EXPECT_FLOAT_EQ(lastEvent(VENDOR_PACK_TEMP_LOW)->value.floatValues[0], 17.0f);
    EXPECT_FLOAT_EQ(lastEvent(VENDOR_PACK_TEMP_AVG)->value.floatValues[0], 18.0f);
    // The old guess read byte 3 as SoC; it must not touch the battery level.
    EXPECT_FALSE(lastEvent(PROP_EV_BATTERY_LEVEL).has_value());
    auto link = lastEvent(VENDOR_LINK_STATUS);
    ASSERT_TRUE(link.has_value());
    EXPECT_NE(link->value.int32Values[0] & LINK_BMS, 0);

    // A corrupted frame (bad checksum) is ignored entirely.
    clearEvents();
    mPeer->processCanFrame(makeFrame(CAN_ID_BMS, false, {0x00, 0x60, 0x00, 0x1B, 0x13, 0x11, 0x00, 0x57}));
    EXPECT_FALSE(lastEvent(VENDOR_DISCHARGE_LIMIT).has_value());
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
    // PID 0xF032 low cell voltage, raw 36500 (0x8E94, big-endian as OBD2 data
    // is) => 3.65 V
    auto frame = makeFrame(CAN_ID_OBD2_RESPONSE, false,
                           {0x05, 0x62, 0xF0, 0x32, 0x8E, 0x94, 0, 0});
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
    mPeer->processCanFrame(makeFrame(CAN_ID_BMS, false, {0x00, 0x60, 0x00, 0x1B, 0x13, 0x11, 0x00, 0x58}));
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
    // Charging needs standstill + charge current SUSTAINED for the dwell:
    // one frame must not flip it.
    int64_t t0 = 1000000000LL;
    mPeer->updateChargingState(0, -8.0f, t0);
    auto charging = lastEvent(VENDOR_CHARGING);
    if (charging.has_value()) EXPECT_EQ(charging->value.int32Values[0], 0);

    // Held for 6 s: now it is a charger, not a regen tail.
    mPeer->updateChargingState(0, -8.0f, t0 + 6000000000LL);
    charging = lastEvent(VENDOR_CHARGING);
    ASSERT_TRUE(charging.has_value());
    EXPECT_EQ(charging->value.int32Values[0], 1);

    // Moving with regen current: charging drops instantly.
    mPeer->updateChargingState(3000, -25.0f, t0 + 7000000000LL);
    charging = lastEvent(VENDOR_CHARGING);
    EXPECT_EQ(charging->value.int32Values[0], 0);

    // Standstill drawing current (accessories/idle): never a candidate,
    // no matter how long it holds.
    mPeer->updateChargingState(0, 2.0f, t0 + 8000000000LL);
    mPeer->updateChargingState(0, 2.0f, t0 + 30000000000LL);
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

TEST_F(MotorcycleVehicleHardwareTest, DisplayUnitsApplyAndValidate) {
    const int32_t speedProp =
            static_cast<int32_t>(VehicleProperty::VEHICLE_SPEED_DISPLAY_UNITS);
    constexpr int32_t kMph = 0x90;   // VehicleUnit::MILES_PER_HOUR
    constexpr int32_t kFahrenheit = 0x31;

    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(speedProp, kMph)), StatusCode::OK);
    auto speed = lastEvent(speedProp);
    ASSERT_TRUE(speed.has_value());
    EXPECT_EQ(speed->value.int32Values[0], kMph);

    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_TEMP_DISPLAY_UNITS, kFahrenheit)),
              StatusCode::OK);

    // Anything outside the supported pair is rejected
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(speedProp, 0x21 /* METER */)),
              StatusCode::INVALID_ARG);
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

// ============================================================================
// BMS OBD2: authoritative SoC / current, big-endian payloads
// ============================================================================

TEST_F(MotorcycleVehicleHardwareTest, SocComesOnlyFromThePid) {
    // The broadcast carries no SoC (it is the current-limit message).
    mPeer->processCanFrame(makeFrame(CAN_ID_BMS, false, {0x00, 0x60, 0x00, 0x1B, 0x13, 0x11, 0x00, 0x58}));
    EXPECT_FALSE(lastEvent(PROP_EV_BATTERY_LEVEL).has_value());

    // 0xF00F answers 61.5% (raw 123 at 0.5%/bit) on Orion's 0x7EB.
    mPeer->processCanFrame(makeFrame(CAN_ID_OBD2_RESPONSE, false,
                                     {0x04, 0x62, 0xF0, 0x0F, 123, 0, 0, 0}));
    EXPECT_FLOAT_EQ(lastEvent(PROP_EV_BATTERY_LEVEL)->value.floatValues[0], 61.5f);
    EXPECT_EQ(CAN_ID_OBD2_RESPONSE, 0x7EBu);
    EXPECT_EQ(CAN_ID_OBD2_REQUEST, 0x7E3u);

    // A following broadcast leaves it alone.
    clearEvents();
    mPeer->processCanFrame(makeFrame(CAN_ID_BMS, false, {0x00, 0x60, 0x00, 0x1B, 0x13, 0x11, 0x00, 0x58}));
    EXPECT_FALSE(lastEvent(PROP_EV_BATTERY_LEVEL).has_value());
}

TEST_F(MotorcycleVehicleHardwareTest, PackAmphoursAreBigEndian) {
    // 0xF010, 68.3Ah = raw 683 = 0x02AB, MSB first
    mPeer->processCanFrame(makeFrame(CAN_ID_OBD2_RESPONSE, false,
                                     {0x05, 0x62, 0xF0, 0x10, 0x02, 0xAB, 0, 0}));
    EXPECT_FLOAT_EQ(lastEvent(VENDOR_PACK_AMPHOURS)->value.floatValues[0], 68.3f);
}

TEST_F(MotorcycleVehicleHardwareTest, BmsSignedCurrentDrivesChargingDetection) {
    // This test is about SOURCE SELECTION (fresh BMS sign outranks the
    // controller), not the dwell - collapse the dwell to zero.
    mPeer->setChargingDwell(0);
    // BMS reports -6.0A (charging): raw -60 = 0xFFC4 big-endian
    mPeer->processCanFrame(makeFrame(CAN_ID_OBD2_RESPONSE, false,
                                     {0x05, 0x62, 0xF0, 0x0C, 0xFF, 0xC4, 0, 0}));
    EXPECT_FLOAT_EQ(lastEvent(VENDOR_PACK_CURRENT)->value.floatValues[0], -6.0f);

    // Controller frame at standstill reporting an unsigned-looking +6.0A: the
    // fresh BMS sign wins and charging is detected.
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x20, 0x00, 0x00, 0xD0, 0x02, 0x3C, 0x00}));
    auto charging = lastEvent(VENDOR_CHARGING);
    ASSERT_TRUE(charging.has_value());
    EXPECT_EQ(charging->value.int32Values[0], 1);
}

// ============================================================================
// Ride summary
// ============================================================================

TEST_F(MotorcycleVehicleHardwareTest, RideSummaryPublishedAtKeyOff) {
    // 100 s at 15 m/s = 1500 m, drawing 72 V x 45 A = 3240 W -> 90 Wh -> 60 Wh/km.
    // Stepped at the controller's real 20 Hz cadence: accumulateDistance
    // ignores gaps wider than kMaxDistanceStepNs.
    int64_t t = 1000000000LL;
    for (int i = 0; i < 2000; i++) {
        t += 50000000LL;
        mPeer->accumulateDistance(15.0f, t);
        mPeer->accumulateEnergy(72.0f, 45.0f, 15.0f, t);
    }
    EXPECT_FALSE(lastEvent(VENDOR_RIDE_SEQ).has_value() &&
                 lastEvent(VENDOR_RIDE_SEQ)->value.int32Values[0] > 0);

    // Key off: the controller link dies.
    mPeer->endRideIfDue(t + 2000000000LL, /*linkDead=*/true);

    auto seq = lastEvent(VENDOR_RIDE_SEQ);
    ASSERT_TRUE(seq.has_value());
    EXPECT_EQ(seq->value.int32Values[0], 1);
    EXPECT_NEAR(lastEvent(VENDOR_RIDE_DISTANCE_M)->value.floatValues[0], 1500.0f, 20.0f);
    EXPECT_NEAR(lastEvent(VENDOR_RIDE_DURATION_S)->value.floatValues[0], 99.0f, 2.0f);
    EXPECT_NEAR(lastEvent(VENDOR_RIDE_WH_PER_KM)->value.floatValues[0], 60.0f, 2.0f);
    EXPECT_FLOAT_EQ(lastEvent(VENDOR_RIDE_MAX_SPEED_MPS)->value.floatValues[0], 15.0f);

    // A second key-off without a new ride publishes nothing more.
    mPeer->endRideIfDue(t + 4000000000LL, true);
    EXPECT_EQ(lastEvent(VENDOR_RIDE_SEQ)->value.int32Values[0], 1);
}

TEST_F(MotorcycleVehicleHardwareTest, ShortShuffleIsNotARide) {
    // 20 s at 5 m/s = 100 m (moving the bike in the garage): no summary.
    int64_t t = 1000000000LL;
    for (int i = 0; i < 400; i++) {
        t += 50000000LL;
        mPeer->accumulateDistance(5.0f, t);
    }
    mPeer->endRideIfDue(t + 2000000000LL, true);
    auto seq = lastEvent(VENDOR_RIDE_SEQ);
    EXPECT_FALSE(seq.has_value() && seq->value.int32Values[0] > 0);
}

TEST_F(MotorcycleVehicleHardwareTest, RideEndsAfterLongStandstill) {
    int64_t t = 1000000000LL;
    for (int i = 0; i < 1200; i++) {
        t += 50000000LL;
        mPeer->accumulateDistance(10.0f, t);   // 600 m
    }
    // Parked with the link alive: not over yet at 4 min, over at 6 min.
    mPeer->endRideIfDue(t + 240LL * 1000000000LL, false);
    auto seq = lastEvent(VENDOR_RIDE_SEQ);
    EXPECT_FALSE(seq.has_value() && seq->value.int32Values[0] > 0);
    mPeer->endRideIfDue(t + 360LL * 1000000000LL, false);
    seq = lastEvent(VENDOR_RIDE_SEQ);
    ASSERT_TRUE(seq.has_value());
    EXPECT_EQ(seq->value.int32Values[0], 1);
    EXPECT_NEAR(lastEvent(VENDOR_RIDE_DISTANCE_M)->value.floatValues[0], 600.0f, 15.0f);
}

TEST_F(MotorcycleVehicleHardwareTest, RegenStopDoesNotFlashCharging) {
    // Braking to a stop with regen: the wheel stops a beat before the charge
    // current dies. Christian watched this flash the garage takeover on the
    // simulator - the exact sequence must never declare charging.
    int64_t t0 = 1000000000LL;
    mPeer->updateChargingState(500, -25.0f, t0);              // rolling regen
    mPeer->updateChargingState(0, -10.0f, t0 + 200000000LL);  // wheel just stopped
    mPeer->updateChargingState(0, -4.0f, t0 + 700000000LL);   // tail dying
    mPeer->updateChargingState(0, 0.0f, t0 + 1200000000LL);   // gone
    auto charging = lastEvent(VENDOR_CHARGING);
    if (charging.has_value()) EXPECT_EQ(charging->value.int32Values[0], 0);

    // Sitting at the light afterwards (small accessory draw): still nothing.
    mPeer->updateChargingState(0, 1.5f, t0 + 30000000000LL);
    charging = lastEvent(VENDOR_CHARGING);
    if (charging.has_value()) EXPECT_EQ(charging->value.int32Values[0], 0);
}

// ============================================================================
// CAN capture
// ============================================================================

TEST_F(MotorcycleVehicleHardwareTest, CaptureWritesCandumpFormat) {
    char dirTemplate[] = "/tmp/motocapXXXXXX";
    ASSERT_NE(mkdtemp(dirTemplate), nullptr);
    std::string dir = dirTemplate;
    mPeer->setCaptureDir(dir);

    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_CAPTURE, 1)), StatusCode::OK);
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x30, 0xE6, 0x09, 0xD0, 0x02, 0x16, 0x01}));
    mPeer->processCanFrame(makeFrame(CAN_ID_BMS, false, {0, 99, 0, 50, 3, 2, 0, 65}));
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_CAPTURE, 0)), StatusCode::OK);

    std::string path;
    if (DIR* d = opendir(dir.c_str())) {
        while (struct dirent* e = readdir(d)) {
            if (std::string(e->d_name).rfind("can-", 0) == 0) path = dir + "/" + e->d_name;
        }
        closedir(d);
    }
    ASSERT_FALSE(path.empty()) << "no capture file in " << dir;
    std::ifstream in(path);
    std::string l1, l2;
    ASSERT_TRUE(std::getline(in, l1));
    ASSERT_TRUE(std::getline(in, l2));
    // candump -l: "(sec.usec) iface ID#DATA"; extended IDs 8 hex digits
    std::regex ext(R"(^\(\d+\.\d{6}\) \S+ 10261022#0030E609D0021601$)");
    std::regex sff(R"(^\(\d+\.\d{6}\) \S+ 6B1#0063003203020041$)");
    EXPECT_TRUE(std::regex_match(l1, ext)) << l1;
    EXPECT_TRUE(std::regex_match(l2, sff)) << l2;

    // Disabled: nothing more is written
    mPeer->processCanFrame(makeFrame(CAN_ID_BMS, false, {0, 99, 0, 50, 3, 2, 0, 65}));
    std::ifstream again(path);
    int lines = 0; std::string tmp;
    while (std::getline(again, tmp)) lines++;
    EXPECT_EQ(lines, 2);
}

}  // namespace
// ---------------------------------------------------------------------------
// Inertial sensing through the HAL (estimator itself: tests/ImuTest.cpp)
// ---------------------------------------------------------------------------

TEST_F(MotorcycleVehicleHardwareTest, ImuCornerPublishesLeanAndRideMaxLean) {
    using namespace imu;
    mPeer->setImuPresent();
    mPeer->setImuMounting(Mounting::identity());
    SyntheticImu sim;
    Scenario sc;
    sc.speedMps = 15.0f;
    int64_t t = 1000000000LL;
    // Ride: 40 s straight, 10 s in a 30 deg right-hander, 10 s straight.
    for (int i = 0; i < 6000; i++) {
        t += 10000000LL;  // 100 Hz
        if (i % 5 == 0) mPeer->accumulateDistance(sc.speedMps, t);  // controller at 20 Hz
        mPeer->setLiveSpeed(sc.speedMps, t);
        if (i == 4000) sc.leanDeg = 30.0f;
        if (i == 5000) sc.leanDeg = 0.0f;
        mPeer->processImuSample(sim.sample(sc), 30.0f, t);
        if (i == 4900) {
            auto lean = lastEvent(VENDOR_LEAN_DEG);
            ASSERT_TRUE(lean.has_value());
            EXPECT_NEAR(lean->value.floatValues[0], 30.0f, 1.5f);
            auto lat = lastEvent(VENDOR_LAT_G);
            ASSERT_TRUE(lat.has_value());
            EXPECT_GT(lat->value.floatValues[0], 0.5f);
        }
    }
    auto status = lastEvent(VENDOR_IMU_STATUS);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->value.int32Values[0] & (IMU_STATUS_PRESENT | IMU_STATUS_LEVEL_SET |
                                              IMU_STATUS_FORWARD_SET | IMU_STATUS_VALID),
              IMU_STATUS_PRESENT | IMU_STATUS_LEVEL_SET | IMU_STATUS_FORWARD_SET | IMU_STATUS_VALID);
    auto raw = lastEvent(VENDOR_IMU_RAW);
    ASSERT_TRUE(raw.has_value());
    EXPECT_EQ(raw->value.floatValues.size(), 6u);

    // Key off: the summary carries the deepest lean, right side only.
    mPeer->endRideIfDue(t + 2000000000LL, true);
    auto seq = lastEvent(VENDOR_RIDE_SEQ);
    ASSERT_TRUE(seq.has_value() && seq->value.int32Values[0] > 0);
    auto maxR = lastEvent(VENDOR_RIDE_MAX_LEAN_R);
    auto maxL = lastEvent(VENDOR_RIDE_MAX_LEAN_L);
    ASSERT_TRUE(maxR.has_value() && maxL.has_value());
    EXPECT_NEAR(maxR->value.floatValues[0], 30.0f, 1.5f);
    EXPECT_LT(maxL->value.floatValues[0], 1.0f);
}

TEST_F(MotorcycleVehicleHardwareTest, SideStandLeanIsNotARideStatistic) {
    using namespace imu;
    mPeer->setImuPresent();
    mPeer->setImuMounting(Mounting::identity());
    SyntheticImu sim;
    int64_t t = 1000000000LL;
    // Ride 600 m, then park on the stand (12 deg left, speed 0) for 10 s.
    for (int i = 0; i < 1200; i++) {
        t += 50000000LL;
        mPeer->accumulateDistance(10.0f, t);
    }
    Scenario stand;
    stand.leanDeg = -12.0f;
    for (int i = 0; i < 1000; i++) {
        t += 10000000LL;
        mPeer->setLiveSpeed(0.0f, t);
        mPeer->processImuSample(sim.sample(stand), 30.0f, t);
    }
    auto lean = lastEvent(VENDOR_LEAN_DEG);
    ASSERT_TRUE(lean.has_value());
    EXPECT_NEAR(lean->value.floatValues[0], -12.0f, 1.0f);  // reported live...
    mPeer->endRideIfDue(t + 1000000000LL, true);
    auto maxL = lastEvent(VENDOR_RIDE_MAX_LEAN_L);
    ASSERT_TRUE(maxL.has_value());
    EXPECT_LT(maxL->value.floatValues[0], 0.5f);            // ...but not a ride statistic
}

TEST_F(MotorcycleVehicleHardwareTest, ImuLevelCommandCapturesUpAndPersists) {
    using namespace imu;
    mPeer->setImuPresent();
    Mounting truth;
    truth.setUp(Vec3(0.2f, -0.3f, 0.93f));
    truth.setForward(Vec3(0.9f, 0.4f, 0.0f));
    SyntheticImu sim(truth);
    Scenario still;
    int64_t t = 1000000000LL;

    // Uncalibrated: status says so, nothing valid.
    mPeer->processImuSample(sim.sample(still), 30.0f, t);
    auto status = lastEvent(VENDOR_IMU_STATUS);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->value.int32Values[0] & IMU_STATUS_LEVEL_SET, 0);

    // Level: 1 s of upright stillness after the Workshop button.
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_IMU_LEVEL, 1)), StatusCode::OK);
    for (int i = 0; i < 120; i++) {
        t += 10000000LL;
        mPeer->processImuSample(sim.sample(still), 30.0f, t);
    }
    status = lastEvent(VENDOR_IMU_STATUS);
    ASSERT_TRUE(status.has_value());
    EXPECT_NE(status->value.int32Values[0] & IMU_STATUS_LEVEL_SET, 0);
    EXPECT_EQ(status->value.int32Values[0] & IMU_STATUS_FORWARD_SET, 0);
    char up[PROPERTY_VALUE_MAX];
    ASSERT_GT(property_get("persist.vendor.motodash.imu.up", up, ""), 0);
    float ux, uy, uz;
    ASSERT_EQ(sscanf(up, "%f,%f,%f", &ux, &uy, &uz), 3);
    EXPECT_GT(Vec3(ux, uy, uz).dot(truth.up), 0.9999f);

    // Forward learns itself from a straight pull-away and is persisted too.
    Scenario go;
    go.longAccelMps2 = 2.0f;
    for (int i = 0; i < 900; i++) {
        t += 10000000LL;
        go.speedMps = 2.0f * (i + 1) * 0.01f;
        mPeer->setLiveSpeed(go.speedMps, t);
        mPeer->processImuSample(sim.sample(go), 30.0f, t);
    }
    status = lastEvent(VENDOR_IMU_STATUS);
    ASSERT_TRUE(status.has_value());
    EXPECT_NE(status->value.int32Values[0] & IMU_STATUS_FORWARD_SET, 0);
    char fwd[PROPERTY_VALUE_MAX];
    EXPECT_GT(property_get("persist.vendor.motodash.imu.fwd", fwd, ""), 0);

    // Clear wipes both.
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_IMU_LEVEL, 0)), StatusCode::OK);
    t += 10000000LL;
    mPeer->processImuSample(sim.sample(still), 30.0f, t);
    status = lastEvent(VENDOR_IMU_STATUS);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->value.int32Values[0] & (IMU_STATUS_LEVEL_SET | IMU_STATUS_FORWARD_SET), 0);
    EXPECT_EQ(property_get("persist.vendor.motodash.imu.up", up, ""), 0);
}

TEST_F(MotorcycleVehicleHardwareTest, ImuLevelRejectedWhileMoving) {
    using namespace imu;
    mPeer->setImuPresent();
    SyntheticImu sim;
    Scenario turning;
    int64_t t = 1000000000LL;
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_IMU_LEVEL, 1)), StatusCode::OK);
    for (int i = 0; i < 120; i++) {
        t += 10000000LL;
        ImuSample s = sim.sample(turning);
        s.gyroDps.z = 25.0f;  // someone is wheeling the bike round
        mPeer->processImuSample(s, 30.0f, t);
    }
    auto status = lastEvent(VENDOR_IMU_STATUS);
    ASSERT_TRUE(status.has_value());
    EXPECT_NE(status->value.int32Values[0] & IMU_STATUS_LEVEL_FAILED, 0);
    EXPECT_EQ(status->value.int32Values[0] & IMU_STATUS_LEVEL_SET, 0);

    // Riding along: refused outright, even with a perfectly quiet sensor
    // (the flag simply stays up; the status does not need to re-fire).
    Scenario straight;
    straight.speedMps = 10.0f;
    EXPECT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_IMU_LEVEL, 1)), StatusCode::OK);
    for (int i = 0; i < 120; i++) {
        t += 10000000LL;
        mPeer->setLiveSpeed(10.0f, t);
        mPeer->processImuSample(sim.sample(straight), 30.0f, t);
    }
    status = lastEvent(VENDOR_IMU_STATUS);
    ASSERT_TRUE(status.has_value());
    EXPECT_NE(status->value.int32Values[0] & IMU_STATUS_LEVEL_FAILED, 0);
    EXPECT_EQ(status->value.int32Values[0] & IMU_STATUS_LEVEL_SET, 0);
}

TEST_F(MotorcycleVehicleHardwareTest, ImuCaptureFollowsTheCanCaptureSwitch) {
    using namespace imu;
    char dirTemplate[] = "/tmp/motodash-imucap-XXXXXX";
    ASSERT_NE(mkdtemp(dirTemplate), nullptr);
    std::string dir = dirTemplate;
    mPeer->setCaptureDir(dir);
    mPeer->setImuPresent();
    mPeer->setImuMounting(Mounting::identity());
    SyntheticImu sim;
    Scenario sc;
    sc.speedMps = 12.0f;
    int64_t t = 1000000000LL;
    auto feed = [&](int n) {
        for (int i = 0; i < n; i++) {
            t += 10000000LL;
            mPeer->setLiveSpeed(sc.speedMps, t);
            mPeer->processImuSample(sim.sample(sc), 30.0f, t);
        }
    };
    auto imuLogs = [&]() {
        std::vector<std::string> names;
        if (DIR* d = opendir(dir.c_str())) {
            while (struct dirent* e = readdir(d)) {
                if (strncmp(e->d_name, "imu-", 4) == 0) names.push_back(e->d_name);
            }
            closedir(d);
        }
        return names;
    };

    feed(50);
    EXPECT_TRUE(imuLogs().empty());  // switch off: nothing written

    ASSERT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_CAPTURE, 1)), StatusCode::OK);
    feed(250);
    auto logs = imuLogs();
    ASSERT_EQ(logs.size(), 1u);
    ImuLogReader reader;
    ASSERT_TRUE(reader.open(dir + "/" + logs[0]));
    ImuLogRecord r;
    ImuLogBaro b;
    bool isBaro;
    int n = 0;
    double lastT = -1;
    while (reader.next(&r, &b, &isBaro)) {
        ASSERT_FALSE(isBaro);
        EXPECT_NEAR(r.sample.accelG.z, 1.0f, 1e-4f);
        EXPECT_FLOAT_EQ(r.speedMps, 12.0f);
        EXPECT_TRUE(r.speedValid);
        if (lastT >= 0) EXPECT_NEAR(r.tS - lastT, 0.010, 1e-4);
        lastT = r.tS;
        n++;
    }
    EXPECT_EQ(n, 250);

    ASSERT_EQ(mPeer->applyConfigValue(makeIntValue(VENDOR_CFG_CAN_CAPTURE, 0)), StatusCode::OK);
    feed(50);
    struct stat st1;
    ASSERT_EQ(stat((dir + "/" + logs[0]).c_str(), &st1), 0);
    feed(50);
    struct stat st2;
    ASSERT_EQ(stat((dir + "/" + logs[0]).c_str(), &st2), 0);
    EXPECT_EQ(st1.st_size, st2.st_size);  // closed: no more growth
    EXPECT_EQ(imuLogs().size(), 1u);

    for (const auto& name : imuLogs()) unlink((dir + "/" + name).c_str());
    rmdir(dir.c_str());
}

// Controller status byte 1 as mapped on the bike 2026-09-05: gear in bits
// 4-5, ride mode in bits 6-7, independent of each other.
TEST_F(MotorcycleVehicleHardwareTest, GearAndRideModeShareTheStatusNibble) {
    struct Case { uint8_t byte1; int gear; int mode; };
    const Case cases[] = {
            {0x00, static_cast<int>(VehicleGear::GEAR_PARK), 0},
            {0x10, static_cast<int>(VehicleGear::GEAR_REVERSE), 0},
            {0x30, static_cast<int>(VehicleGear::GEAR_DRIVE), 0},
            {0x70, static_cast<int>(VehicleGear::GEAR_DRIVE), 1},
            {0xB0, static_cast<int>(VehicleGear::GEAR_DRIVE), 2},
            {0xF0, static_cast<int>(VehicleGear::GEAR_DRIVE), DRIVE_MODE_SPORT},
            {0x80, static_cast<int>(VehicleGear::GEAR_PARK), 2},
            {0x32, static_cast<int>(VehicleGear::GEAR_DRIVE), 0},   // brake bit in the low nibble
    };
    for (const auto& c : cases) {
        // Gear and mode publish on change only, so lastEvent() carries over
        // between cases whose value is unchanged - which is what we assert.
        mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                         {0x00, c.byte1, 0x00, 0x00, 0x5F, 0x03, 0xEF, 0xFF}));
        auto gear = lastEvent(PROP_CURRENT_GEAR);
        ASSERT_TRUE(gear.has_value()) << "byte1 " << (int)c.byte1;
        EXPECT_EQ(gear->value.int32Values[0], c.gear) << "byte1 " << (int)c.byte1;
        auto mode = lastEvent(VENDOR_DRIVE_MODE);
        if (mode.has_value()) {
            EXPECT_EQ(mode->value.int32Values[0], c.mode) << "byte1 " << (int)c.byte1;
        } else {
            EXPECT_EQ(c.mode, 0) << "no mode event for byte1 " << (int)c.byte1;
        }
    }
    // The brake bit reaches STATUS_FLAGS.
    auto flags = lastEvent(VENDOR_STATUS_FLAGS);
    ASSERT_TRUE(flags.has_value());
    EXPECT_NE(flags->value.int32Values[0] & STATUS_BRAKE, 0);
}

// The controller flags over-voltage on a full 21s pack (86.3 V); only a pack
// above its configured ceiling counts as a fault.
TEST_F(MotorcycleVehicleHardwareTest, OverVoltageFlagGatedByPackCeiling) {
    // Status frame: 86.3 V (0x035F LE at bytes 4-5), then temps frame with bit 1 of byte 6.
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x00, 0x00, 0x00, 0x5F, 0x03, 0xEF, 0xFF}));
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_TEMPS, true,
                                     {0x20, 0x13, 0x00, 0x05, 0x00, 0x00, 0x02, 0x00}));
    auto faults = lastEvent(VENDOR_FAULT_FLAGS);
    EXPECT_FALSE(faults.has_value() && (faults->value.int32Values[0] & FAULT_OVER_VOLTAGE));

    // 90.0 V (0x0384) is above the 89.25 V default: now it is a fault.
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_STATUS, true,
                                     {0x00, 0x00, 0x00, 0x00, 0x84, 0x03, 0x00, 0x00}));
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_TEMPS, true,
                                     {0x20, 0x13, 0x00, 0x05, 0x00, 0x00, 0x02, 0x00}));
    faults = lastEvent(VENDOR_FAULT_FLAGS);
    ASSERT_TRUE(faults.has_value());
    EXPECT_NE(faults->value.int32Values[0] & FAULT_OVER_VOLTAGE, 0);

    // Raising the ceiling from the Workshop clears it on the next frame.
    ASSERT_EQ(mPeer->applyConfigValue(makeFloatValue(VENDOR_CFG_PACK_MAX_VOLTAGE, 92.0f)),
              StatusCode::OK);
    mPeer->processCanFrame(makeFrame(CAN_ID_CONTROLLER_TEMPS, true,
                                     {0x20, 0x13, 0x00, 0x05, 0x00, 0x00, 0x02, 0x00}));
    faults = lastEvent(VENDOR_FAULT_FLAGS);
    ASSERT_TRUE(faults.has_value());
    EXPECT_EQ(faults->value.int32Values[0] & FAULT_OVER_VOLTAGE, 0);
}

// Every property id carries its value type in bits 16-23; CarService trusts
// that nibble, so a value filled as int32 under an INT64 id never reaches
// the UI (pack cycles showed 0 on the bike, 2026-09-05: 0x2150xxxx = INT64).
TEST_F(MotorcycleVehicleHardwareTest, PropertyIdTypeMatchesStoredValue) {
    constexpr int32_t kTypeMask = 0x00FF0000;
    constexpr int32_t kInt32 = 0x00400000, kInt64 = 0x00500000, kFloat = 0x00600000;
    constexpr int32_t kInt32Vec = 0x00410000, kFloatVec = 0x00610000, kBool = 0x00200000;
    for (const auto& cfg : mPeer->propertyConfigs()) {
        const auto& v = mPeer->currentValue(cfg.prop);
        int32_t type = cfg.prop & kTypeMask;
        char id[16];
        snprintf(id, sizeof(id), "0x%08X", cfg.prop);
        if (type == kFloat) {
            EXPECT_EQ(v.value.floatValues.size(), 1u) << id;
        } else if (type == kFloatVec) {
            EXPECT_GE(v.value.floatValues.size(), 1u) << id;
        } else if (type == kInt32 || type == kBool) {
            EXPECT_EQ(v.value.int32Values.size(), 1u) << id;
        } else if (type == kInt32Vec) {
            EXPECT_GE(v.value.int32Values.size(), 1u) << id;
        } else if (type == kInt64) {
            EXPECT_EQ(v.value.int64Values.size(), 1u) << id << " is INT64 but the HAL fills int32";
        } else {
            ADD_FAILURE() << id << " has an unexpected value type nibble";
        }
    }
}

}  // namespace android::hardware::automotive::vehicle::motorcycle
