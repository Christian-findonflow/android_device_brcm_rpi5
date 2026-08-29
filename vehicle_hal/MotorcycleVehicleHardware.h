/*
 * Copyright (C) 2024 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Custom Vehicle HAL hardware backend for electric motorcycle.
 * Reads CAN frames and maps to Android VehicleProperties.
 */

#pragma once

#include <IVehicleHardware.h>
#include <VehicleHalTypes.h>
#include <VehicleUtils.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/gpio.h>

namespace android::hardware::automotive::vehicle::motorcycle {

using ::aidl::android::hardware::automotive::vehicle::GetValueRequest;
using ::aidl::android::hardware::automotive::vehicle::GetValueResult;
using ::aidl::android::hardware::automotive::vehicle::SetValueRequest;
using ::aidl::android::hardware::automotive::vehicle::SetValueResult;
using ::aidl::android::hardware::automotive::vehicle::StatusCode;
using ::aidl::android::hardware::automotive::vehicle::SubscribeOptions;
using ::aidl::android::hardware::automotive::vehicle::VehicleAreaConfig;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropConfig;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropValue;
using ::aidl::android::hardware::automotive::vehicle::VehicleProperty;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyAccess;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyChangeMode;

// CAN IDs for electric motorcycle controller (extended 29-bit IDs)
constexpr uint32_t CAN_ID_CONTROLLER_STATUS = 0x10261022;  // RPM, Gear, Battery V/A, Errors
constexpr uint32_t CAN_ID_CONTROLLER_TEMPS = 0x10261023;   // Controller/Motor temps, Throttle
constexpr uint32_t CAN_ID_BMS = 0x6B1;                      // Battery SoC, Temp

// OBD2 CAN IDs for Orion BMS (standard 11-bit IDs)
constexpr uint32_t CAN_ID_OBD2_REQUEST = 0x7E0;            // OBD2 request to BMS
constexpr uint32_t CAN_ID_OBD2_RESPONSE = 0x7E8;           // OBD2 response from BMS

// Orion BMS OBD2 PIDs (Mode 0x22 - Extended Diagnostics)
constexpr uint16_t BMS_PID_PACK_CURRENT = 0xF00C;          // Signed pack current (0.1A)
constexpr uint16_t BMS_PID_PACK_VOLTAGE = 0xF00D;          // Pack voltage (0.1V)
constexpr uint16_t BMS_PID_PACK_SOC = 0xF00F;              // State of charge (0.5%)
constexpr uint16_t BMS_PID_PACK_SOH = 0xF013;              // State of health (%)
constexpr uint16_t BMS_PID_PACK_CYCLES = 0xF018;           // Total cycles
constexpr uint16_t BMS_PID_TEMP_HIGH = 0xF028;             // Highest pack temp (°C)
constexpr uint16_t BMS_PID_TEMP_LOW = 0xF029;              // Lowest pack temp (°C)
constexpr uint16_t BMS_PID_TEMP_AVG = 0xF02A;              // Average pack temp (°C)
constexpr uint16_t BMS_PID_HEATSINK_TEMP = 0xF02D;         // Heatsink temp (°C)
constexpr uint16_t BMS_PID_FAN_SPEED = 0xF02B;             // Fan speed (0-6)
constexpr uint16_t BMS_PID_CELL_LOW = 0xF032;              // Low cell voltage (0.0001V)
constexpr uint16_t BMS_PID_CELL_HIGH = 0xF033;             // High cell voltage (0.0001V)
constexpr uint16_t BMS_PID_CELL_AVG = 0xF034;              // Avg cell voltage (0.0001V)
constexpr uint16_t BMS_PID_CELL_LOW_ID = 0xF03E;           // Low cell ID
constexpr uint16_t BMS_PID_CELL_HIGH_ID = 0xF03D;          // High cell ID
constexpr uint16_t BMS_PID_CHARGE_LIMIT = 0xF00A;          // Charge current limit (A)
constexpr uint16_t BMS_PID_DISCHARGE_LIMIT = 0xF00B;       // Discharge current limit (A)
constexpr uint16_t BMS_PID_PACK_AMPHOURS = 0xF010;         // Pack Ah (0.1Ah)
constexpr uint16_t BMS_PID_PACK_RESISTANCE = 0xF011;       // Pack resistance (0.01mOhm)
constexpr uint16_t BMS_PID_PACK_DOD = 0xF012;              // Depth of discharge (0.5%)

// Vendor-specific property IDs for motorcycle data
// Format: 0x2[area_type][value_type][property_id]
// Area type: 1 = GLOBAL, Value type: 6 = FLOAT, 5 = INT32
// Using vendor range: 0x21000000 - 0x2FFFFFFF
constexpr int32_t VENDOR_BATTERY_VOLTAGE = 0x21600001;     // Battery voltage in volts
constexpr int32_t VENDOR_BATTERY_CURRENT = 0x21600002;     // Battery current in amps
constexpr int32_t VENDOR_THROTTLE_POSITION = 0x21600003;   // Throttle position 0-100% (as float)

// BMS properties (from Orion BMS OBD2 PIDs)
constexpr int32_t VENDOR_PACK_SOH = 0x21600010;            // Pack state of health (%)
constexpr int32_t VENDOR_PACK_TEMP_AVG = 0x21600011;       // Pack average temperature (°C)
constexpr int32_t VENDOR_CELL_VOLTAGE_LOW = 0x21600012;    // Lowest cell voltage (V)
constexpr int32_t VENDOR_CELL_VOLTAGE_HIGH = 0x21600013;   // Highest cell voltage (V)
constexpr int32_t VENDOR_PACK_TEMP_HIGH = 0x21600014;      // Pack highest temperature (°C)
constexpr int32_t VENDOR_PACK_TEMP_LOW = 0x21600015;       // Pack lowest temperature (°C)
constexpr int32_t VENDOR_PACK_CYCLES = 0x21500016;         // Total pack cycles (int)
constexpr int32_t VENDOR_PACK_AMPHOURS = 0x21600017;       // Pack capacity (Ah)
constexpr int32_t VENDOR_PACK_RESISTANCE = 0x21600018;     // Pack resistance (mOhm)
constexpr int32_t VENDOR_CHARGE_LIMIT = 0x21600019;        // Charge current limit (A)
constexpr int32_t VENDOR_DISCHARGE_LIMIT = 0x2160001A;     // Discharge current limit (A)
constexpr int32_t VENDOR_CELL_VOLTAGE_AVG = 0x2160001B;    // Average cell voltage (V)
constexpr int32_t VENDOR_CELL_LOW_ID = 0x2150001C;         // Low cell ID (int)
constexpr int32_t VENDOR_CELL_HIGH_ID = 0x2150001D;        // High cell ID (int)
constexpr int32_t VENDOR_FAN_SPEED = 0x2150001E;           // Fan speed 0-6 (int)
constexpr int32_t VENDOR_HEATSINK_TEMP = 0x2160001F;       // Heatsink temperature (°C)
constexpr int32_t VENDOR_PACK_DOD = 0x21600020;            // Depth of discharge (%)

// Fault flags from the controller, combined into one bitfield so the UI needs
// a single subscription. Low byte comes from 0x10261022 byte 0, second byte
// from 0x10261023 byte 6.
constexpr int32_t VENDOR_FAULT_FLAGS = 0x21400040;         // Bitfield (int32)

// Trip distance in km. READ_WRITE: writing any value sets the trip meter,
// so the UI resets it by writing 0.
constexpr int32_t VENDOR_TRIP_DISTANCE = 0x21600041;       // km (float)

// Fault bits, low byte: controller status frame (0x10261022 data[0])
constexpr int32_t FAULT_MOTOR = 1 << 0;
constexpr int32_t FAULT_HALL = 1 << 1;
constexpr int32_t FAULT_THROTTLE = 1 << 2;
constexpr int32_t FAULT_CONTROLLER = 1 << 3;
constexpr int32_t FAULT_BRAKE = 1 << 4;
constexpr int32_t FAULT_LIMP_HOME = 1 << 5;
// Fault bits, second byte: temperature frame (0x10261023 data[6])
constexpr int32_t FAULT_OVER_CURRENT = 1 << 8;
constexpr int32_t FAULT_OVER_VOLTAGE = 1 << 9;
constexpr int32_t FAULT_UNDER_VOLTAGE = 1 << 10;
constexpr int32_t FAULT_CONTROLLER_OVER_TEMP = 1 << 11;
constexpr int32_t FAULT_MOTOR_OVER_TEMP = 1 << 12;

constexpr int32_t FAULT_MASK_CONTROLLER_STATUS = 0x00FF;
constexpr int32_t FAULT_MASK_CONTROLLER_TEMPS = 0xFF00;

// Writable configuration properties. Set from the dashboard settings UI via
// CarPropertyManager (requires android.car.permission.CAR_VENDOR_EXTENSION);
// the HAL validates, applies, and persists them to persist.vendor.motodash.cfg.*.
// Value type nibble: 6 = FLOAT, 4 = INT32.
constexpr int32_t VENDOR_CFG_WHEEL_CIRCUMFERENCE = 0x21600030;   // meters (applies live)
constexpr int32_t VENDOR_CFG_GEAR_RATIO = 0x21600031;            // motor rev / wheel rev (applies live)
constexpr int32_t VENDOR_CFG_CAN_ID_CONTROLLER_STATUS = 0x21400032;  // CAN ID (applies live)
constexpr int32_t VENDOR_CFG_CAN_ID_CONTROLLER_TEMPS = 0x21400033;   // CAN ID (applies live)
constexpr int32_t VENDOR_CFG_CAN_ID_BMS = 0x21400034;                // CAN ID (applies live)
constexpr int32_t VENDOR_CFG_GPIO_LEFT_TURN = 0x21400035;    // BCM pin, -1 = disabled (needs reboot)
constexpr int32_t VENDOR_CFG_GPIO_RIGHT_TURN = 0x21400036;   // BCM pin, -1 = disabled (needs reboot)
constexpr int32_t VENDOR_CFG_GPIO_HIGH_BEAM = 0x21400037;    // BCM pin, -1 = disabled (needs reboot)
constexpr int32_t VENDOR_CFG_GPIO_ACTIVE_LOW = 0x21400038;   // 0/1 (applies live)

// Validation limits for writable configuration
constexpr float CFG_MIN_WHEEL_CIRCUMFERENCE_M = 0.5f;
constexpr float CFG_MAX_WHEEL_CIRCUMFERENCE_M = 5.0f;
constexpr float CFG_MIN_GEAR_RATIO = 0.5f;
constexpr float CFG_MAX_GEAR_RATIO = 30.0f;
constexpr int32_t CFG_MAX_CAN_ID = 0x1FFFFFFF;  // 29-bit extended ID space
constexpr int32_t CFG_MAX_GPIO_PIN = 27;        // RPi header BCM pins

// GPIO configuration (BCM pin numbers, -1 = disabled)
constexpr int GPIO_PIN_LEFT_TURN = -1;   // Will be read from system property
constexpr int GPIO_PIN_RIGHT_TURN = -1;
constexpr int GPIO_PIN_HIGH_BEAM = -1;
constexpr bool GPIO_ACTIVE_LOW = true;   // Optoisolator pulls low when active

// Gear values from controller
constexpr int GEAR_PARK = 0;
constexpr int GEAR_REVERSE = 1;
constexpr int GEAR_NEUTRAL = 2;
constexpr int GEAR_DRIVE = 3;

class MotorcycleVehicleHardware : public IVehicleHardware {
  public:
    // canInterfaceOverride: when non-empty, use this CAN interface instead of
    // the persist.vendor.motodash.can_interface property. Used by host-side
    // tests and the replay tool (e.g. "vcan0").
    explicit MotorcycleVehicleHardware(std::string canInterfaceOverride = "");
    ~MotorcycleVehicleHardware() override;

    std::vector<VehiclePropConfig> getAllPropertyConfigs() const override;

    StatusCode setValues(std::shared_ptr<const SetValuesCallback> callback,
                         const std::vector<SetValueRequest>& requests) override;

    StatusCode getValues(std::shared_ptr<const GetValuesCallback> callback,
                         const std::vector<GetValueRequest>& requests) const override;

    DumpResult dump(const std::vector<std::string>& options) override;

    StatusCode checkHealth() override;

    void registerOnPropertyChangeEvent(
            std::unique_ptr<const PropertyChangeCallback> callback) override;

    void registerOnPropertySetErrorEvent(
            std::unique_ptr<const PropertySetErrorCallback> callback) override;

    StatusCode subscribe(SubscribeOptions options) override;
    StatusCode unsubscribe(int32_t propId, int32_t areaId) override;

  private:
    // Test accessor: lets host-side unit tests drive private decode paths
    // (tests/MotorcycleVehicleHardwareTest.cpp) without a live CAN bus.
    friend class MotorcycleVehicleHardwareTestPeer;

    void initPropertyConfigs();
    bool openCanSocket();
    void canReaderThread();
    void processCanFrame(const struct can_frame& frame);
    void processControllerStatus(const uint8_t* data);
    void processControllerTemps(const uint8_t* data);
    void processBmsData(const uint8_t* data);
    
    // OBD2 BMS functions
    void bmsPollingThread();
    bool sendObd2Request(uint16_t pid);
    void processObd2Response(const uint8_t* data, uint8_t len);
    void updateBmsProperty(int32_t propId, float value);
    void updateBmsPropertyInt(int32_t propId, int32_t value);
    
    // Fault flags and distance accumulation (CAN reader thread only)
    void updateFaultFlags(int32_t newBits, int32_t mask);
    void accumulateDistance(float speedMps, int64_t timestamp);
    void publishDistance(int64_t timestamp);
    void persistDistanceIfDue(int64_t timestamp, bool force);

    // Configuration (CAN IDs, speed parameters, GPIO pins)
    void loadConfig();
    StatusCode applyConfigValue(const VehiclePropValue& value);
    void persistConfig(const char* name, const std::string& value);

    // GPIO functions
    bool openGpioChip();
    // Debug-only indicator source used when there is no GPIO controller
    // (the Cuttlefish simulator). Never runs on a user build.
    void gpioDebugReaderLoop();
    void gpioReaderThread();
    void updateTurnSignalState(int state);
    void updateHighBeamState(bool on);
    
    void notifyPropertyChange(int32_t propId, const VehiclePropValue& value);
    float calculateSpeedFromRpm(int rpm) const;
    int mapGearToVehicleGear(int controllerGear) const;

    std::vector<VehiclePropConfig> mPropertyConfigs;
    mutable std::mutex mValuesMutex;
    std::unordered_map<int32_t, VehiclePropValue> mCurrentValues;

    std::unique_ptr<const PropertyChangeCallback> mOnPropertyChangeCallback;
    std::unique_ptr<const PropertySetErrorCallback> mOnPropertySetErrorCallback;

    int mCanSocket = -1;
    std::atomic<bool> mRunning{false};
    std::thread mCanReaderThread;
    std::thread mBmsPollingThread;
    
    // OBD2 response tracking
    std::mutex mObd2Mutex;
    uint16_t mPendingPid = 0;
    std::condition_variable mObd2ResponseCv;
    bool mObd2ResponseReceived = false;
    
    // GPIO state
    int mGpioChipFd = -1;
    int mGpioLeftTurnPin = -1;
    int mGpioRightTurnPin = -1;
    int mGpioHighBeamPin = -1;
    std::atomic<bool> mGpioActiveLow{true};
    std::thread mGpioReaderThread;
    bool mGpioDebugSource = false;
    
    // CAN interface name (configurable via system property, default: can1)
    std::string mCanInterface = "can1";

    // Vehicle parameters for speed calculation (defaults; runtime values are
    // configurable via the VENDOR_CFG_* properties)
    static constexpr float DEFAULT_WHEEL_CIRCUMFERENCE_M = 1.894f;  // 17" wheel, 120/70 tire
    static constexpr float DEFAULT_GEAR_RATIO = 4.0f;

    // Fault state and trip/odometer accumulation. These are owned by the CAN
    // reader thread; a trip reset from a binder thread is requested via the
    // atomic flag rather than by writing the counters directly.
    int32_t mFaultFlags = 0;
    double mOdometerMeters = 0.0;
    double mTripMeters = 0.0;
    int64_t mLastDistanceTimestamp = 0;
    double mPersistedOdometerMeters = 0.0;
    double mPersistedTripMeters = 0.0;
    int64_t mLastPersistTimestamp = 0;
    std::atomic<bool> mTripResetRequested{false};

    // Runtime-configurable decode parameters. Atomics: the CAN reader thread
    // reads them per frame while binder threads may update them via setValues.
    std::atomic<uint32_t> mCanIdControllerStatus{CAN_ID_CONTROLLER_STATUS};
    std::atomic<uint32_t> mCanIdControllerTemps{CAN_ID_CONTROLLER_TEMPS};
    std::atomic<uint32_t> mCanIdBms{CAN_ID_BMS};
    std::atomic<float> mWheelCircumference{DEFAULT_WHEEL_CIRCUMFERENCE_M};
    std::atomic<float> mGearRatio{DEFAULT_GEAR_RATIO};
};

}  // namespace android::hardware::automotive::vehicle::motorcycle
