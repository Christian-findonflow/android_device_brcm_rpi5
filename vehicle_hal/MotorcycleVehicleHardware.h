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

#include "imu/ImuLog.h"
#include "imu/ImuSource.h"
#include "imu/LeanEstimator.h"

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
constexpr uint32_t CAN_ID_BMS = 0x6B1;                      // Orion broadcast: DCL, CCL, high/low temp, checksum
// Display -> Controller: the controller expects its display to keep and
// report the odometer/trip/speed (stock-system behaviour). 250ms, extended ID.
constexpr uint32_t CAN_ID_DISPLAY_REPORT = 0x1026105A;

// OBD2 CAN IDs for Orion BMS (standard 11-bit IDs)
// Orion BMS answers diagnostics on the *second* OBD-II pair (0x7E3 -> 0x7EB),
// not the engine-ECU pair 0x7E0/0x7E8. Confirmed on the bike 2026-09-05:
// requests on 0x7E0 were ignored, 0x7E3 answered SoC 96.5% / 85.6 V.
constexpr uint32_t CAN_ID_OBD2_REQUEST = 0x7E3;            // OBD2 request to BMS
constexpr uint32_t CAN_ID_OBD2_RESPONSE = 0x7EB;           // OBD2 response from BMS

// Orion BMS OBD2 PIDs (Mode 0x22 - Extended Diagnostics)
constexpr uint16_t BMS_PID_PACK_CURRENT = 0xF00C;          // Signed pack current (0.1A)
constexpr uint16_t BMS_PID_PACK_VOLTAGE = 0xF00D;          // Pack voltage (0.1V)
constexpr uint16_t BMS_PID_PACK_SOC = 0xF00F;              // State of charge (0.5%/bit)
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

// Status flags from the controller status frame (0x10261022 data1 bits 0-3).
// Side stand deserves a UI indicator; the rest are informational.
constexpr int32_t VENDOR_STATUS_FLAGS = 0x21400042;        // Bitfield (int32)
constexpr int32_t STATUS_LOCKED = 1 << 0;
constexpr int32_t STATUS_BRAKE = 1 << 1;
constexpr int32_t STATUS_CRUISE = 1 << 2;
constexpr int32_t STATUS_SIDE_STAND = 1 << 3;

// Data-link status. If the controller stops broadcasting, the dashboard
// must show "no data" rather than freeze at the last values - a speedo
// stuck at the last reading is worse than a dead one.
constexpr int32_t VENDOR_LINK_STATUS = 0x21400043;
constexpr int32_t VENDOR_CFG_GEAR_BASE = 0x21400047;       // raw nibble that means P (0 or 1, applies live)
constexpr int32_t VENDOR_RAW_GEAR_STATUS = 0x21400048;     // raw byte1 of the status frame, for Workshop diagnosis         // Bitfield (int32)
constexpr int32_t LINK_CONTROLLER = 1 << 0;   // 0x10261022/23 frames flowing
constexpr int32_t LINK_BMS = 1 << 1;          // 0x6B1 or OBD2 responses flowing

// Charging: standstill with sustained charge current. rpm==0 excludes regen,
// which always coincides with the motor turning.
constexpr int32_t VENDOR_CHARGING = 0x21400044;            // 0/1 (int32)

// Temperature display unit (VehicleUnit CELSIUS/FAHRENHEIT). A vendor prop
// rather than the standard HVAC_TEMPERATURE_DISPLAY_UNITS: that property is
// owned by CarService's HVAC handling (CONTROL_CAR_CLIMATE to write, and
// climate-specific plumbing on read), whereas every consumer here - launcher
// and SystemUI - already reads vendor props via CAR_VENDOR_EXTENSION. Speed
// and distance use the standard *_DISPLAY_UNITS properties (normal
// READ_CAR_DISPLAY_UNITS / CONTROL_CAR_DISPLAY_UNITS permissions).
constexpr int32_t VENDOR_CFG_TEMP_DISPLAY_UNITS = 0x21400045;

// CAN capture (0/1). While on, every frame the HAL sees or sends is appended
// in candump -l format to /data/vendor/motodash/can-<epoch>.log, so a ride
// can be pulled with adb and replayed through moto_can_replay and the host
// tests. Toggled from Dash Settings; persisted; default off.
constexpr int32_t VENDOR_CFG_CAN_CAPTURE = 0x21400046;
// Signed pack current from the BMS (0xF00C), amps, +discharge. The
// controller's current field is documented as unsigned, so this is the
// trustworthy sign for charging/regen decisions once the BMS answers.
constexpr int32_t VENDOR_PACK_CURRENT = 0x21600047;
// Ride summary, published once per ride when the ride ends (controller link
// lost = key off, or 5 min at standstill). The last summary is persisted and
// republished at boot so the cockpit can show "last ride" after a restart.
constexpr int32_t VENDOR_RIDE_DISTANCE_M = 0x21600050;    // float, metres
constexpr int32_t VENDOR_RIDE_DURATION_S = 0x21600051;    // float, seconds moving
constexpr int32_t VENDOR_RIDE_WH_PER_KM = 0x21600052;     // float, net incl. regen
constexpr int32_t VENDOR_RIDE_MAX_SPEED_MPS = 0x21600053; // float
constexpr int32_t VENDOR_RIDE_SEQ = 0x21400049;           // int, increments per summary
// Inertial sensing: ISM330DHCX + BMP280 on /dev/i2c-1 (Grove port of the CAN
// HAT), or the scenario file /data/vendor/motodash/imu_sim on the simulator.
// Lean is estimated with the vehicle-aware filter in imu/LeanEstimator.h;
// + = right. Level (up axis) comes from the Workshop, forward is learned.
constexpr int32_t VENDOR_LEAN_DEG = 0x21600060;         // float, + = right
constexpr int32_t VENDOR_PITCH_DEG = 0x21600061;        // float, + = nose up
constexpr int32_t VENDOR_LAT_G = 0x21600062;            // float, + = right turn
constexpr int32_t VENDOR_LONG_G = 0x21600063;           // float, + = accelerating
constexpr int32_t VENDOR_BARO_HPA = 0x21600064;         // float
constexpr int32_t VENDOR_ALTITUDE_M = 0x21600065;       // float, ISA from pressure
constexpr int32_t VENDOR_IMU_TEMP_C = 0x21600066;       // float
constexpr int32_t VENDOR_IMU_STATUS = 0x21400067;       // int bitfield (IMU_STATUS_*)
constexpr int32_t VENDOR_CFG_IMU_LEVEL = 0x21400068;    // int RW: write 1 = capture level, 0 = clear calibration
constexpr int32_t VENDOR_RIDE_MAX_LEAN_L = 0x21600069;  // float deg, with the ride summary
constexpr int32_t VENDOR_RIDE_MAX_LEAN_R = 0x2160006A;  // float deg
constexpr int32_t VENDOR_IMU_RAW = 0x2161006B;          // float[6] ax ay az (g) gx gy gz (dps), sensor frame
constexpr int32_t IMU_STATUS_PRESENT = 1 << 0;
constexpr int32_t IMU_STATUS_BARO = 1 << 1;
constexpr int32_t IMU_STATUS_LEVEL_SET = 1 << 2;
constexpr int32_t IMU_STATUS_FORWARD_SET = 1 << 3;
constexpr int32_t IMU_STATUS_SIMULATED = 1 << 4;
constexpr int32_t IMU_STATUS_VALID = 1 << 5;
constexpr int32_t IMU_STATUS_LEVEL_FAILED = 1 << 6;
// Raw GPIO input levels for the Workshop indicator diagnosis: bit0 left
// turn, bit1 right turn, bit2 high beam (electrical level, before the
// active-low logic); bit3..5 = the corresponding line is configured.
constexpr int32_t VENDOR_RAW_GPIO = 0x2140006C;
// Ride mode from the controller status byte (bits 6-7 of byte 1, mapped on
// the bike 2026-09-05): 0 = mode 1, 1 = mode 2, 2 = mode 3, 3 = Sport. The
// gear lives in bits 4-5 of the same byte (0 P, 1 R, 2 N, 3 D) and the two
// fields are independent: P with mode 3 selected reads 0x80.
constexpr int32_t VENDOR_DRIVE_MODE = 0x2140006D;
constexpr int32_t DRIVE_MODE_SPORT = 3;
constexpr float CHARGING_CURRENT_THRESHOLD_A = -0.5f;

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
// Usable pack energy in Wh. Drives the RANGE_REMAINING estimate and is
// mirrored into INFO_EV_BATTERY_CAPACITY. Applies live.
constexpr int32_t VENDOR_CFG_PACK_ENERGY_WH = 0x21600039;

// Validation limits for writable configuration
constexpr float CFG_MIN_WHEEL_CIRCUMFERENCE_M = 0.5f;
constexpr float CFG_MAX_WHEEL_CIRCUMFERENCE_M = 5.0f;
constexpr float CFG_MIN_PACK_ENERGY_WH = 500.0f;
constexpr float CFG_MAX_PACK_ENERGY_WH = 50000.0f;
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
    // Range model (CAN reader thread only): integrates battery power over the
    // controller frames, folds each completed distance chunk into a Wh/km
    // EMA, and projects RANGE_REMAINING from the remaining pack energy.
    void accumulateEnergy(float voltage, float current, float speedMps, int64_t timestamp);
    void publishRange(int64_t timestamp);
    void updateStatusFlags(int32_t bits);
    void updateChargingState(int rpm, float current, int64_t nowNs);
    // Ride accounting (see VENDOR_RIDE_*). trackRide runs on the CAN thread
    // from accumulateDistance; endRideIfDue runs from the link watchdog too,
    // hence the mutex.
    void trackRide(float speedMps, int64_t timestamp);
    void addRideEnergy(double wh);
    void endRideIfDue(int64_t nowNs, bool linkDead);
    void publishRideSummary(float meters, float seconds, float whPerKm, float maxMps,
                            float maxLeanL, float maxLeanR, int32_t seq, int64_t timestamp);
    // Called by the watchdog (and tests, with an explicit now) to drop link
    // bits when frames stop arriving.
    void checkLinkTimeouts(int64_t nowNs);
    void setLinkBit(int32_t bit, bool alive);
    void linkWatchdogThread();
    // Inertial sensing (imu thread): reads the sensors or the simulator
    // file at 100 Hz, runs the lean estimator, publishes VENDOR_LEAN_DEG
    // and friends, and services the Workshop Level command. See imu/.
    void imuThread();
    std::unique_ptr<imu::ImuSource> openImuSource();
    void processImuSample(const imu::ImuSample& s, float tempC, int64_t nowNs);
    void publishImuStatus(int64_t nowNs);
    void captureImuSample(const imu::ImuSample& s, float speedMps, bool speedValid,
                          const imu::LeanEstimator::State& st, int64_t nowNs);
    void loadImuCalibration();
    void persistImuCalibration();
    void trackRideLean(float rollDeg, float speedMps);
    void sendDisplayReportIfDue(int64_t timestamp, float speedMps);
    // CAN capture to /data (see VENDOR_CFG_CAN_CAPTURE). Called for every
    // received frame (processCanFrame) and every frame we transmit.
    void captureFrame(const struct can_frame& frame);
    void closeCapture();

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
    // Interruptible sleep for worker threads: returns false when shutting
    // down, so no thread holds up service stop by a fixed sleep. The CAN
    // reader's blocking read() is unblocked separately via shutdown() on the
    // socket in the destructor.
    bool sleepUnlessStopping(int64_t ms);
    std::mutex mShutdownMutex;
    std::condition_variable mShutdownCv;
    // Verbose CAN frame logging, persist.vendor.motodash.debug.canlog.
    // Off by default: at 20-50Hz per frame type the dumps flood logcat.
    bool mVerboseCanLog = false;
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
    // Set when active-low changes at runtime: the GPIO thread re-applies the
    // line bias (pull-up for active-low, pull-down otherwise).
    std::atomic<bool> mGpioBiasDirty{false};
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
    int32_t mStatusFlags = 0;
    int32_t mLinkStatus = 0;
    int32_t mCharging = 0;

    // Range model state (CAN reader thread only). mWhPerKm is the learned
    // consumption EMA; 0 means "not yet learned" and suppresses the range
    // estimate rather than inventing one. Persisted alongside the odometer.
    double mChunkEnergyWh = 0.0;
    double mChunkStartMeters = -1.0;
    int64_t mLastEnergyTimestamp = 0;
    float mWhPerKm = 0.0f;
    float mPersistedWhPerKm = 0.0f;
    float mLastSocPercent = -1.0f;
    // Samsung 35E 21s20p: 75.6V nominal x 70Ah = 5292Wh. Configurable via
    // VENDOR_CFG_PACK_ENERGY_WH; kept in sync with INFO_EV_BATTERY_CAPACITY.
    std::atomic<float> mPackEnergyWh{5292.0f};

    // Display units (standard *_DISPLAY_UNITS properties). Pure pass-through
    // settings: the settings UI writes them, every display surface (cluster,
    // cockpit, SystemUI bar) subscribes, and the HAL persists them so a
    // reboot keeps the rider's choice. The HAL itself always works in SI.
    // CAN capture state. The file is opened lazily on the first frame after
    // enabling, so a boot with an idle bus creates nothing.
    std::atomic<bool> mCaptureEnabled{false};
    std::mutex mCaptureMutex;
    int mCaptureFd = -1;
    int64_t mLastCaptureSyncNs = 0;
    uint64_t mCaptureBytes = 0;
    bool mCaptureFailureLogged = false;
    std::string mCaptureDir = "/data/vendor/motodash";

    // BMS-sourced SoC and current (OBD2 0xF00F / 0xF00C). When fresh they
    // outrank the 0x6B1 broadcast decode (whose layout is inferred) and the
    // controller's unsigned current. CAN reader thread only.
    int64_t mLastSocPidNs = 0;
    int64_t mLastBmsCurrentNs = 0;
    float mBmsCurrentA = 0.0f;

    int32_t mSpeedDisplayUnits;        // VehicleUnit KILOMETERS_PER_HOUR / MILES_PER_HOUR
    int32_t mDistanceDisplayUnits;     // VehicleUnit KILOMETER / MILE
    int32_t mTemperatureDisplayUnits;  // VehicleUnit CELSIUS / FAHRENHEIT
    std::atomic<int64_t> mLastControllerFrameNs{0};
    std::atomic<int64_t> mLastBmsFrameNs{0};
    std::thread mLinkWatchdogThread;
    int64_t mLastDisplayReportTimestamp = 0;

    // Runtime-configurable decode parameters. Atomics: the CAN reader thread
    // reads them per frame while binder threads may update them via setValues.
    std::atomic<uint32_t> mCanIdControllerStatus{CAN_ID_CONTROLLER_STATUS};
    std::atomic<uint32_t> mCanIdControllerTemps{CAN_ID_CONTROLLER_TEMPS};
    std::atomic<uint32_t> mCanIdBms{CAN_ID_BMS};
    std::atomic<float> mWheelCircumference{DEFAULT_WHEEL_CIRCUMFERENCE_M};
    std::atomic<float> mGearRatio{DEFAULT_GEAR_RATIO};
    std::atomic<int32_t> mGearBase{0};
    // Charging dwell: standstill + charge current must hold this long before
    // the charging state is declared (kills the regen-stop flash).
    int64_t mChargingCandidateSinceNs = 0;
    std::atomic<int64_t> mChargingDwellNs{5LL * 1000000000LL};

    std::mutex mRideMutex;
    bool mRideActive = false;
    double mRideStartMeters = 0.0;
    int64_t mRideStartNs = 0;
    int64_t mRideLastMoveNs = 0;
    int64_t mRideMovingNs = 0;
    int64_t mRideLastTrackNs = 0;
    double mRideEnergyWh = 0.0;
    float mRideMaxSpeedMps = 0.0f;
    float mRideMaxLeanL = 0.0f;  // deg, this ride, speed-gated (imu thread)
    float mRideMaxLeanR = 0.0f;
    int32_t mRideSeq = 0;

    // Inertial sensing. Owned by the imu thread except the atomics, which
    // the CAN thread (speed) and binder threads (Level command) write.
    std::thread mImuThread;
    imu::LeanEstimator mLean;
    imu::LevelCapture mLevelCapture;
    bool mLevelCapturing = false;
    std::atomic<int> mLevelRequest{-1};  // -1 none, 1 capture level, 0 clear
    std::atomic<float> mLastSpeedMps{0.0f};
    int32_t mImuSourceBits = 0;     // PRESENT | BARO | SIMULATED of the open source
    bool mImuLevelFailed = false;
    int32_t mImuStatus = 0;         // last published VENDOR_IMU_STATUS
    int64_t mLastImuSampleNs = 0;
    int64_t mLastLeanPublishNs = 0;
    int64_t mLastRawPublishNs = 0;
    int64_t mLastBaroReadNs = 0;
    std::string mImuI2cPath = "/dev/i2c-1";
    int mImuAddr = 0x6A;
    int mBaroAddr = 0x77;
    std::string mImuSimPath = "/data/vendor/motodash/imu_sim";
    // Raw IMU capture, tied to the CAN capture switch (VENDOR_CFG_CAN_CAPTURE);
    // imu thread only.
    imu::ImuLogWriter mImuLog;
    bool mImuLogFailureLogged = false;
};

}  // namespace android::hardware::automotive::vehicle::motorcycle
