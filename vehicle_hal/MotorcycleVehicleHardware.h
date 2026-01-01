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
#include <memory>
#include <mutex>
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
    MotorcycleVehicleHardware();
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
    void initPropertyConfigs();
    bool openCanSocket();
    void canReaderThread();
    void processCanFrame(const struct can_frame& frame);
    void processControllerStatus(const uint8_t* data);
    void processControllerTemps(const uint8_t* data);
    void processBmsData(const uint8_t* data);
    
    // GPIO functions
    void loadGpioConfig();
    bool openGpioChip();
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
    
    // GPIO state
    int mGpioChipFd = -1;
    int mGpioLeftTurnPin = -1;
    int mGpioRightTurnPin = -1;
    int mGpioHighBeamPin = -1;
    bool mGpioActiveLow = true;
    std::thread mGpioReaderThread;

    // Vehicle parameters for speed calculation
    static constexpr float WHEEL_CIRCUMFERENCE_M = 1.894f;  // meters
    static constexpr float GEAR_RATIO = 4.0f;
    static constexpr float RPM_TO_MPS = WHEEL_CIRCUMFERENCE_M / (GEAR_RATIO * 60.0f);
};

}  // namespace android::hardware::automotive::vehicle::motorcycle
