/*
 * Copyright (C) 2024 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "MotorcycleVehicleHardware"

#include "MotorcycleVehicleHardware.h"

#include <android-base/logging.h>
#include <utils/SystemClock.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

#include <cutils/properties.h>

namespace android::hardware::automotive::vehicle::motorcycle {

using ::aidl::android::hardware::automotive::vehicle::VehicleGear;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyType;
using ::aidl::android::hardware::automotive::vehicle::VehicleUnit;

MotorcycleVehicleHardware::MotorcycleVehicleHardware(std::string canInterfaceOverride) {
    LOG(INFO) << "MotorcycleVehicleHardware initializing...";
    
    // Load persisted configuration (CAN IDs, speed parameters, GPIO pins)
    loadConfig();
    if (!canInterfaceOverride.empty()) {
        mCanInterface = std::move(canInterfaceOverride);
        LOG(INFO) << "CAN interface overridden: " << mCanInterface;
    }
    
    initPropertyConfigs();
    
    // Start CAN initialization in a separate thread to allow retries
    // CAN interfaces may not be available immediately at boot
    mRunning = true;
    mCanReaderThread = std::thread(&MotorcycleVehicleHardware::canReaderThread, this);
    LOG(INFO) << "CAN reader thread started (will retry connection)";
    
    // Start GPIO reader thread if any GPIO pins are configured
    if (mGpioDebugSource || mGpioLeftTurnPin >= 0 || mGpioRightTurnPin >= 0
        || mGpioHighBeamPin >= 0) {
        mGpioReaderThread = std::thread(&MotorcycleVehicleHardware::gpioReaderThread, this);
        LOG(INFO) << "GPIO reader thread started";
    }
    
    // Start BMS polling thread for OBD2 queries
    mBmsPollingThread = std::thread(&MotorcycleVehicleHardware::bmsPollingThread, this);

    // Link watchdog: drops the link bits when frames stop, so the UI can show
    // "no data" instead of freezing at the last values.
    mLinkWatchdogThread = std::thread(&MotorcycleVehicleHardware::linkWatchdogThread, this);
    // Inertial sensing: probes the I2C sensors (or the simulator file) and
    // keeps probing, so a module plugged in later just starts working.
    mImuThread = std::thread(&MotorcycleVehicleHardware::imuThread, this);
    LOG(INFO) << "BMS polling thread started";
}

MotorcycleVehicleHardware::~MotorcycleVehicleHardware() {
    mRunning = false;
    // Flush the odometer so a clean service stop does not lose distance since
    // the last throttled write.
    persistDistanceIfDue(elapsedRealtimeNano(), /*force=*/true);
    mShutdownCv.notify_all();      // Wake any thread in sleepUnlessStopping
    mObd2ResponseCv.notify_all();  // Wake up BMS polling thread
    if (mCanSocket >= 0) {
        // Unblock the reader's blocking read(); on a silent bus it would
        // otherwise never notice mRunning and the join below would hang.
        shutdown(mCanSocket, SHUT_RDWR);
    }
    if (mCanReaderThread.joinable()) {
        mCanReaderThread.join();
    }
    if (mBmsPollingThread.joinable()) {
        mBmsPollingThread.join();
    }
    if (mGpioReaderThread.joinable()) {
        mGpioReaderThread.join();
    }
    if (mLinkWatchdogThread.joinable()) {
        mLinkWatchdogThread.join();
    }
    if (mImuThread.joinable()) {
        mImuThread.join();
    }
    mImuLog.close();
    closeCapture();
    if (mCanSocket >= 0) {
        close(mCanSocket);
    }
    if (mGpioChipFd >= 0) {
        close(mGpioChipFd);
    }
}

void MotorcycleVehicleHardware::initPropertyConfigs() {
    // PERF_VEHICLE_SPEED - Vehicle speed in m/s
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::PERF_VEHICLE_SPEED);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 1.0f;
        config.maxSampleRate = 100.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = 0.0f;
        areaConfig.maxFloatValue = 100.0f;  // ~360 km/h max
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        // Initialize default value
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(0.0f);
        mCurrentValues[config.prop] = value;
    }

    // ENGINE_RPM - Motor RPM
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::ENGINE_RPM);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 1.0f;
        config.maxSampleRate = 100.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = 0.0f;
        areaConfig.maxFloatValue = 15000.0f;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(0.0f);
        mCurrentValues[config.prop] = value;
    }

    // CURRENT_GEAR
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::CURRENT_GEAR);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(static_cast<int32_t>(VehicleGear::GEAR_NEUTRAL));
        mCurrentValues[config.prop] = value;
    }

    // GEAR_SELECTION
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::GEAR_SELECTION);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(static_cast<int32_t>(VehicleGear::GEAR_NEUTRAL));
        mCurrentValues[config.prop] = value;
    }

    // PARKING_BRAKE_ON - derived from gear: P = brake on, R/N/D = off. The
    // bike has no parking-brake switch, but CarDrivingStateService refuses to
    // initialize without this property, which left the whole driving-state
    // layer (and every UX restriction) permanently inert. Boot default is ON
    // so an unknown gear reads as parked.
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::PARKING_BRAKE_ON);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);

        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(1);  // parked until the controller says otherwise
        mCurrentValues[config.prop] = value;
    }

    // EV_BATTERY_LEVEL - Battery State of Charge (%)
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 0.1f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = 0.0f;
        areaConfig.maxFloatValue = 100.0f;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(0.0f);
        mCurrentValues[config.prop] = value;
    }

    // EV_BATTERY_INSTANTANEOUS_CHARGE_RATE - Current in/out of battery (Watts)
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::EV_BATTERY_INSTANTANEOUS_CHARGE_RATE);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 1.0f;
        config.maxSampleRate = 100.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(0.0f);
        mCurrentValues[config.prop] = value;
    }

    // INFO_EV_BATTERY_CAPACITY - Battery capacity in Wh
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::INFO_EV_BATTERY_CAPACITY);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::STATIC;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(mPackEnergyWh.load());
        mCurrentValues[config.prop] = value;
    }

    // RANGE_REMAINING - Estimated remaining range in meters. Computed from
    // the learned Wh/km consumption EMA and remaining pack energy; 0 until
    // the model has learned a consumption figure (the UI shows "--").
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::RANGE_REMAINING);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 0.1f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);

        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(0.0f);
        mCurrentValues[config.prop] = value;
    }

    // PERF_ODOMETER - Odometer in km
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::PERF_ODOMETER);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 0.1f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(static_cast<float>(mOdometerMeters / 1000.0));
        mCurrentValues[config.prop] = value;
    }

    // ENGINE_COOLANT_TEMP - Controller temperature (repurposed)
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::ENGINE_COOLANT_TEMP);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 0.5f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = -40.0f;
        areaConfig.maxFloatValue = 150.0f;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(25.0f);
        mCurrentValues[config.prop] = value;
    }

    // ENGINE_OIL_TEMP - Motor temperature (repurposed)
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::ENGINE_OIL_TEMP);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 0.5f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = -40.0f;
        areaConfig.maxFloatValue = 150.0f;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(25.0f);
        mCurrentValues[config.prop] = value;
    }

    // VENDOR_BATTERY_VOLTAGE - Battery voltage in volts
    {
        VehiclePropConfig config;
        config.prop = VENDOR_BATTERY_VOLTAGE;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 1.0f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = 0.0f;
        areaConfig.maxFloatValue = 100.0f;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(0.0f);
        mCurrentValues[config.prop] = value;
    }

    // VENDOR_BATTERY_CURRENT - Battery current in amps
    {
        VehiclePropConfig config;
        config.prop = VENDOR_BATTERY_CURRENT;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 1.0f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = -500.0f;
        areaConfig.maxFloatValue = 500.0f;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(0.0f);
        mCurrentValues[config.prop] = value;
    }

    // VENDOR_THROTTLE_POSITION - Throttle position 0-100%
    {
        VehiclePropConfig config;
        config.prop = VENDOR_THROTTLE_POSITION;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 1.0f;
        config.maxSampleRate = 30.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = 0.0f;
        areaConfig.maxFloatValue = 100.0f;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(0.0f);
        mCurrentValues[config.prop] = value;
    }

    // TURN_SIGNAL_LIGHT_STATE - Turn signal indicator state (from GPIO)
    // Using TURN_SIGNAL_LIGHT_STATE instead of deprecated TURN_SIGNAL_STATE
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::TURN_SIGNAL_LIGHT_STATE);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(0);  // VehicleTurnSignal::NONE
        mCurrentValues[config.prop] = value;
    }

    // HIGH_BEAM_LIGHTS_STATE - High beam indicator state (from GPIO)
    {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(VehicleProperty::HIGH_BEAM_LIGHTS_STATE);
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(0);  // VehicleLightState::OFF
        mCurrentValues[config.prop] = value;
    }

    // ========================================================================
    // BMS Properties (Orion BMS OBD2 data)
    // ========================================================================

    // Helper lambda to add float vendor properties
    auto addFloatVendorProp = [this](int32_t propId, float minVal, float maxVal, float defaultVal) {
        VehiclePropConfig config;
        config.prop = propId;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 0.5f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.minFloatValue = minVal;
        areaConfig.maxFloatValue = maxVal;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = propId;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(defaultVal);
        mCurrentValues[propId] = value;
    };

    // Helper lambda to add int vendor properties
    auto addIntVendorProp = [this](int32_t propId, int32_t defaultVal) {
        VehiclePropConfig config;
        config.prop = propId;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        
        VehiclePropValue value;
        value.prop = propId;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(defaultVal);
        mCurrentValues[propId] = value;
    };

    // Pack health and capacity
    addFloatVendorProp(VENDOR_PACK_SOH, 0.0f, 100.0f, 100.0f);           // State of Health %
    addFloatVendorProp(VENDOR_PACK_CURRENT, -3276.7f, 3276.7f, 0.0f);   // BMS signed current, A (+discharge)

    // Ride summary (last ride; restored from persist props at startup)
    addFloatVendorProp(VENDOR_RIDE_DISTANCE_M, 0.0f, 10000000.0f, 0.0f);
    addFloatVendorProp(VENDOR_RIDE_DURATION_S, 0.0f, 1000000.0f, 0.0f);
    addFloatVendorProp(VENDOR_RIDE_WH_PER_KM, -1000.0f, 1000.0f, 0.0f);
    addFloatVendorProp(VENDOR_RIDE_MAX_SPEED_MPS, 0.0f, 100.0f, 0.0f);
    addIntVendorProp(VENDOR_RIDE_SEQ, 0);
    addFloatVendorProp(VENDOR_RIDE_MAX_LEAN_L, 0.0f, 90.0f, 0.0f);
    addFloatVendorProp(VENDOR_RIDE_MAX_LEAN_R, 0.0f, 90.0f, 0.0f);

    // Inertial sensing (imu/): lean at 10 Hz, the rest slower.
    addFloatVendorProp(VENDOR_LEAN_DEG, -90.0f, 90.0f, 0.0f);
    addFloatVendorProp(VENDOR_PITCH_DEG, -90.0f, 90.0f, 0.0f);
    addFloatVendorProp(VENDOR_LAT_G, -3.0f, 3.0f, 0.0f);
    addFloatVendorProp(VENDOR_LONG_G, -3.0f, 3.0f, 0.0f);
    addFloatVendorProp(VENDOR_BARO_HPA, 300.0f, 1200.0f, 0.0f);
    addFloatVendorProp(VENDOR_ALTITUDE_M, -1000.0f, 10000.0f, 0.0f);
    addFloatVendorProp(VENDOR_IMU_TEMP_C, -50.0f, 150.0f, 0.0f);
    addIntVendorProp(VENDOR_IMU_STATUS, 0);
    addIntVendorProp(VENDOR_RAW_GPIO, 0);
    addIntVendorProp(VENDOR_DRIVE_MODE, 0);
    {
        // Raw sensor axes for the Workshop mounting check: float[6].
        VehiclePropConfig config;
        config.prop = VENDOR_IMU_RAW;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 0.5f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);
        VehiclePropValue value;
        value.prop = VENDOR_IMU_RAW;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.assign(6, 0.0f);
        mCurrentValues[VENDOR_IMU_RAW] = value;
    }
    addFloatVendorProp(VENDOR_PACK_DOD, 0.0f, 100.0f, 0.0f);             // Depth of Discharge %
    addFloatVendorProp(VENDOR_PACK_AMPHOURS, 0.0f, 500.0f, 100.0f);      // Capacity Ah
    addFloatVendorProp(VENDOR_PACK_RESISTANCE, 0.0f, 1000.0f, 0.0f);     // Resistance mOhm
    addIntVendorProp(VENDOR_PACK_CYCLES, 0);                             // Cycle count

    // Pack temperatures
    addFloatVendorProp(VENDOR_PACK_TEMP_AVG, -40.0f, 80.0f, 25.0f);      // Avg temp
    addFloatVendorProp(VENDOR_PACK_TEMP_HIGH, -40.0f, 80.0f, 25.0f);     // High temp
    addFloatVendorProp(VENDOR_PACK_TEMP_LOW, -40.0f, 80.0f, 25.0f);      // Low temp
    addFloatVendorProp(VENDOR_HEATSINK_TEMP, -40.0f, 80.0f, 25.0f);      // Heatsink temp
    addIntVendorProp(VENDOR_FAN_SPEED, 0);                               // Fan speed 0-6

    // Cell voltages
    addFloatVendorProp(VENDOR_CELL_VOLTAGE_LOW, 0.0f, 5.0f, 3.7f);       // Low cell V
    addFloatVendorProp(VENDOR_CELL_VOLTAGE_HIGH, 0.0f, 5.0f, 3.7f);      // High cell V
    addFloatVendorProp(VENDOR_CELL_VOLTAGE_AVG, 0.0f, 5.0f, 3.7f);       // Avg cell V
    addIntVendorProp(VENDOR_CELL_LOW_ID, 0);                             // Low cell #
    addIntVendorProp(VENDOR_CELL_HIGH_ID, 0);                            // High cell #

    // Current limits
    addFloatVendorProp(VENDOR_CHARGE_LIMIT, 0.0f, 500.0f, 0.0f);         // Charge limit A
    addFloatVendorProp(VENDOR_DISCHARGE_LIMIT, 0.0f, 500.0f, 0.0f);      // Discharge limit A

    // Fault flags (ON_CHANGE - only fires when the bitfield actually changes)
    {
        VehiclePropConfig config;
        config.prop = VENDOR_FAULT_FLAGS;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);

        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(0);
        mCurrentValues[config.prop] = value;
    }

    // Status flags (side stand, brake, cruise, lock) - ON_CHANGE
    {
        VehiclePropConfig config;
        config.prop = VENDOR_STATUS_FLAGS;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);

        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(0);
        mCurrentValues[config.prop] = value;
    }

    // Link status and charging state - ON_CHANGE
    for (int32_t prop : {VENDOR_LINK_STATUS, VENDOR_CHARGING, VENDOR_RAW_GEAR_STATUS}) {
        VehiclePropConfig config;
        config.prop = prop;
        config.access = VehiclePropertyAccess::READ;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);

        VehiclePropValue value;
        value.prop = prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(0);
        mCurrentValues[prop] = value;
    }

    // Trip distance (READ_WRITE so the UI can reset it by writing 0)
    {
        VehiclePropConfig config;
        config.prop = VENDOR_TRIP_DISTANCE;
        config.access = VehiclePropertyAccess::READ_WRITE;
        config.changeMode = VehiclePropertyChangeMode::CONTINUOUS;
        config.minSampleRate = 0.1f;
        config.maxSampleRate = 10.0f;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.access = VehiclePropertyAccess::READ_WRITE;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);

        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.floatValues.push_back(static_cast<float>(mTripMeters / 1000.0));
        mCurrentValues[config.prop] = value;
    }

    // Writable configuration properties (see VENDOR_CFG_* in the header).
    // Registered with the values loaded from persist.vendor.motodash.* so the
    // settings UI reads back the current configuration.
    auto addConfigProp = [this](int32_t propId, bool isFloat, float floatVal, int32_t intVal) {
        VehiclePropConfig config;
        config.prop = propId;
        config.access = VehiclePropertyAccess::READ_WRITE;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.access = VehiclePropertyAccess::READ_WRITE;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);

        VehiclePropValue value;
        value.prop = propId;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        if (isFloat) {
            value.value.floatValues.push_back(floatVal);
        } else {
            value.value.int32Values.push_back(intVal);
        }
        mCurrentValues[propId] = value;
    };

    addConfigProp(VENDOR_CFG_WHEEL_CIRCUMFERENCE, true, mWheelCircumference.load(), 0);
    addConfigProp(VENDOR_CFG_GEAR_RATIO, true, mGearRatio.load(), 0);
    addConfigProp(VENDOR_CFG_CAN_ID_CONTROLLER_STATUS, false, 0.0f,
                  static_cast<int32_t>(mCanIdControllerStatus.load()));
    addConfigProp(VENDOR_CFG_CAN_ID_CONTROLLER_TEMPS, false, 0.0f,
                  static_cast<int32_t>(mCanIdControllerTemps.load()));
    addConfigProp(VENDOR_CFG_CAN_ID_BMS, false, 0.0f,
                  static_cast<int32_t>(mCanIdBms.load()));
    addConfigProp(VENDOR_CFG_GPIO_LEFT_TURN, false, 0.0f, mGpioLeftTurnPin);
    addConfigProp(VENDOR_CFG_GPIO_RIGHT_TURN, false, 0.0f, mGpioRightTurnPin);
    addConfigProp(VENDOR_CFG_GPIO_HIGH_BEAM, false, 0.0f, mGpioHighBeamPin);
    addConfigProp(VENDOR_CFG_GPIO_ACTIVE_LOW, false, 0.0f, mGpioActiveLow.load() ? 1 : 0);
    addConfigProp(VENDOR_CFG_PACK_ENERGY_WH, true, mPackEnergyWh.load(), 0);
    addConfigProp(VENDOR_CFG_CAN_CAPTURE, false, 0.0f, mCaptureEnabled.load() ? 1 : 0);
    // Which raw nibble means P: the spec hints the bike may report 1=P..4=D
    // instead of 0=P..3=D ("display needs +1"). Settable from Workshop
    // settings on the fly so a first-ride discrepancy needs no rebuild.
    addConfigProp(VENDOR_CFG_GEAR_BASE, false, 0.0f, mGearBase.load());
    addConfigProp(VENDOR_CFG_IMU_LEVEL, false, 0.0f, 0);
    addConfigProp(VENDOR_CFG_PACK_MAX_VOLTAGE, true, mPackMaxVoltage.load(), 0);

    // Standard display-unit properties. configArray lists the supported
    // VehicleUnit values, as the property docs require.
    auto addUnitsProp = [this](VehicleProperty prop, int32_t initial,
                               std::vector<int32_t> supported) {
        VehiclePropConfig config;
        config.prop = static_cast<int32_t>(prop);
        config.access = VehiclePropertyAccess::READ_WRITE;
        config.changeMode = VehiclePropertyChangeMode::ON_CHANGE;
        config.configArray = std::move(supported);
        VehicleAreaConfig areaConfig;
        areaConfig.areaId = 0;
        areaConfig.access = VehiclePropertyAccess::READ_WRITE;
        config.areaConfigs.push_back(areaConfig);
        mPropertyConfigs.push_back(config);

        VehiclePropValue value;
        value.prop = config.prop;
        value.areaId = 0;
        value.timestamp = elapsedRealtimeNano();
        value.value.int32Values.push_back(initial);
        mCurrentValues[config.prop] = value;
    };
    addUnitsProp(VehicleProperty::VEHICLE_SPEED_DISPLAY_UNITS, mSpeedDisplayUnits,
                 {static_cast<int32_t>(VehicleUnit::KILOMETERS_PER_HOUR),
                  static_cast<int32_t>(VehicleUnit::MILES_PER_HOUR)});
    addUnitsProp(VehicleProperty::DISTANCE_DISPLAY_UNITS, mDistanceDisplayUnits,
                 {static_cast<int32_t>(VehicleUnit::KILOMETER),
                  static_cast<int32_t>(VehicleUnit::MILE)});
    // Temperature is a vendor prop - see VENDOR_CFG_TEMP_DISPLAY_UNITS in the
    // header for why it cannot be the standard HVAC property.
    addUnitsProp(static_cast<VehicleProperty>(VENDOR_CFG_TEMP_DISPLAY_UNITS),
                 mTemperatureDisplayUnits,
                 {static_cast<int32_t>(VehicleUnit::CELSIUS),
                  static_cast<int32_t>(VehicleUnit::FAHRENHEIT)});

    LOG(INFO) << "Initialized " << mPropertyConfigs.size() << " property configs";
}

bool MotorcycleVehicleHardware::sleepUnlessStopping(int64_t ms) {
    std::unique_lock<std::mutex> lock(mShutdownMutex);
    mShutdownCv.wait_for(lock, std::chrono::milliseconds(ms),
                         [this] { return !mRunning; });
    return mRunning;
}

bool MotorcycleVehicleHardware::openCanSocket() {
    mCanSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mCanSocket < 0) {
        LOG(ERROR) << "Failed to create CAN socket: " << strerror(errno);
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, mCanInterface.c_str(), IFNAMSIZ - 1);

    if (ioctl(mCanSocket, SIOCGIFINDEX, &ifr) < 0) {
        LOG(ERROR) << "Failed to get interface index for " << mCanInterface << ": " << strerror(errno);
        close(mCanSocket);
        mCanSocket = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(mCanSocket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOG(ERROR) << "Failed to bind CAN socket: " << strerror(errno);
        close(mCanSocket);
        mCanSocket = -1;
        return false;
    }

    LOG(INFO) << "CAN socket opened on " << mCanInterface << " (ifindex=" << ifr.ifr_ifindex << ")";
    return true;
}

void MotorcycleVehicleHardware::canReaderThread() {
    LOG(INFO) << "CAN reader thread starting, will attempt to open CAN socket...";
    
    // Retry opening the CAN socket until it works. A vehicle HAL must never
    // give up on its bus: kernel modules or the interface may come up late
    // (seen in practice on the simulator), and a HAL that stopped retrying
    // after 30s left the dashboard permanently dead until service restart.
    for (int retry = 0; mRunning; retry++) {
        if (openCanSocket()) {
            LOG(INFO) << "CAN socket opened successfully after " << retry << " retries";
            break;
        }
        if (retry % 30 == 0) {
            LOG(WARNING) << "CAN socket not ready (attempt " << (retry + 1)
                         << "), retrying every 1s";
        }
        if (!sleepUnlessStopping(1000)) break;
    }

    if (mCanSocket < 0) {
        return;  // only reachable when shutting down
    }
    
    LOG(INFO) << "CAN reader thread running, socket fd=" << mCanSocket;
    struct can_frame frame;
    int frameCount = 0;

    while (mRunning) {
        ssize_t nbytes = read(mCanSocket, &frame, sizeof(frame));
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            if (!mRunning) break;
            // ENETDOWN and friends: the interface went down (bus-off restart,
            // `ip link set can1 down/up` on the bench). Seen on the bike
            // 2026-09-05: the thread used to exit here and the dashboard
            // stayed blind while the OBD2 poller kept transmitting. Reopen.
            LOG(ERROR) << "CAN read error: " << strerror(errno) << " - reopening the socket";
            close(mCanSocket);
            mCanSocket = -1;
            while (mRunning && !openCanSocket()) {
                if (!sleepUnlessStopping(1000)) break;
            }
            if (mCanSocket < 0) break;
            LOG(INFO) << "CAN socket reopened, fd=" << mCanSocket;
            continue;
        }

        if (nbytes == sizeof(frame)) {
            frameCount++;
            if (mVerboseCanLog && frameCount % 100 == 1) {
                LOG(INFO) << "Received " << frameCount << " CAN frames, last ID=0x" 
                          << std::hex << (frame.can_id & CAN_EFF_MASK) << std::dec
                          << " extended=" << ((frame.can_id & CAN_EFF_FLAG) != 0);
            }
            processCanFrame(frame);
        }
    }

    LOG(INFO) << "CAN reader thread exiting, total frames=" << frameCount;
}

namespace {
// Rate-limited logging of CAN IDs we don't decode. Bounded so a noisy or
// corrupted bus cannot grow the map without limit on the reader thread.
void logUnknownCanId(uint32_t canId, bool isExtended, const uint8_t* data) {
    static std::unordered_map<uint32_t, int> unknownIds;
    constexpr size_t kMaxTrackedIds = 64;
    if (unknownIds.size() >= kMaxTrackedIds && unknownIds.find(canId) == unknownIds.end()) {
        return;  // Too many distinct unknown IDs; stop tracking new ones.
    }
    if (++unknownIds[canId] % 100 == 1) {
        LOG(INFO) << "Unknown " << (isExtended ? "extended" : "standard")
                  << " CAN ID 0x" << std::hex << canId << std::dec
                  << " data=[" << (int)data[0] << "," << (int)data[1] << ","
                  << (int)data[2] << "," << (int)data[3] << ","
                  << (int)data[4] << "," << (int)data[5] << ","
                  << (int)data[6] << "," << (int)data[7] << "]";
    }
}
}  // namespace

void MotorcycleVehicleHardware::processCanFrame(const struct can_frame& frame) {
    captureFrame(frame);
    uint32_t canId = frame.can_id & CAN_EFF_MASK;
    bool isExtended = (frame.can_id & CAN_EFF_FLAG) != 0;

    // Compare against the configured (runtime-changeable) message IDs.
    if (canId == mCanIdControllerStatus.load(std::memory_order_relaxed)) {
        processControllerStatus(frame.data);
    } else if (canId == mCanIdControllerTemps.load(std::memory_order_relaxed)) {
        processControllerTemps(frame.data);
    } else if (canId == mCanIdBms.load(std::memory_order_relaxed)) {
        processBmsData(frame.data);
    } else if (!isExtended && canId == CAN_ID_OBD2_RESPONSE) {
        // OBD2 response from BMS
        processObd2Response(frame.data, frame.can_dlc);
    } else {
        logUnknownCanId(canId, isExtended, frame.data);
    }
}

void MotorcycleVehicleHardware::processControllerStatus(const uint8_t* data) {
    // Message 0x10261022 format (from controller spec):
    // Byte 0: Error Codes (bitfield)
    // Byte 1: Status Flags (bits 0-3) & Gear (bits 4-7: 00=P, 01=R, 10=N, 11=D)
    // Bytes 2-3: RPM (UINT16 little-endian, 1 rpm/bit)
    // Bytes 4-5: Battery Voltage (UINT16 little-endian, 0.1 V/bit)
    // Bytes 6-7: Battery Current (UINT16 little-endian, 0.1 A/bit)
    
    int errors = data[0];
    int statusAndGear = data[1];
    // Byte 1 high nibble = gear (bits 4-5) + ride mode (bits 6-7); confirmed
    // on the bike 2026-09-05 (P 00, R 10, D 30, D+mode2 70, D+mode3 B0,
    // D+Sport F0, P+mode3 80). Mask the mode out before the gear map.
    // The Workshop gear base (spec ambiguity, 1=P..4=D firmware) shifts the
    // nibble before the split; 0 on this bike.
    int nibble = ((statusAndGear >> 4) - mGearBase.load(std::memory_order_relaxed)) & 0x0F;
    int gearRaw = nibble & 0x03;
    int driveMode = (nibble >> 2) & 0x03;
    int rpm = data[2] | (data[3] << 8);
    int voltageRaw = data[4] | (data[5] << 8);
    float voltage = voltageRaw * 0.1f;
    mLastPackVoltage.store(voltage, std::memory_order_relaxed);
    // Current is signed 16-bit (negative = regen/charging, positive = discharge)
    int16_t currentRaw = static_cast<int16_t>(data[6] | (data[7] << 8));
    float current = currentRaw * 0.1f;
    
    // Map gear: 00=P(0), 01=R(1), 10=N(2), 11=D(3).
    int gear = gearRaw;
    
    // Frame dump, gated: at 20Hz this alone is ~2 lines/s in logcat
    static int statusMsgCount = 0;
    if (mVerboseCanLog && ++statusMsgCount % 10 == 1) {
        LOG(INFO) << "Controller 0x10261022: data[0-7]=" 
                  << (int)data[0] << "," << (int)data[1] << "," 
                  << (int)data[2] << "," << (int)data[3] << ","
                  << (int)data[4] << "," << (int)data[5] << ","
                  << (int)data[6] << "," << (int)data[7]
                  << " -> errors=" << errors << " gear=" << gear 
                  << " rpm=" << rpm << " V=" << voltage << " A=" << current;
    }

    int64_t timestamp = elapsedRealtimeNano();

    // Update RPM
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::ENGINE_RPM)];
        value.value.floatValues[0] = static_cast<float>(rpm);
        value.timestamp = timestamp;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::ENGINE_RPM), value);
    }

    // Update Speed (calculated from RPM)
    {
        float speedMps = calculateSpeedFromRpm(rpm);
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::PERF_VEHICLE_SPEED)];
        value.value.floatValues[0] = speedMps;
        value.timestamp = timestamp;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::PERF_VEHICLE_SPEED), value);
    }

    // Fault bits from this frame, and distance accumulated from the speed we
    // just derived. Both run on the CAN reader thread, which owns that state.
    mLastControllerFrameNs.store(timestamp, std::memory_order_relaxed);
    setLinkBit(LINK_CONTROLLER, true);
    updateFaultFlags(errors, FAULT_MASK_CONTROLLER_STATUS);
    updateStatusFlags(statusAndGear & 0x0F);
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& mode = mCurrentValues[VENDOR_DRIVE_MODE];
        if (mode.value.int32Values[0] != driveMode) {
            mode.value.int32Values[0] = driveMode;
            mode.timestamp = timestamp;
            notifyPropertyChange(VENDOR_DRIVE_MODE, mode);
            LOG(INFO) << "Ride mode changed: " << driveMode << (driveMode == DRIVE_MODE_SPORT ? " (Sport)" : "");
        }
    }
    // The controller's current sensor reads -1.7 A with the bike at rest
    // (measured 2026-09-05); the BMS shunt (0xF00C) is the truth whenever it
    // is fresh, for charging detection, the energy model and the display.
    constexpr int64_t kBmsCurrentFreshNs = 5LL * 1000000000LL;
    bool bmsFresh = mLastBmsCurrentNs != 0 && (timestamp - mLastBmsCurrentNs) < kBmsCurrentFreshNs;
    float bestCurrent = bmsFresh ? mBmsCurrentA : current;
    updateChargingState(rpm, bestCurrent, timestamp);
    float speedMps = calculateSpeedFromRpm(rpm);
    mLastSpeedMps.store(speedMps, std::memory_order_relaxed);  // for the lean estimator
    accumulateDistance(speedMps, timestamp);
    accumulateEnergy(voltage, bestCurrent, speedMps, timestamp);
    publishDistance(timestamp);
    persistDistanceIfDue(timestamp, /*force=*/false);
    sendDisplayReportIfDue(timestamp, speedMps);

    // Update Gear
    {
        int vehicleGear = mapGearToVehicleGear(gear);
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& gearValue = mCurrentValues[static_cast<int32_t>(VehicleProperty::CURRENT_GEAR)];
        if (gearValue.value.int32Values[0] != vehicleGear) {
            LOG(INFO) << "Gear change: raw=" << gear << " mapped=" << vehicleGear 
                      << " (prev=" << gearValue.value.int32Values[0] << ")";
            gearValue.value.int32Values[0] = vehicleGear;
            gearValue.timestamp = timestamp;
            notifyPropertyChange(static_cast<int32_t>(VehicleProperty::CURRENT_GEAR), gearValue);
            
            auto& selValue = mCurrentValues[static_cast<int32_t>(VehicleProperty::GEAR_SELECTION)];
            selValue.value.int32Values[0] = vehicleGear;
            selValue.timestamp = timestamp;
            notifyPropertyChange(static_cast<int32_t>(VehicleProperty::GEAR_SELECTION), selValue);
        }
    }

    // Parking brake follows the gear: P = on, anything else = off. Published
    // outside the gear-change branch above so a boot straight into N (gear
    // equal to the default) still drops the brake on the first frame.
    {
        int32_t brake = (gear == GEAR_PARK) ? 1 : 0;
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& brakeValue = mCurrentValues[static_cast<int32_t>(VehicleProperty::PARKING_BRAKE_ON)];
        if (brakeValue.value.int32Values[0] != brake) {
            LOG(INFO) << "Parking brake (gear-derived): " << (brake ? "ON" : "OFF");
            brakeValue.value.int32Values[0] = brake;
            brakeValue.timestamp = timestamp;
            notifyPropertyChange(static_cast<int32_t>(VehicleProperty::PARKING_BRAKE_ON), brakeValue);
        }
    }

    // Raw gear/status byte for the Workshop diagnosis row.
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& rawValue = mCurrentValues[VENDOR_RAW_GEAR_STATUS];
        if (rawValue.value.int32Values[0] != statusAndGear) {
            rawValue.value.int32Values[0] = statusAndGear;
            rawValue.timestamp = timestamp;
            notifyPropertyChange(VENDOR_RAW_GEAR_STATUS, rawValue);
        }
    }

    // Update battery charge rate. AOSP defines this property in MILLIWATTS
    // with POSITIVE = charging; our controller reports current with positive
    // = discharge, hence the negation. (Previously published as Watts with
    // the sign inverted; nothing consumed it, the cockpit computes power from
    // the vendor V/A properties.)
    {
        float chargeRateMw = -(voltage * current) * 1000.0f;
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::EV_BATTERY_INSTANTANEOUS_CHARGE_RATE)];
        value.value.floatValues[0] = chargeRateMw;
        value.timestamp = timestamp;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::EV_BATTERY_INSTANTANEOUS_CHARGE_RATE), value);
    }

    // Update Voltage (vendor property)
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[VENDOR_BATTERY_VOLTAGE];
        value.value.floatValues[0] = voltage;
        value.timestamp = timestamp;
        notifyPropertyChange(VENDOR_BATTERY_VOLTAGE, value);
    }

    // Update Current (vendor property) - BMS shunt when fresh, see above
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[VENDOR_BATTERY_CURRENT];
        value.value.floatValues[0] = bestCurrent;
        value.timestamp = timestamp;
        notifyPropertyChange(VENDOR_BATTERY_CURRENT, value);
    }
}

void MotorcycleVehicleHardware::processControllerTemps(const uint8_t* data) {
    // Message 0x10261023 format (from controller spec):
    // Byte 0: Controller Temperature (°C)
    // Byte 1: Motor Temperature (°C)
    // Byte 3: HALL status (bitfield)
    // Byte 4: Throttle Signal (0-100%)
    // Byte 6: Error Codes (bitfield)
    
    int controllerTemp = data[0];
    int motorTemp = data[1];
    int throttle = data[4];  // Byte 4, not byte 2!
    int tempErrors = data[6];
    // Over-voltage (bit 1) is the controller's view of the world; ours is
    // the pack's full-charge ceiling. A full 21s pack is not a fault.
    if ((tempErrors & 0x02) &&
        mLastPackVoltage.load(std::memory_order_relaxed) <= mPackMaxVoltage.load(std::memory_order_relaxed)) {
        tempErrors &= ~0x02;
    }

    static int tempsMsgCount = 0;
    if (mVerboseCanLog && ++tempsMsgCount % 10 == 1) {
        LOG(INFO) << "Controller 0x10261023 temps: data=[" 
                  << (int)data[0] << "," << (int)data[1] << ","
                  << (int)data[2] << "," << (int)data[3] << ","
                  << (int)data[4] << "," << (int)data[5] << ","
                  << (int)data[6] << "," << (int)data[7] << "]"
                  << " -> ctrlTemp=" << controllerTemp << " motorTemp=" << motorTemp
                  << " throttle=" << throttle << " errors=" << tempErrors;
    }

    int64_t timestamp = elapsedRealtimeNano();

    mLastControllerFrameNs.store(timestamp, std::memory_order_relaxed);
    setLinkBit(LINK_CONTROLLER, true);
    // Second fault byte lives in this frame
    updateFaultFlags(tempErrors << 8, FAULT_MASK_CONTROLLER_TEMPS);

    // Update Controller temp (using ENGINE_COOLANT_TEMP)
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::ENGINE_COOLANT_TEMP)];
        value.value.floatValues[0] = static_cast<float>(controllerTemp);
        value.timestamp = timestamp;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::ENGINE_COOLANT_TEMP), value);
    }

    // Update Motor temp (using ENGINE_OIL_TEMP)
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::ENGINE_OIL_TEMP)];
        value.value.floatValues[0] = static_cast<float>(motorTemp);
        value.timestamp = timestamp;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::ENGINE_OIL_TEMP), value);
    }

    // Update Throttle (vendor property)
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[VENDOR_THROTTLE_POSITION];
        value.value.floatValues[0] = static_cast<float>(throttle);
        value.timestamp = timestamp;
        notifyPropertyChange(VENDOR_THROTTLE_POSITION, value);
    }
}

void MotorcycleVehicleHardware::processBmsData(const uint8_t* data) {
    // Orion BMS default broadcast 0x6B1, confirmed on the bike 2026-09-05
    // (raw 00 60 00 1B 13 11 00 58 with the pack at 4.11 V/cell):
    //   bytes 0-1  discharge current limit, A, big-endian
    //   bytes 2-3  charge current limit, A, big-endian
    //   byte 4     highest cell temperature, degC (signed)
    //   byte 5     lowest cell temperature, degC (signed)
    //   byte 6     unused
    //   byte 7     checksum = (sum of bytes 0-6 + length 8 + CAN ID) & 0xFF
    // SoC, pack current and voltage are NOT in here (they live in Orion's
    // 0x6B0 message, which this BMS does not send); they come from the OBD2
    // PIDs. The earlier "byte 3 = SoC" guess was the charge current limit.
    uint32_t sum = 8 + (mCanIdBms.load(std::memory_order_relaxed) & 0xFF);
    for (int i = 0; i < 7; i++) sum += data[i];
    if ((sum & 0xFF) != data[7]) {
        static int badChecksums = 0;
        if (++badChecksums % 100 == 1) {
            LOG(WARNING) << "BMS 0x6B1 checksum mismatch (" << badChecksums << " so far)";
        }
        return;
    }
    mLastBmsFrameNs.store(elapsedRealtimeNano(), std::memory_order_relaxed);
    setLinkBit(LINK_BMS, true);

    float dcl = static_cast<float>((data[0] << 8) | data[1]);
    float ccl = static_cast<float>((data[2] << 8) | data[3]);
    float tHigh = static_cast<float>(static_cast<int8_t>(data[4]));
    float tLow = static_cast<float>(static_cast<int8_t>(data[5]));
    int64_t timestamp = elapsedRealtimeNano();

    static int bmsMsgCount = 0;
    if (mVerboseCanLog && ++bmsMsgCount % 20 == 1) {
        LOG(INFO) << "BMS 0x6B1: DCL " << dcl << " A, CCL " << ccl << " A, cells " << tLow
                  << ".." << tHigh << " C";
    }
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto setF = [&](int32_t prop, float v) {
        auto& value = mCurrentValues[prop];
        if (value.value.floatValues[0] == v && value.timestamp != 0) return;
        value.value.floatValues[0] = v;
        value.timestamp = timestamp;
        notifyPropertyChange(prop, value);
    };
    setF(VENDOR_DISCHARGE_LIMIT, dcl);
    setF(VENDOR_CHARGE_LIMIT, ccl);
    setF(VENDOR_PACK_TEMP_HIGH, tHigh);
    setF(VENDOR_PACK_TEMP_LOW, tLow);
    setF(VENDOR_PACK_TEMP_AVG, (tHigh + tLow) / 2.0f);
}

// ============================================================================
// OBD2 BMS Implementation (Orion BMS Mode 0x22 Extended Diagnostics)
// ============================================================================

void MotorcycleVehicleHardware::updateBmsProperty(int32_t propId, float value) {
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto it = mCurrentValues.find(propId);
    if (it != mCurrentValues.end()) {
        it->second.value.floatValues[0] = value;
        it->second.timestamp = elapsedRealtimeNano();
        notifyPropertyChange(propId, it->second);
    }
}

void MotorcycleVehicleHardware::updateBmsPropertyInt(int32_t propId, int32_t value) {
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto it = mCurrentValues.find(propId);
    if (it != mCurrentValues.end()) {
        it->second.value.int32Values[0] = value;
        it->second.timestamp = elapsedRealtimeNano();
        notifyPropertyChange(propId, it->second);
    }
}

bool MotorcycleVehicleHardware::sendObd2Request(uint16_t pid) {
    if (mCanSocket < 0) {
        return false;
    }

    // OBD2 Mode 0x22 request format:
    // Byte 0: Length (3 bytes follow)
    // Byte 1: Mode (0x22 = Read Data By Identifier)
    // Byte 2-3: PID (big-endian)
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = CAN_ID_OBD2_REQUEST;
    frame.can_dlc = 8;
    frame.data[0] = 0x03;  // 3 bytes follow
    frame.data[1] = 0x22;  // Mode 0x22
    frame.data[2] = (pid >> 8) & 0xFF;  // PID high byte
    frame.data[3] = pid & 0xFF;         // PID low byte
    // Bytes 4-7 are padding (0x00)

    {
        std::lock_guard<std::mutex> lock(mObd2Mutex);
        mPendingPid = pid;
        mObd2ResponseReceived = false;
    }

    ssize_t nbytes = write(mCanSocket, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        LOG(WARNING) << "Failed to send OBD2 request for PID 0x" << std::hex << pid;
        return false;
    }

    captureFrame(frame);
    return true;
}

void MotorcycleVehicleHardware::processObd2Response(const uint8_t* data, uint8_t len) {
    if (len < 4) return;
    mLastBmsFrameNs.store(elapsedRealtimeNano(), std::memory_order_relaxed);
    setLinkBit(LINK_BMS, true);

    // OBD2 Mode 0x22 response format:
    // Byte 0: Length
    // Byte 1: Mode + 0x40 (0x62 for Mode 0x22 response)
    // Byte 2-3: PID (big-endian)
    // Byte 4+: Data
    
    if (data[1] != 0x62) {
        // Not a Mode 0x22 response
        return;
    }

    uint16_t pid = (data[2] << 8) | data[3];
    uint8_t dataLen = data[0] - 3;  // Subtract mode + PID bytes

    // Notify waiting thread
    {
        std::lock_guard<std::mutex> lock(mObd2Mutex);
        if (pid == mPendingPid) {
            mObd2ResponseReceived = true;
            mObd2ResponseCv.notify_one();
        }
    }

    // Parse response based on PID
    switch (pid) {
        case BMS_PID_PACK_SOC: {
            // Authoritative SoC (0.5%/bit). Outranks the 0x6B1 broadcast, whose
            // byte layout is inferred from one observed frame.
            if (dataLen >= 1) {
                float soc = static_cast<float>(data[4]) * 0.5f;
                mLastSocPidNs = elapsedRealtimeNano();
                {
                    std::lock_guard<std::mutex> lock(mValuesMutex);
                    auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL)];
                    value.value.floatValues[0] = soc;
                    value.timestamp = mLastSocPidNs;
                    notifyPropertyChange(static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL), value);
                }
                mLastSocPercent = soc;
                publishRange(mLastSocPidNs);
            }
            break;
        }
        case BMS_PID_PACK_CURRENT: {
            if (dataLen >= 2) {
                int16_t raw = static_cast<int16_t>((data[4] << 8) | data[5]);
                mBmsCurrentA = raw * 0.1f;
                mLastBmsCurrentNs = elapsedRealtimeNano();
                updateBmsProperty(VENDOR_PACK_CURRENT, mBmsCurrentA);
            }
            break;
        }
        case BMS_PID_PACK_SOH: {
            // 1 byte, direct percentage (no scaling per Orion spec)
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_SOH, static_cast<float>(data[4]));
            }
            break;
        }
        case BMS_PID_PACK_DOD: {
            // 1 byte, 0.5% scaling per Orion spec
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_DOD, static_cast<float>(data[4]) * 0.5f);
            }
            break;
        }
        case BMS_PID_PACK_CYCLES: {
            // 2 bytes, direct count
            if (dataLen >= 2) {
                int cycles = (data[4] << 8) | data[5];
                updateBmsPropertyInt(VENDOR_PACK_CYCLES, cycles);
            }
            break;
        }
        // Temperatures are plain signed degC, NO -40 offset: confirmed on the
        // bike 2026-09-05 (0xF028/F029 answered 20/18, identical to the 0x6B1
        // broadcast's high/low; the heatsink had shown -15 with the offset).
        case BMS_PID_TEMP_HIGH: {
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_TEMP_HIGH, static_cast<float>(static_cast<int8_t>(data[4])));
            }
            break;
        }
        case BMS_PID_TEMP_LOW: {
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_TEMP_LOW, static_cast<float>(static_cast<int8_t>(data[4])));
            }
            break;
        }
        case BMS_PID_TEMP_AVG: {
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_TEMP_AVG, static_cast<float>(static_cast<int8_t>(data[4])));
            }
            break;
        }
        case BMS_PID_HEATSINK_TEMP: {
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_HEATSINK_TEMP, static_cast<float>(static_cast<int8_t>(data[4])));
            }
            break;
        }
        case BMS_PID_FAN_SPEED: {
            if (dataLen >= 1) {
                updateBmsPropertyInt(VENDOR_FAN_SPEED, data[4]);
            }
            break;
        }
        case BMS_PID_CELL_LOW: {
            // 2 bytes, 0.0001V resolution
            if (dataLen >= 2) {
                int raw = (data[4] << 8) | data[5];
                updateBmsProperty(VENDOR_CELL_VOLTAGE_LOW, raw * 0.0001f);
            }
            break;
        }
        case BMS_PID_CELL_HIGH: {
            if (dataLen >= 2) {
                int raw = (data[4] << 8) | data[5];
                updateBmsProperty(VENDOR_CELL_VOLTAGE_HIGH, raw * 0.0001f);
            }
            break;
        }
        case BMS_PID_CELL_AVG: {
            if (dataLen >= 2) {
                int raw = (data[4] << 8) | data[5];
                updateBmsProperty(VENDOR_CELL_VOLTAGE_AVG, raw * 0.0001f);
            }
            break;
        }
        case BMS_PID_CELL_LOW_ID: {
            if (dataLen >= 2) {
                int cellId = (data[4] << 8) | data[5];
                updateBmsPropertyInt(VENDOR_CELL_LOW_ID, cellId);
            }
            break;
        }
        case BMS_PID_CELL_HIGH_ID: {
            if (dataLen >= 2) {
                int cellId = (data[4] << 8) | data[5];
                updateBmsPropertyInt(VENDOR_CELL_HIGH_ID, cellId);
            }
            break;
        }
        case BMS_PID_CHARGE_LIMIT: {
            // 2 bytes, direct amps
            if (dataLen >= 2) {
                int amps = (data[4] << 8) | data[5];
                updateBmsProperty(VENDOR_CHARGE_LIMIT, static_cast<float>(amps));
            }
            break;
        }
        case BMS_PID_DISCHARGE_LIMIT: {
            if (dataLen >= 2) {
                int amps = (data[4] << 8) | data[5];
                updateBmsProperty(VENDOR_DISCHARGE_LIMIT, static_cast<float>(amps));
            }
            break;
        }
        case BMS_PID_PACK_AMPHOURS: {
            // 2 bytes, 0.1Ah resolution
            if (dataLen >= 2) {
                int raw = (data[4] << 8) | data[5];
                updateBmsProperty(VENDOR_PACK_AMPHOURS, raw * 0.1f);
            }
            break;
        }
        case BMS_PID_PACK_RESISTANCE: {
            // 2 bytes, 0.01mOhm resolution
            if (dataLen >= 2) {
                int raw = (data[4] << 8) | data[5];
                updateBmsProperty(VENDOR_PACK_RESISTANCE, raw * 0.01f);
            }
            break;
        }
        default:
            LOG(DEBUG) << "Unknown BMS PID response: 0x" << std::hex << pid;
            break;
    }
}

void MotorcycleVehicleHardware::bmsPollingThread() {
    LOG(INFO) << "BMS polling thread starting...";
    
    // Wait for CAN socket to be ready
    while (mRunning && mCanSocket < 0) {
        sleepUnlessStopping(1000);
    }
    
    if (!mRunning) return;
    
    LOG(INFO) << "BMS polling thread active, starting OBD2 queries";
    
    // List of PIDs to poll (slower-changing data)
    const std::vector<uint16_t> pollPids = {
        BMS_PID_PACK_SOH,
        BMS_PID_PACK_DOD,
        BMS_PID_PACK_CYCLES,
        BMS_PID_TEMP_HIGH,
        BMS_PID_TEMP_LOW,
        BMS_PID_TEMP_AVG,
        BMS_PID_HEATSINK_TEMP,
        BMS_PID_FAN_SPEED,
        BMS_PID_CELL_LOW,
        BMS_PID_CELL_HIGH,
        BMS_PID_CELL_AVG,
        BMS_PID_CELL_LOW_ID,
        BMS_PID_CELL_HIGH_ID,
        BMS_PID_CHARGE_LIMIT,
        BMS_PID_DISCHARGE_LIMIT,
        BMS_PID_PACK_AMPHOURS,
        BMS_PID_PACK_RESISTANCE,
    };
    
    // SoC and signed current are polled every round (they drive the battery
    // display and charging detection); the slow PIDs rotate one per round.
    const uint16_t fastPids[] = {BMS_PID_PACK_SOC, BMS_PID_PACK_CURRENT};
    auto query = [this](uint16_t pid) {
        if (sendObd2Request(pid)) {
            std::unique_lock<std::mutex> lock(mObd2Mutex);
            mObd2ResponseCv.wait_for(lock, std::chrono::milliseconds(100),
                [this] { return mObd2ResponseReceived || !mRunning; });
        }
    };
    size_t pidIndex = 0;
    bool firstPassDone = false;
    bool waitingLogged = false;
    while (mRunning) {
        // Only poll a BMS that is demonstrably on the bus (0x6B1 seen in the
        // last 5 s). Polling a silent bus leaves every request unACKed and
        // drives the CAN controller to bus-off within seconds - seen on the
        // bike 2026-09-05 before the bus was wired, and the same would happen
        // whenever the Pi boots ahead of the BMS.
        constexpr int64_t kBmsAliveNs = 5LL * 1000000000LL;
        int64_t lastBms = mLastBmsFrameNs.load(std::memory_order_relaxed);
        if (lastBms == 0 || elapsedRealtimeNano() - lastBms > kBmsAliveNs) {
            if (!waitingLogged) {
                LOG(INFO) << "BMS polling paused: no 0x6B1 broadcast on the bus";
                waitingLogged = true;
            }
            if (!sleepUnlessStopping(500)) break;
            continue;
        }
        if (waitingLogged) {
            LOG(INFO) << "BMS broadcast seen, OBD2 polling resumed";
            waitingLogged = false;
        }
        for (uint16_t pid : fastPids) {
            query(pid);
        }
        query(pollPids[pidIndex]);
        pidIndex = (pidIndex + 1) % pollPids.size();
        if (pidIndex == 0) {
            firstPassDone = true;
        }
        if (!sleepUnlessStopping(firstPassDone ? 700 : 100)) break;
    }
    LOG(INFO) << "BMS polling thread exiting";
}

// Distance/odometer persistence limits. Odometer values live in
// persist.vendor.motodash.* so they survive a reboot; writes are throttled
// because every persist property write rewrites the persistent property file.
namespace {
constexpr double kPersistDistanceThresholdM = 500.0;
constexpr int64_t kPersistIntervalNs = 30LL * 1000000000LL;
// Ignore implausible gaps (first frame after boot, or a pause in traffic) so a
// stale timestamp cannot add a large bogus distance in one step.
constexpr int64_t kMaxDistanceStepNs = 1000000000LL;
// Range model: consumption is learned per completed distance chunk and folded
// into an EMA. 200m chunks with alpha 0.1 give a time constant of ~2km, slow
// enough to ride out a single overtake, fast enough to notice a headwind.
constexpr double kRangeChunkMeters = 200.0;
constexpr float kWhPerKmEmaAlpha = 0.1f;
// A chunk outside these bounds is discarded as a decode glitch rather than
// folded into the average. Net-regen chunks (long downhill) clamp to the
// minimum instead of going negative.
constexpr float kMinChunkWhPerKm = 0.0f;
constexpr float kMaxChunkWhPerKm = 500.0f;
// Below this learned consumption the projection would be absurd (and divide
// toward infinity); treat the model as not yet trustworthy.
constexpr float kMinUsableWhPerKm = 5.0f;
// Only integrate energy while actually moving: charging at standstill would
// otherwise pour negative energy into the open chunk and poison the average.
constexpr float kEnergyIntegrationMinSpeedMps = 0.5f;
}  // namespace

void MotorcycleVehicleHardware::updateFaultFlags(int32_t newBits, int32_t mask) {
    int32_t updated = (mFaultFlags & ~mask) | (newBits & mask);
    if (updated == mFaultFlags) {
        return;  // ON_CHANGE: only notify on an actual transition
    }
    LOG(INFO) << "Fault flags changed: 0x" << std::hex << mFaultFlags << " -> 0x" << updated
              << std::dec;
    mFaultFlags = updated;

    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto& value = mCurrentValues[VENDOR_FAULT_FLAGS];
    value.value.int32Values[0] = updated;
    value.timestamp = elapsedRealtimeNano();
    notifyPropertyChange(VENDOR_FAULT_FLAGS, value);
}

void MotorcycleVehicleHardware::updateStatusFlags(int32_t bits) {
    if (bits == mStatusFlags) {
        return;
    }
    mStatusFlags = bits;
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto& value = mCurrentValues[VENDOR_STATUS_FLAGS];
    value.value.int32Values[0] = bits;
    value.timestamp = elapsedRealtimeNano();
    notifyPropertyChange(VENDOR_STATUS_FLAGS, value);
}

// The stock display reports odometer/trip/speed back to the controller at
// 250ms on 0x1026105A. We do the same so the controller sees a compliant
// display. Layout per the controller spec: data0/data2 odometer km (16-bit,
// low/high), data3/data4 trip km, data5 speed km/h, data6 rolling counter.
void MotorcycleVehicleHardware::sendDisplayReportIfDue(int64_t timestamp, float speedMps) {
    if (mCanSocket < 0 || (timestamp - mLastDisplayReportTimestamp) < 250000000LL) {
        return;
    }
    mLastDisplayReportTimestamp = timestamp;

    uint32_t odoKm = static_cast<uint32_t>(mOdometerMeters / 1000.0);
    if (odoKm > 99999) odoKm = 99999;
    uint32_t tripKm = static_cast<uint32_t>(mTripMeters / 1000.0);
    if (tripKm > 1024) tripKm = 1024;
    uint32_t speedKmh = static_cast<uint32_t>(speedMps * 3.6f);
    if (speedKmh > 199) speedKmh = 199;
    static uint8_t counter = 0;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = CAN_ID_DISPLAY_REPORT | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    frame.data[0] = odoKm & 0xFF;
    frame.data[2] = (odoKm >> 8) & 0xFF;
    frame.data[3] = tripKm & 0xFF;
    frame.data[4] = (tripKm >> 8) & 0xFF;
    frame.data[5] = static_cast<uint8_t>(speedKmh);
    frame.data[6] = counter++;

    if (write(mCanSocket, &frame, sizeof(frame)) != sizeof(frame)) {
        static int failCount = 0;
        if (++failCount % 100 == 1) {
            LOG(WARNING) << "Failed to send display report: " << strerror(errno);
        }
        return;
    }
    captureFrame(frame);
}

void MotorcycleVehicleHardware::setLinkBit(int32_t bit, bool alive) {
    std::lock_guard<std::mutex> lock(mValuesMutex);
    int32_t updated = alive ? (mLinkStatus | bit) : (mLinkStatus & ~bit);
    if (updated == mLinkStatus) {
        return;
    }
    LOG(INFO) << "Link status: 0x" << std::hex << mLinkStatus << " -> 0x" << updated << std::dec;
    mLinkStatus = updated;
    auto& value = mCurrentValues[VENDOR_LINK_STATUS];
    value.value.int32Values[0] = updated;
    value.timestamp = elapsedRealtimeNano();
    notifyPropertyChange(VENDOR_LINK_STATUS, value);
}

void MotorcycleVehicleHardware::checkLinkTimeouts(int64_t nowNs) {
    // Controller broadcasts at 50ms; 1s of silence means the link is gone.
    // The BMS broadcast rate is unconfirmed, so it gets a longer leash.
    constexpr int64_t kControllerTimeoutNs = 1000000000LL;
    constexpr int64_t kBmsTimeoutNs = 5000000000LL;

    int64_t lastController = mLastControllerFrameNs.load(std::memory_order_relaxed);
    bool controllerDead = lastController != 0 && nowNs - lastController > kControllerTimeoutNs;
    if (controllerDead) {
        setLinkBit(LINK_CONTROLLER, false);
    }
    endRideIfDue(nowNs, controllerDead);
    int64_t lastBms = mLastBmsFrameNs.load(std::memory_order_relaxed);
    if (lastBms != 0 && nowNs - lastBms > kBmsTimeoutNs) {
        setLinkBit(LINK_BMS, false);
    }
}

void MotorcycleVehicleHardware::linkWatchdogThread() {
    while (sleepUnlessStopping(500)) {
        checkLinkTimeouts(elapsedRealtimeNano());
    }
}

void MotorcycleVehicleHardware::updateChargingState(int rpm, float current, int64_t nowNs) {
    // Standstill with charge current flowing in. rpm==0 excludes rolling
    // regen - but the instant the wheel stops after regen braking, the pack
    // current is still negative for a beat, which flashed the charging
    // takeover during a simulated stop (Christian caught it watching the
    // ride replay). Charging is therefore only declared once standstill AND
    // charge current have held together for a dwell period; a regen tail
    // dies in well under a second, a real charger holds indefinitely.
    // Movement or the current drying up cancels instantly.
    bool candidate = (rpm == 0 && current <= CHARGING_CURRENT_THRESHOLD_A);
    if (!candidate) {
        mChargingCandidateSinceNs = 0;
    } else if (mChargingCandidateSinceNs == 0) {
        mChargingCandidateSinceNs = nowNs;
    }
    int32_t charging = (candidate &&
            (nowNs - mChargingCandidateSinceNs) >=
                    mChargingDwellNs.load(std::memory_order_relaxed)) ? 1 : 0;
    if (charging == mCharging) {
        return;
    }
    mCharging = charging;
    LOG(INFO) << (charging ? "Charging detected" : "Charging ended");
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto& value = mCurrentValues[VENDOR_CHARGING];
    value.value.int32Values[0] = charging;
    value.timestamp = elapsedRealtimeNano();
    notifyPropertyChange(VENDOR_CHARGING, value);
}

// ---------------------------------------------------------------------------
// Ride summary
// ---------------------------------------------------------------------------
namespace {
constexpr float kRideMovingSpeedMps = 0.5f;
constexpr int64_t kRideStandstillEndNs = 5LL * 60 * 1000000000LL;   // 5 min parked = ride over
constexpr double kRideMinMeters = 200.0;                             // shorter is a shuffle, not a ride
}  // namespace

void MotorcycleVehicleHardware::trackRide(float speedMps, int64_t timestamp) {
    std::lock_guard<std::mutex> lock(mRideMutex);
    bool moving = speedMps > kRideMovingSpeedMps;
    if (!mRideActive) {
        if (!moving) return;
        mRideActive = true;
        mRideStartMeters = mOdometerMeters;
        mRideStartNs = timestamp;
        mRideMovingNs = 0;
        mRideEnergyWh = 0.0;
        mRideMaxSpeedMps = 0.0f;
        mRideMaxLeanL = mRideMaxLeanR = 0.0f;
        mRideLastTrackNs = timestamp;
        mRideLastMoveNs = timestamp;
        LOG(INFO) << "Ride started at odo " << mOdometerMeters << " m";
        return;
    }
    int64_t delta = timestamp - mRideLastTrackNs;
    mRideLastTrackNs = timestamp;
    if (moving) {
        if (delta > 0 && delta < kMaxDistanceStepNs) mRideMovingNs += delta;
        mRideLastMoveNs = timestamp;
        if (speedMps > mRideMaxSpeedMps) mRideMaxSpeedMps = speedMps;
    }
}

void MotorcycleVehicleHardware::addRideEnergy(double wh) {
    std::lock_guard<std::mutex> lock(mRideMutex);
    if (mRideActive) mRideEnergyWh += wh;
}

void MotorcycleVehicleHardware::endRideIfDue(int64_t nowNs, bool linkDead) {
    float meters, seconds, whPerKm, maxMps, maxLeanL, maxLeanR;
    int32_t seq;
    {
        std::lock_guard<std::mutex> lock(mRideMutex);
        if (!mRideActive) return;
        bool parkedLong = (nowNs - mRideLastMoveNs) > kRideStandstillEndNs;
        if (!linkDead && !parkedLong) return;
        mRideActive = false;
        double distance = mOdometerMeters - mRideStartMeters;
        if (distance < kRideMinMeters) {
            LOG(INFO) << "Ride ended after " << distance << " m - too short to summarise";
            return;
        }
        meters = static_cast<float>(distance);
        seconds = static_cast<float>(mRideMovingNs / 1e9);
        whPerKm = static_cast<float>(mRideEnergyWh / (distance / 1000.0));
        maxMps = mRideMaxSpeedMps;
        maxLeanL = mRideMaxLeanL;
        maxLeanR = mRideMaxLeanR;
        seq = ++mRideSeq;
    }
    LOG(INFO) << "Ride #" << seq << " ended (" << (linkDead ? "key off" : "parked") << "): "
              << meters << " m, " << seconds << " s moving, " << whPerKm << " Wh/km, max "
              << maxMps << " m/s, lean " << maxLeanL << "L/" << maxLeanR << "R";
    publishRideSummary(meters, seconds, whPerKm, maxMps, maxLeanL, maxLeanR, seq, nowNs);
    persistConfig("persist.vendor.motodash.ride.maxleanl", std::to_string(maxLeanL));
    persistConfig("persist.vendor.motodash.ride.maxleanr", std::to_string(maxLeanR));
    persistConfig("persist.vendor.motodash.ride.meters", std::to_string(meters));
    persistConfig("persist.vendor.motodash.ride.seconds", std::to_string(seconds));
    persistConfig("persist.vendor.motodash.ride.whperkm", std::to_string(whPerKm));
    persistConfig("persist.vendor.motodash.ride.maxmps", std::to_string(maxMps));
    persistConfig("persist.vendor.motodash.ride.seq", std::to_string(seq));
}

void MotorcycleVehicleHardware::publishRideSummary(float meters, float seconds, float whPerKm,
                                                   float maxMps, float maxLeanL, float maxLeanR,
                                                   int32_t seq, int64_t timestamp) {
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto setF = [&](int32_t prop, float v) {
        auto& value = mCurrentValues[prop];
        value.value.floatValues[0] = v;
        value.timestamp = timestamp;
        notifyPropertyChange(prop, value);
    };
    setF(VENDOR_RIDE_DISTANCE_M, meters);
    setF(VENDOR_RIDE_DURATION_S, seconds);
    setF(VENDOR_RIDE_WH_PER_KM, whPerKm);
    setF(VENDOR_RIDE_MAX_SPEED_MPS, maxMps);
    setF(VENDOR_RIDE_MAX_LEAN_L, maxLeanL);
    setF(VENDOR_RIDE_MAX_LEAN_R, maxLeanR);
    auto& s = mCurrentValues[VENDOR_RIDE_SEQ];
    s.value.int32Values[0] = seq;
    s.timestamp = timestamp;
    notifyPropertyChange(VENDOR_RIDE_SEQ, s);   // last, so listeners see complete values
}

void MotorcycleVehicleHardware::accumulateDistance(float speedMps, int64_t timestamp) {
    if (mTripResetRequested.exchange(false)) {
        mTripMeters = 0.0;
        mPersistedTripMeters = -1.0;  // force a persist on the next check
    }

    int64_t deltaNs = timestamp - mLastDistanceTimestamp;
    mLastDistanceTimestamp = timestamp;
    if (deltaNs <= 0 || deltaNs > kMaxDistanceStepNs || speedMps <= 0.0f) {
        return;
    }

    double metres = static_cast<double>(speedMps) * (static_cast<double>(deltaNs) / 1e9);
    mOdometerMeters += metres;
    mTripMeters += metres;
    trackRide(speedMps, timestamp);
}

void MotorcycleVehicleHardware::accumulateEnergy(float voltage, float current, float speedMps,
                                                 int64_t timestamp) {
    int64_t deltaNs = timestamp - mLastEnergyTimestamp;
    mLastEnergyTimestamp = timestamp;
    if (deltaNs <= 0 || deltaNs > kMaxDistanceStepNs
            || speedMps < kEnergyIntegrationMinSpeedMps) {
        return;
    }
    if (mChunkStartMeters < 0.0) {
        mChunkStartMeters = mOdometerMeters;
    }

    // Positive current = discharge, so V*I is consumption; regen while moving
    // subtracts, which is exactly what it does to real consumption.
    double dtHours = static_cast<double>(deltaNs) / 3.6e12;
    double wh = static_cast<double>(voltage) * static_cast<double>(current) * dtHours;
    mChunkEnergyWh += wh;
    addRideEnergy(wh);

    double chunkMeters = mOdometerMeters - mChunkStartMeters;
    if (chunkMeters < kRangeChunkMeters) {
        return;
    }
    float chunkWhPerKm = static_cast<float>(mChunkEnergyWh / (chunkMeters / 1000.0));
    mChunkEnergyWh = 0.0;
    mChunkStartMeters = mOdometerMeters;
    if (chunkWhPerKm > kMaxChunkWhPerKm) {
        LOG(WARNING) << "Discarding implausible consumption chunk: " << chunkWhPerKm << " Wh/km";
        return;
    }
    if (chunkWhPerKm < kMinChunkWhPerKm) {
        chunkWhPerKm = kMinChunkWhPerKm;
    }
    mWhPerKm = (mWhPerKm > 0.0f)
            ? mWhPerKm * (1.0f - kWhPerKmEmaAlpha) + chunkWhPerKm * kWhPerKmEmaAlpha
            : chunkWhPerKm;
    publishRange(timestamp);
}

void MotorcycleVehicleHardware::publishRange(int64_t timestamp) {
    float rangeMeters = 0.0f;  // 0 = unknown; the UI shows "--"
    if (mWhPerKm >= kMinUsableWhPerKm && mLastSocPercent >= 0.0f) {
        float remainingWh = (mLastSocPercent / 100.0f) * mPackEnergyWh.load();
        rangeMeters = remainingWh / mWhPerKm * 1000.0f;
    }
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::RANGE_REMAINING)];
    value.value.floatValues[0] = rangeMeters;
    value.timestamp = timestamp;
    notifyPropertyChange(static_cast<int32_t>(VehicleProperty::RANGE_REMAINING), value);
}

void MotorcycleVehicleHardware::publishDistance(int64_t timestamp) {
    std::lock_guard<std::mutex> lock(mValuesMutex);

    auto& odo = mCurrentValues[static_cast<int32_t>(VehicleProperty::PERF_ODOMETER)];
    odo.value.floatValues[0] = static_cast<float>(mOdometerMeters / 1000.0);
    odo.timestamp = timestamp;
    notifyPropertyChange(static_cast<int32_t>(VehicleProperty::PERF_ODOMETER), odo);

    auto& trip = mCurrentValues[VENDOR_TRIP_DISTANCE];
    trip.value.floatValues[0] = static_cast<float>(mTripMeters / 1000.0);
    trip.timestamp = timestamp;
    notifyPropertyChange(VENDOR_TRIP_DISTANCE, trip);
}

void MotorcycleVehicleHardware::persistDistanceIfDue(int64_t timestamp, bool force) {
    bool distanceDue = std::abs(mOdometerMeters - mPersistedOdometerMeters)
                               >= kPersistDistanceThresholdM
                       || std::abs(mTripMeters - mPersistedTripMeters)
                               >= kPersistDistanceThresholdM;
    bool timeDue = (timestamp - mLastPersistTimestamp) >= kPersistIntervalNs;
    if (!force && !(distanceDue && timeDue)) {
        return;
    }
    if (mOdometerMeters == mPersistedOdometerMeters && mTripMeters == mPersistedTripMeters) {
        return;
    }

    persistConfig("persist.vendor.motodash.odometer",
                  std::to_string(static_cast<long long>(mOdometerMeters)));
    persistConfig("persist.vendor.motodash.trip",
                  std::to_string(static_cast<long long>(mTripMeters)));
    // The learned consumption rides along on the odometer's persist cycle so
    // a fresh boot starts with last ride's Wh/km instead of relearning.
    if (mWhPerKm > 0.0f && std::abs(mWhPerKm - mPersistedWhPerKm) > 0.5f) {
        persistConfig("persist.vendor.motodash.whperkm", std::to_string(mWhPerKm));
        mPersistedWhPerKm = mWhPerKm;
    }
    mPersistedOdometerMeters = mOdometerMeters;
    mPersistedTripMeters = mTripMeters;
    mLastPersistTimestamp = timestamp;
}

float MotorcycleVehicleHardware::calculateSpeedFromRpm(int rpm) const {
    // Speed (m/s) = RPM * wheel_circumference / (gear_ratio * 60)
    return static_cast<float>(rpm) * mWheelCircumference.load(std::memory_order_relaxed) /
           (mGearRatio.load(std::memory_order_relaxed) * 60.0f);
}

int MotorcycleVehicleHardware::mapGearToVehicleGear(int controllerGear) const {
    switch (controllerGear) {
        case GEAR_PARK:
            return static_cast<int>(VehicleGear::GEAR_PARK);
        case GEAR_REVERSE:
            return static_cast<int>(VehicleGear::GEAR_REVERSE);
        case GEAR_NEUTRAL:
            return static_cast<int>(VehicleGear::GEAR_NEUTRAL);
        case GEAR_DRIVE:
            return static_cast<int>(VehicleGear::GEAR_DRIVE);
        default:
            return static_cast<int>(VehicleGear::GEAR_NEUTRAL);
    }
}

void MotorcycleVehicleHardware::captureFrame(const struct can_frame& frame) {
    if (!mCaptureEnabled.load(std::memory_order_relaxed)) {
        return;
    }
    // 256 MB is ~30 hours of this bus; stopping beats filling /data.
    constexpr uint64_t kMaxCaptureBytes = 256ULL * 1024 * 1024;
    constexpr int64_t kSyncIntervalNs = 1000000000LL;

    std::lock_guard<std::mutex> lock(mCaptureMutex);
    if (mCaptureFd < 0) {
        struct timeval now;
        gettimeofday(&now, nullptr);
        std::string path = mCaptureDir + "/can-" + std::to_string(now.tv_sec) + ".log";
        mCaptureFd = open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0640);
        if (mCaptureFd < 0) {
            if (!mCaptureFailureLogged) {
                LOG(ERROR) << "CAN capture: cannot open " << path << ": " << strerror(errno);
                mCaptureFailureLogged = true;
            }
            return;
        }
        LOG(INFO) << "CAN capture started: " << path;
        mCaptureBytes = 0;
    }
    if (mCaptureBytes >= kMaxCaptureBytes) {
        return;
    }

    // candump -l format: (sec.usec) iface ID#DATA, 8 hex digits for extended
    // IDs, 3 for standard - exactly what moto_can_replay -f consumes.
    struct timeval now;
    gettimeofday(&now, nullptr);
    char line[96];
    int len = snprintf(line, sizeof(line), "(%ld.%06ld) %s ", static_cast<long>(now.tv_sec),
                       static_cast<long>(now.tv_usec), mCanInterface.c_str());
    if (frame.can_id & CAN_EFF_FLAG) {
        len += snprintf(line + len, sizeof(line) - len, "%08X#", frame.can_id & CAN_EFF_MASK);
    } else {
        len += snprintf(line + len, sizeof(line) - len, "%03X#", frame.can_id & CAN_SFF_MASK);
    }
    for (int i = 0; i < frame.can_dlc && i < 8; i++) {
        len += snprintf(line + len, sizeof(line) - len, "%02X", frame.data[i]);
    }
    line[len++] = '\n';
    if (write(mCaptureFd, line, len) != len) {
        if (!mCaptureFailureLogged) {
            LOG(ERROR) << "CAN capture: write failed: " << strerror(errno);
            mCaptureFailureLogged = true;
        }
        return;
    }
    mCaptureBytes += len;
    if (mCaptureBytes >= kMaxCaptureBytes) {
        LOG(WARNING) << "CAN capture: size cap reached, capture stopped";
    }
    // Key-off is a power cut on the bike: sync often enough that a ride
    // loses at most its last second.
    int64_t nowNs = elapsedRealtimeNano();
    if (nowNs - mLastCaptureSyncNs > kSyncIntervalNs) {
        fdatasync(mCaptureFd);
        mLastCaptureSyncNs = nowNs;
    }
}

void MotorcycleVehicleHardware::closeCapture() {
    std::lock_guard<std::mutex> lock(mCaptureMutex);
    if (mCaptureFd >= 0) {
        fdatasync(mCaptureFd);
        close(mCaptureFd);
        mCaptureFd = -1;
        LOG(INFO) << "CAN capture closed after " << mCaptureBytes << " bytes";
    }
}

void MotorcycleVehicleHardware::notifyPropertyChange(int32_t propId, const VehiclePropValue& value) {
    static int speedNotifyCount = 0;
    if (mOnPropertyChangeCallback) {
        std::vector<VehiclePropValue> values = {value};
        // Log PERF_VEHICLE_SPEED specifically since app subscribes to it
        if (mVerboseCanLog && propId == 291504647) {  // PERF_VEHICLE_SPEED
            if (++speedNotifyCount % 30 == 1) {
                LOG(INFO) << "SPEED notification #" << speedNotifyCount << " value=" 
                          << (value.value.floatValues.empty() ? 0 : value.value.floatValues[0]);
            }
        }
        // Log turn signal and high beam notifications
        if (mVerboseCanLog && (propId == 289408560 || propId == 289410562)) {
            LOG(INFO) << "Calling callback for propId=" << propId << " value=" 
                      << (value.value.int32Values.empty() ? -1 : value.value.int32Values[0]);
        }
        (*mOnPropertyChangeCallback)(values);
    } else {
        LOG(WARNING) << "No callback registered for propId=" << propId;
    }
}

std::vector<VehiclePropConfig> MotorcycleVehicleHardware::getAllPropertyConfigs() const {
    return mPropertyConfigs;
}

StatusCode MotorcycleVehicleHardware::setValues(
        std::shared_ptr<const SetValuesCallback> callback,
        const std::vector<SetValueRequest>& requests) {
    // Only the VENDOR_CFG_* configuration properties are writable;
    // everything else is read-only data from the CAN bus.
    std::vector<SetValueResult> results;
    for (const auto& request : requests) {
        SetValueResult result;
        result.requestId = request.requestId;
        result.status = applyConfigValue(request.value);
        results.push_back(result);
    }
    (*callback)(results);
    return StatusCode::OK;
}

StatusCode MotorcycleVehicleHardware::applyConfigValue(const VehiclePropValue& value) {
    const int32_t propId = value.prop;

    auto floatArg = [&value](float min, float max, float* out) {
        if (value.value.floatValues.size() != 1) return false;
        float v = value.value.floatValues[0];
        if (v < min || v > max) return false;
        *out = v;
        return true;
    };
    auto intArg = [&value](int32_t min, int32_t max, int32_t* out) {
        if (value.value.int32Values.size() != 1) return false;
        int32_t v = value.value.int32Values[0];
        if (v < min || v > max) return false;
        *out = v;
        return true;
    };

    switch (propId) {
        case VENDOR_CFG_WHEEL_CIRCUMFERENCE: {
            float v;
            if (!floatArg(CFG_MIN_WHEEL_CIRCUMFERENCE_M, CFG_MAX_WHEEL_CIRCUMFERENCE_M, &v)) {
                return StatusCode::INVALID_ARG;
            }
            mWheelCircumference = v;
            persistConfig("persist.vendor.motodash.cfg.wheel_circumference", std::to_string(v));
            break;
        }
        case VENDOR_CFG_GEAR_RATIO: {
            float v;
            if (!floatArg(CFG_MIN_GEAR_RATIO, CFG_MAX_GEAR_RATIO, &v)) {
                return StatusCode::INVALID_ARG;
            }
            mGearRatio = v;
            persistConfig("persist.vendor.motodash.cfg.gear_ratio", std::to_string(v));
            break;
        }
        case VENDOR_CFG_CAN_ID_CONTROLLER_STATUS: {
            int32_t v;
            if (!intArg(1, CFG_MAX_CAN_ID, &v)) return StatusCode::INVALID_ARG;
            mCanIdControllerStatus = static_cast<uint32_t>(v);
            persistConfig("persist.vendor.motodash.cfg.can_id_controller_status", std::to_string(v));
            break;
        }
        case VENDOR_CFG_CAN_ID_CONTROLLER_TEMPS: {
            int32_t v;
            if (!intArg(1, CFG_MAX_CAN_ID, &v)) return StatusCode::INVALID_ARG;
            mCanIdControllerTemps = static_cast<uint32_t>(v);
            persistConfig("persist.vendor.motodash.cfg.can_id_controller_temps", std::to_string(v));
            break;
        }
        case VENDOR_CFG_CAN_ID_BMS: {
            int32_t v;
            if (!intArg(1, CFG_MAX_CAN_ID, &v)) return StatusCode::INVALID_ARG;
            mCanIdBms = static_cast<uint32_t>(v);
            persistConfig("persist.vendor.motodash.cfg.can_id_bms", std::to_string(v));
            break;
        }
        case VENDOR_CFG_GPIO_LEFT_TURN: {
            int32_t v;
            if (!intArg(-1, CFG_MAX_GPIO_PIN, &v)) return StatusCode::INVALID_ARG;
            mGpioLeftTurnPin = v;  // GPIO lines are requested at startup; new pin applies on next boot
            persistConfig("persist.vendor.motodash.gpio.left_turn", std::to_string(v));
            break;
        }
        case VENDOR_CFG_GPIO_RIGHT_TURN: {
            int32_t v;
            if (!intArg(-1, CFG_MAX_GPIO_PIN, &v)) return StatusCode::INVALID_ARG;
            mGpioRightTurnPin = v;
            persistConfig("persist.vendor.motodash.gpio.right_turn", std::to_string(v));
            break;
        }
        case VENDOR_CFG_GPIO_HIGH_BEAM: {
            int32_t v;
            if (!intArg(-1, CFG_MAX_GPIO_PIN, &v)) return StatusCode::INVALID_ARG;
            mGpioHighBeamPin = v;
            persistConfig("persist.vendor.motodash.gpio.high_beam", std::to_string(v));
            break;
        }
        case VENDOR_TRIP_DISTANCE: {
            // Any write resets the trip meter; the UI writes 0. Handled by the
            // CAN reader thread so it cannot race distance accumulation.
            if (value.value.floatValues.size() != 1) return StatusCode::INVALID_ARG;
            mTripResetRequested = true;
            LOG(INFO) << "Trip meter reset requested";
            return StatusCode::OK;
        }
        case VENDOR_CFG_GEAR_BASE: {
            int32_t v;
            if (!intArg(0, 1, &v)) return StatusCode::INVALID_ARG;
            mGearBase = v;
            persistConfig("persist.vendor.motodash.cfg.gear_base", std::to_string(v));
            LOG(INFO) << "Gear base set to " << v << " (raw " << v << " = P)";
            break;
        }
        case VENDOR_CFG_IMU_LEVEL: {
            int32_t v;
            if (!intArg(0, 1, &v)) return StatusCode::INVALID_ARG;
            mLevelRequest = v;  // serviced by the imu thread
            LOG(INFO) << (v ? "IMU level capture requested" : "IMU calibration clear requested");
            break;
        }
        case VENDOR_CFG_CAN_CAPTURE: {
            int32_t v;
            if (!intArg(0, 1, &v)) return StatusCode::INVALID_ARG;
            mCaptureEnabled = (v != 0);
            if (v == 0) closeCapture();
            persistConfig("persist.vendor.motodash.cfg.can_capture", std::to_string(v));
            LOG(INFO) << "CAN capture " << (v ? "enabled" : "disabled");
            break;
        }
        case VENDOR_CFG_PACK_ENERGY_WH: {
            float v;
            if (!floatArg(CFG_MIN_PACK_ENERGY_WH, CFG_MAX_PACK_ENERGY_WH, &v)) {
                return StatusCode::INVALID_ARG;
            }
            mPackEnergyWh = v;
            persistConfig("persist.vendor.motodash.cfg.pack_energy_wh", std::to_string(v));
            // Mirror into INFO_EV_BATTERY_CAPACITY so standard consumers agree.
            // The range itself re-projects on the next BMS frame (the model
            // state is owned by the CAN reader thread; don't touch it here).
            {
                std::lock_guard<std::mutex> lock(mValuesMutex);
                auto& cap = mCurrentValues[static_cast<int32_t>(
                        VehicleProperty::INFO_EV_BATTERY_CAPACITY)];
                cap.value.floatValues[0] = v;
                cap.timestamp = elapsedRealtimeNano();
                notifyPropertyChange(
                        static_cast<int32_t>(VehicleProperty::INFO_EV_BATTERY_CAPACITY), cap);
            }
            break;
        }
        case VENDOR_CFG_PACK_MAX_VOLTAGE: {
            float v;
            if (!floatArg(CFG_MIN_PACK_MAX_VOLTAGE, CFG_MAX_PACK_MAX_VOLTAGE, &v)) {
                return StatusCode::INVALID_ARG;
            }
            mPackMaxVoltage = v;
            persistConfig("persist.vendor.motodash.cfg.pack_max_v", std::to_string(v));
            LOG(INFO) << "Pack max voltage set to " << v << " V";
            break;
        }
        case VENDOR_CFG_GPIO_ACTIVE_LOW: {
            int32_t v;
            if (!intArg(0, 1, &v)) return StatusCode::INVALID_ARG;
            mGpioActiveLow = (v != 0);
            mGpioBiasDirty = true;  // GPIO thread re-applies the matching pull
            persistConfig("persist.vendor.motodash.gpio.active_low", std::to_string(v));
            break;
        }
        default: {
            // Display units: validate against the supported VehicleUnit pair
            // and persist; the stored-value/notify tail below does the rest.
            auto unitsArg = [&value](int32_t a, int32_t b, int32_t* out) {
                if (value.value.int32Values.size() != 1) return false;
                int32_t v = value.value.int32Values[0];
                if (v != a && v != b) return false;
                *out = v;
                return true;
            };
            if (propId == static_cast<int32_t>(VehicleProperty::VEHICLE_SPEED_DISPLAY_UNITS)) {
                if (!unitsArg(static_cast<int32_t>(VehicleUnit::KILOMETERS_PER_HOUR),
                              static_cast<int32_t>(VehicleUnit::MILES_PER_HOUR),
                              &mSpeedDisplayUnits)) {
                    return StatusCode::INVALID_ARG;
                }
                persistConfig("persist.vendor.motodash.cfg.units_speed",
                              std::to_string(mSpeedDisplayUnits));
                break;
            }
            if (propId == static_cast<int32_t>(VehicleProperty::DISTANCE_DISPLAY_UNITS)) {
                if (!unitsArg(static_cast<int32_t>(VehicleUnit::KILOMETER),
                              static_cast<int32_t>(VehicleUnit::MILE),
                              &mDistanceDisplayUnits)) {
                    return StatusCode::INVALID_ARG;
                }
                persistConfig("persist.vendor.motodash.cfg.units_distance",
                              std::to_string(mDistanceDisplayUnits));
                break;
            }
            if (propId == VENDOR_CFG_TEMP_DISPLAY_UNITS) {
                if (!unitsArg(static_cast<int32_t>(VehicleUnit::CELSIUS),
                              static_cast<int32_t>(VehicleUnit::FAHRENHEIT),
                              &mTemperatureDisplayUnits)) {
                    return StatusCode::INVALID_ARG;
                }
                persistConfig("persist.vendor.motodash.cfg.units_temp",
                              std::to_string(mTemperatureDisplayUnits));
                break;
            }
            return StatusCode::ACCESS_DENIED;
        }
    }

    LOG(INFO) << "Config property 0x" << std::hex << propId << std::dec << " updated";

    // Store the new value and notify subscribers so settings UIs stay in sync.
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& stored = mCurrentValues[propId];
        stored.prop = propId;
        stored.areaId = 0;
        stored.value = value.value;
        stored.timestamp = elapsedRealtimeNano();
        notifyPropertyChange(propId, stored);
    }
    return StatusCode::OK;
}

void MotorcycleVehicleHardware::persistConfig(const char* name, const std::string& value) {
    if (property_set(name, value.c_str()) != 0) {
        LOG(WARNING) << "Failed to persist config property " << name;
    }
}

StatusCode MotorcycleVehicleHardware::getValues(
        std::shared_ptr<const GetValuesCallback> callback,
        const std::vector<GetValueRequest>& requests) const {
    std::vector<GetValueResult> results;
    
    std::lock_guard<std::mutex> lock(mValuesMutex);
    for (const auto& request : requests) {
        GetValueResult result;
        result.requestId = request.requestId;
        
        auto it = mCurrentValues.find(request.prop.prop);
        if (it != mCurrentValues.end()) {
            result.status = StatusCode::OK;
            result.prop = it->second;
        } else {
            result.status = StatusCode::INVALID_ARG;
        }
        results.push_back(result);
    }
    
    (*callback)(results);
    return StatusCode::OK;
}

DumpResult MotorcycleVehicleHardware::dump(const std::vector<std::string>& /*options*/) {
    DumpResult result;
    result.callerShouldDumpState = true;
    result.buffer = "MotorcycleVehicleHardware\n";
    result.buffer += "  CAN socket: " + std::to_string(mCanSocket) + "\n";
    result.buffer += "  Running: " + std::string(mRunning ? "yes" : "no") + "\n";
    
    std::lock_guard<std::mutex> lock(mValuesMutex);
    result.buffer += "  Current values:\n";
    for (const auto& [propId, value] : mCurrentValues) {
        result.buffer += "    PropId " + std::to_string(propId) + ": ";
        if (!value.value.floatValues.empty()) {
            result.buffer += std::to_string(value.value.floatValues[0]);
        } else if (!value.value.int32Values.empty()) {
            result.buffer += std::to_string(value.value.int32Values[0]);
        }
        result.buffer += "\n";
    }
    
    return result;
}

StatusCode MotorcycleVehicleHardware::checkHealth() {
    return StatusCode::OK;
}

void MotorcycleVehicleHardware::registerOnPropertyChangeEvent(
        std::unique_ptr<const PropertyChangeCallback> callback) {
    LOG(INFO) << "registerOnPropertyChangeEvent called, callback=" << (callback ? "valid" : "null");
    mOnPropertyChangeCallback = std::move(callback);
}

void MotorcycleVehicleHardware::registerOnPropertySetErrorEvent(
        std::unique_ptr<const PropertySetErrorCallback> callback) {
    mOnPropertySetErrorCallback = std::move(callback);
}

StatusCode MotorcycleVehicleHardware::subscribe(SubscribeOptions /*options*/) {
    return StatusCode::OK;
}

StatusCode MotorcycleVehicleHardware::unsubscribe(int32_t /*propId*/, int32_t /*areaId*/) {
    return StatusCode::OK;
}

// ============================================================================
// GPIO Implementation
// ============================================================================

void MotorcycleVehicleHardware::loadConfig() {
    // Read persisted configuration from vendor properties. The persist.vendor.
    // motodash.cfg.* values are written back by this HAL when the settings UI
    // updates the VENDOR_CFG_* properties; the GPIO/product defaults may also
    // come from PRODUCT_VENDOR_PROPERTIES.
    char propValue[PROPERTY_VALUE_MAX];
    
    // Read CAN interface name (default: can1 for Seeed CAN-FD HAT v2)
    if (property_get("persist.vendor.motodash.can_interface", propValue, "can1") > 0) {
        mCanInterface = propValue;
    }
    LOG(INFO) << "CAN interface configured: " << mCanInterface;

    // Speed calculation parameters
    if (property_get("persist.vendor.motodash.cfg.wheel_circumference", propValue, "") > 0) {
        float v = strtof(propValue, nullptr);
        if (v >= CFG_MIN_WHEEL_CIRCUMFERENCE_M && v <= CFG_MAX_WHEEL_CIRCUMFERENCE_M) {
            mWheelCircumference = v;
        } else {
            LOG(WARNING) << "Ignoring out-of-range wheel_circumference: " << propValue;
        }
    }
    if (property_get("persist.vendor.motodash.cfg.gear_base", propValue, "") > 0) {
        int v = atoi(propValue);
        if (v == 0 || v == 1) {
            mGearBase = v;
        } else {
            LOG(WARNING) << "Ignoring out-of-range gear_base: " << propValue;
        }
    }
    if (property_get("persist.vendor.motodash.cfg.gear_ratio", propValue, "") > 0) {
        float v = strtof(propValue, nullptr);
        if (v >= CFG_MIN_GEAR_RATIO && v <= CFG_MAX_GEAR_RATIO) {
            mGearRatio = v;
        } else {
            LOG(WARNING) << "Ignoring out-of-range gear_ratio: " << propValue;
        }
    }

    // CAN IDs (stored as decimal; base 0 also accepts 0x-prefixed hex)
    auto loadCanId = [&propValue](const char* name, std::atomic<uint32_t>* out) {
        if (property_get(name, propValue, "") > 0) {
            uint32_t v = static_cast<uint32_t>(strtoul(propValue, nullptr, 0));
            if (v >= 1 && v <= static_cast<uint32_t>(CFG_MAX_CAN_ID)) {
                *out = v;
            } else {
                LOG(WARNING) << "Ignoring out-of-range CAN ID for " << name << ": " << propValue;
            }
        }
    };
    loadCanId("persist.vendor.motodash.cfg.can_id_controller_status", &mCanIdControllerStatus);
    loadCanId("persist.vendor.motodash.cfg.can_id_controller_temps", &mCanIdControllerTemps);
    loadCanId("persist.vendor.motodash.cfg.can_id_bms", &mCanIdBms);

    // Restore the odometer and trip meter. The odometer must never go
    // backwards across a reboot, so it is persisted (throttled) as we ride.
    if (property_get("persist.vendor.motodash.odometer", propValue, "0") > 0) {
        double v = strtod(propValue, nullptr);
        if (v >= 0.0) mOdometerMeters = v;
    }
    if (property_get("persist.vendor.motodash.trip", propValue, "0") > 0) {
        double v = strtod(propValue, nullptr);
        if (v >= 0.0) mTripMeters = v;
    }
    mPersistedOdometerMeters = mOdometerMeters;
    mPersistedTripMeters = mTripMeters;
    LOG(INFO) << "Restored odometer=" << (mOdometerMeters / 1000.0) << "km trip="
              << (mTripMeters / 1000.0) << "km";

    // Range model: learned consumption and configured pack energy
    if (property_get("persist.vendor.motodash.ride.seq", propValue, "") > 0) {
        int32_t seq = atoi(propValue);
        char m[PROPERTY_VALUE_MAX], s[PROPERTY_VALUE_MAX], w[PROPERTY_VALUE_MAX], x[PROPERTY_VALUE_MAX];
        property_get("persist.vendor.motodash.ride.meters", m, "0");
        property_get("persist.vendor.motodash.ride.seconds", s, "0");
        property_get("persist.vendor.motodash.ride.whperkm", w, "0");
        property_get("persist.vendor.motodash.ride.maxmps", x, "0");
        char ll[PROPERTY_VALUE_MAX], lr[PROPERTY_VALUE_MAX];
        property_get("persist.vendor.motodash.ride.maxleanl", ll, "0");
        property_get("persist.vendor.motodash.ride.maxleanr", lr, "0");
        {
            std::lock_guard<std::mutex> lock(mRideMutex);
            mRideSeq = seq;
        }
        publishRideSummary(atof(m), atof(s), atof(w), atof(x), atof(ll), atof(lr), seq,
                           elapsedRealtimeNano());
        LOG(INFO) << "Restored last ride summary #" << seq;
    }
    if (property_get("persist.vendor.motodash.whperkm", propValue, "") > 0) {
        float v = strtof(propValue, nullptr);
        if (v >= kMinUsableWhPerKm && v <= kMaxChunkWhPerKm) {
            mWhPerKm = v;
            mPersistedWhPerKm = v;
        }
    }
    if (property_get("persist.vendor.motodash.cfg.pack_energy_wh", propValue, "") > 0) {
        float v = strtof(propValue, nullptr);
        if (v >= CFG_MIN_PACK_ENERGY_WH && v <= CFG_MAX_PACK_ENERGY_WH) {
            mPackEnergyWh = v;
        } else {
            LOG(WARNING) << "Ignoring out-of-range pack_energy_wh: " << propValue;
        }
    }
    LOG(INFO) << "Range model: whPerKm=" << mWhPerKm << " packWh=" << mPackEnergyWh.load();
    if (property_get("persist.vendor.motodash.cfg.can_capture", propValue, "0") > 0) {
        mCaptureEnabled = (atoi(propValue) != 0);
        if (mCaptureEnabled) LOG(INFO) << "CAN capture enabled from persisted config";
    }

    // Display units (metric defaults; persisted when the rider changes them)
    auto loadUnit = [&propValue](const char* name, int32_t a, int32_t b, int32_t* out) {
        *out = a;
        if (property_get(name, propValue, "") > 0) {
            int32_t v = atoi(propValue);
            if (v == a || v == b) *out = v;
        }
    };
    loadUnit("persist.vendor.motodash.cfg.units_speed",
             static_cast<int32_t>(VehicleUnit::KILOMETERS_PER_HOUR),
             static_cast<int32_t>(VehicleUnit::MILES_PER_HOUR), &mSpeedDisplayUnits);
    loadUnit("persist.vendor.motodash.cfg.units_distance",
             static_cast<int32_t>(VehicleUnit::KILOMETER),
             static_cast<int32_t>(VehicleUnit::MILE), &mDistanceDisplayUnits);
    loadUnit("persist.vendor.motodash.cfg.units_temp",
             static_cast<int32_t>(VehicleUnit::CELSIUS),
             static_cast<int32_t>(VehicleUnit::FAHRENHEIT), &mTemperatureDisplayUnits);

    LOG(INFO) << "Decode config: wheel=" << mWheelCircumference << "m ratio=" << mGearRatio
              << " canStatus=0x" << std::hex << mCanIdControllerStatus
              << " canTemps=0x" << mCanIdControllerTemps
              << " canBms=0x" << mCanIdBms << std::dec;
    
    if (property_get("persist.vendor.motodash.gpio.left_turn", propValue, "-1") > 0) {
        mGpioLeftTurnPin = atoi(propValue);
    }
    if (property_get("persist.vendor.motodash.gpio.right_turn", propValue, "-1") > 0) {
        mGpioRightTurnPin = atoi(propValue);
    }
    if (property_get("persist.vendor.motodash.gpio.high_beam", propValue, "-1") > 0) {
        mGpioHighBeamPin = atoi(propValue);
    }
    // Accepts "1"/"0" (what the HAL persists) and "true"/"false" (what the
    // product makefile ships). atoi("true") is 0, which silently flipped
    // every indicator to active-high on a fresh userdata.
    mGpioActiveLow = property_get_bool("persist.vendor.motodash.gpio.active_low", true);
    
    mVerboseCanLog = property_get_bool("persist.vendor.motodash.debug.canlog", false);
    if (mVerboseCanLog) {
        LOG(WARNING) << "Verbose CAN logging enabled";
    }

    // Debug indicator source: only honoured on a debuggable build. Lets the
    // simulator exercise the turn signal / high beam path end to end on a
    // kernel with no GPIO controller (Cuttlefish has neither gpio-sim nor
    // gpio-mockup). Real hardware always uses the GPIO lines.
    if (property_get_bool("ro.debuggable", false) &&
        property_get_bool("persist.vendor.motodash.gpio.debug", false)) {
        mGpioDebugSource = true;
        LOG(WARNING) << "GPIO debug source ENABLED - indicators driven by "
                        "vendor.motodash.debug.* properties, not real GPIO";
    }

    if (property_get("persist.vendor.motodash.cfg.pack_max_v", propValue, "") > 0) {
        float v = strtof(propValue, nullptr);
        if (v >= CFG_MIN_PACK_MAX_VOLTAGE && v <= CFG_MAX_PACK_MAX_VOLTAGE) mPackMaxVoltage = v;
    }
    // Inertial sensing: bus/address overrides and the persisted calibration.
    if (property_get("persist.vendor.motodash.imu.i2c", propValue, "") > 0) {
        mImuI2cPath = propValue;
    }
    if (property_get("persist.vendor.motodash.imu.addr", propValue, "") > 0) {
        mImuAddr = static_cast<int>(strtol(propValue, nullptr, 0));
    }
    if (property_get("persist.vendor.motodash.imu.baro_addr", propValue, "") > 0) {
        mBaroAddr = static_cast<int>(strtol(propValue, nullptr, 0));
    }
    loadImuCalibration();

    LOG(INFO) << "GPIO config loaded: left=" << mGpioLeftTurnPin 
              << " right=" << mGpioRightTurnPin 
              << " highbeam=" << mGpioHighBeamPin
              << " activeLow=" << mGpioActiveLow;
}

bool MotorcycleVehicleHardware::openGpioChip() {
    // Open the GPIO character device for Raspberry Pi 5
    // RPi5 uses gpiochip0 for the main GPIO header (BCM pins)
    const char* gpioChips[] = {"/dev/gpiochip0", "/dev/gpiochip4"};
    
    for (const char* chipPath : gpioChips) {
        mGpioChipFd = open(chipPath, O_RDWR);
        if (mGpioChipFd >= 0) {
            LOG(INFO) << "Opened GPIO chip: " << chipPath;
            return true;
        }
        LOG(WARNING) << "Failed to open " << chipPath << ": " << strerror(errno);
    }
    
    LOG(ERROR) << "Failed to open any GPIO chip";
    return false;
}

void MotorcycleVehicleHardware::gpioDebugReaderLoop() {
    LOG(INFO) << "GPIO debug reader running (properties, not hardware)";
    int lastTurn = -1;
    int lastHighBeam = -1;
    while (mRunning) {
        char value[PROPERTY_VALUE_MAX];
        int turn = 0;
        if (property_get("vendor.motodash.debug.turn", value, "0") > 0) {
            turn = atoi(value);
        }
        int highBeam = 0;
        if (property_get("vendor.motodash.debug.high_beam", value, "0") > 0) {
            highBeam = atoi(value);
        }
        if (turn != lastTurn) {
            updateTurnSignalState(turn);
            lastTurn = turn;
        }
        if (highBeam != lastHighBeam) {
            updateHighBeamState(highBeam != 0);
            lastHighBeam = highBeam;
        }
        sleepUnlessStopping(100);
    }
    LOG(INFO) << "GPIO debug reader exiting";
}

void MotorcycleVehicleHardware::gpioReaderThread() {
    if (mGpioDebugSource) {
        gpioDebugReaderLoop();
        return;
    }
    LOG(INFO) << "GPIO reader thread starting with pins: left=" << mGpioLeftTurnPin 
              << " right=" << mGpioRightTurnPin << " highbeam=" << mGpioHighBeamPin;
    
    // Wait for GPIO chip to be available
    const int maxRetries = 30;
    for (int retry = 0; retry < maxRetries && mRunning; retry++) {
        if (openGpioChip()) {
            break;
        }
        LOG(WARNING) << "GPIO chip not ready, retry " << (retry + 1) << "/" << maxRetries;
        if (!sleepUnlessStopping(1000)) break;
    }
    
    if (mGpioChipFd < 0) {
        LOG(ERROR) << "Failed to open GPIO chip after retries";
        return;
    }
    
    // Use GPIO character device ioctl interface
    // Request GPIO lines for input
    // The indicator inputs come through optoisolators, i.e. open-collector
    // outputs: an idle line is floating and needs a pull toward the
    // inactive level, or it reads whatever the SoC's default pull says
    // (BCM 9-27 default to pull-down, which with active-low logic shows
    // every indicator "on"). Pull-up for active-low, pull-down otherwise.
    auto biasFlags = [this]() -> uint64_t {
        return mGpioActiveLow.load() ? GPIO_V2_LINE_FLAG_BIAS_PULL_UP
                                     : GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
    };
    auto requestGpioLine = [this, &biasFlags](int pin) -> int {
        if (pin < 0) return -1;
        
        struct gpio_v2_line_request req;
        memset(&req, 0, sizeof(req));
        req.offsets[0] = pin;
        req.num_lines = 1;
        req.config.flags = GPIO_V2_LINE_FLAG_INPUT | biasFlags();
        strncpy(req.consumer, "motodash", sizeof(req.consumer) - 1);
        
        if (ioctl(mGpioChipFd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
            LOG(ERROR) << "Failed to request GPIO line " << pin << ": " << strerror(errno);
            return -1;
        }
        LOG(INFO) << "Requested GPIO line " << pin << ", fd=" << req.fd;
        return req.fd;
    };
    
    auto readGpioLine = [](int lineFd) -> int {
        if (lineFd < 0) return -1;
        
        struct gpio_v2_line_values vals;
        memset(&vals, 0, sizeof(vals));
        vals.mask = 1;
        
        if (ioctl(lineFd, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals) < 0) {
            return -1;
        }
        return (vals.bits & 1) ? 1 : 0;
    };
    
    // Request lines for configured pins
    int leftFd = requestGpioLine(mGpioLeftTurnPin);
    int rightFd = requestGpioLine(mGpioRightTurnPin);
    int highBeamFd = requestGpioLine(mGpioHighBeamPin);
    
    if (leftFd < 0 && rightFd < 0 && highBeamFd < 0) {
        LOG(ERROR) << "Failed to request any GPIO lines";
        return;
    }
    
    LOG(INFO) << "GPIO reader thread running, polling pins...";
    
    int lastTurnState = -1;
    int lastHighBeamState = -1;
    int lastRawBits = -1;
    auto reconfigure = [&biasFlags](int lineFd) {
        if (lineFd < 0) return;
        struct gpio_v2_line_config cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.flags = GPIO_V2_LINE_FLAG_INPUT | biasFlags();
        if (ioctl(lineFd, GPIO_V2_LINE_SET_CONFIG_IOCTL, &cfg) < 0) {
            LOG(WARNING) << "GPIO bias reconfigure failed: " << strerror(errno);
        }
    };
    
    while (mRunning) {
        if (mGpioBiasDirty.exchange(false)) {
            reconfigure(leftFd);
            reconfigure(rightFd);
            reconfigure(highBeamFd);
            LOG(INFO) << "GPIO bias re-applied for activeLow=" << mGpioActiveLow.load();
        }
        // Read GPIO states
        int leftRaw = readGpioLine(leftFd);
        int rightRaw = readGpioLine(rightFd);
        int highBeamRaw = readGpioLine(highBeamFd);

        // Raw electrical levels for the Workshop, published on change.
        int rawBits = (leftRaw == 1 ? 1 : 0) | (rightRaw == 1 ? 2 : 0) | (highBeamRaw == 1 ? 4 : 0) |
                      (leftFd >= 0 ? 8 : 0) | (rightFd >= 0 ? 16 : 0) | (highBeamFd >= 0 ? 32 : 0);
        if (rawBits != lastRawBits) {
            lastRawBits = rawBits;
            std::lock_guard<std::mutex> lock(mValuesMutex);
            auto& v = mCurrentValues[VENDOR_RAW_GPIO];
            v.value.int32Values[0] = rawBits;
            v.timestamp = elapsedRealtimeNano();
            notifyPropertyChange(VENDOR_RAW_GPIO, v);
        }
        
        // Apply active-low logic if needed
        bool leftActive = (leftRaw >= 0) && (mGpioActiveLow ? (leftRaw == 0) : (leftRaw == 1));
        bool rightActive = (rightRaw >= 0) && (mGpioActiveLow ? (rightRaw == 0) : (rightRaw == 1));
        bool highBeamActive = (highBeamRaw >= 0) && (mGpioActiveLow ? (highBeamRaw == 0) : (highBeamRaw == 1));
        
        // Determine turn signal state
        // VehicleTurnSignal: NONE=0, RIGHT=1, LEFT=2
        int turnState = 0;
        if (leftActive && rightActive) {
            turnState = 2;  // Hazard - show LEFT
        } else if (leftActive) {
            turnState = 2;  // LEFT
        } else if (rightActive) {
            turnState = 1;  // RIGHT
        }
        
        // Update turn signal if changed
        if (turnState != lastTurnState) {
            updateTurnSignalState(turnState);
            lastTurnState = turnState;
            LOG(INFO) << "Turn signal changed: left=" << leftActive << " right=" << rightActive << " state=" << turnState;
        }
        
        // Update high beam if changed
        int highBeamState = highBeamActive ? 1 : 0;
        if (highBeamState != lastHighBeamState) {
            updateHighBeamState(highBeamActive);
            lastHighBeamState = highBeamState;
            LOG(INFO) << "High beam changed: " << highBeamActive;
        }
        
        // Poll at 20Hz (50ms)
        sleepUnlessStopping(50);
    }
    
    // Cleanup
    if (leftFd >= 0) close(leftFd);
    if (rightFd >= 0) close(rightFd);
    if (highBeamFd >= 0) close(highBeamFd);
    
    LOG(INFO) << "GPIO reader thread exiting";
}

void MotorcycleVehicleHardware::updateTurnSignalState(int state) {
    int64_t timestamp = elapsedRealtimeNano();
    
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::TURN_SIGNAL_LIGHT_STATE)];
    if (value.value.int32Values.empty() || value.value.int32Values[0] != state) {
        if (value.value.int32Values.empty()) {
            value.value.int32Values.push_back(state);
        } else {
            value.value.int32Values[0] = state;
        }
        value.prop = static_cast<int32_t>(VehicleProperty::TURN_SIGNAL_LIGHT_STATE);
        value.areaId = 0;
        value.timestamp = timestamp;
        LOG(INFO) << "Notifying TURN_SIGNAL_LIGHT_STATE change: " << state << " propId=" << value.prop;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::TURN_SIGNAL_LIGHT_STATE), value);
    }
}

void MotorcycleVehicleHardware::updateHighBeamState(bool on) {
    int64_t timestamp = elapsedRealtimeNano();
    // VehicleLightState: OFF=0, ON=1, DAYTIME_RUNNING=2
    int state = on ? 1 : 0;
    
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::HIGH_BEAM_LIGHTS_STATE)];
    if (value.value.int32Values.empty() || value.value.int32Values[0] != state) {
        if (value.value.int32Values.empty()) {
            value.value.int32Values.push_back(state);
        } else {
            value.value.int32Values[0] = state;
        }
        value.timestamp = timestamp;
        LOG(INFO) << "Notifying HIGH_BEAM_LIGHTS_STATE change: " << state;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::HIGH_BEAM_LIGHTS_STATE), value);
    }
}

// ---------------------------------------------------------------------------
// Inertial sensing (lean / pitch / g / altitude)
// ---------------------------------------------------------------------------
namespace {
constexpr int64_t kImuPeriodMs = 10;                    // 100 Hz
constexpr int64_t kLeanPublishNs = 100LL * 1000000LL;   // 10 Hz
constexpr int64_t kRawPublishNs = 200LL * 1000000LL;    // 5 Hz (Workshop rows)
constexpr int64_t kBaroPeriodNs = 500LL * 1000000LL;    // 2 Hz
constexpr int64_t kImuProbeNs = 5LL * 1000000000LL;
constexpr float kLeanTrackMinSpeedMps = 2.0f;           // side-stand lean is not a ride statistic

bool parseVec3(const char* text, imu::Vec3* out) {
    float x, y, z;
    if (sscanf(text, "%f,%f,%f", &x, &y, &z) != 3) return false;
    *out = imu::Vec3(x, y, z);
    return out->norm() > 0.5f;
}

std::string vec3ToString(const imu::Vec3& v) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%.5f,%.5f,%.5f", v.x, v.y, v.z);
    return buf;
}
}  // namespace

void MotorcycleVehicleHardware::loadImuCalibration() {
    char propValue[PROPERTY_VALUE_MAX];
    imu::Mounting m;
    imu::Vec3 v;
    if (property_get("persist.vendor.motodash.imu.up", propValue, "") > 0 && parseVec3(propValue, &v)) {
        m.setUp(v);
    }
    if (m.hasUp && property_get("persist.vendor.motodash.imu.fwd", propValue, "") > 0 &&
        parseVec3(propValue, &v)) {
        m.setForward(v);
    }
    mLean.setMounting(m);
    LOG(INFO) << "IMU calibration: up " << (m.hasUp ? "set" : "unset") << ", forward "
              << (m.hasForward ? "set" : "unset");
}

void MotorcycleVehicleHardware::persistImuCalibration() {
    const imu::Mounting& m = mLean.mounting();
    persistConfig("persist.vendor.motodash.imu.up", m.hasUp ? vec3ToString(m.up) : "");
    persistConfig("persist.vendor.motodash.imu.fwd", m.hasForward ? vec3ToString(m.forward) : "");
}

std::unique_ptr<imu::ImuSource> MotorcycleVehicleHardware::openImuSource() {
    // The scenario file wins when present (emulator, e2e test, bench demo).
    if (access(mImuSimPath.c_str(), R_OK) == 0) {
        LOG(WARNING) << "IMU: SIMULATED from " << mImuSimPath;
        if (!mLean.mounting().complete()) {
            // The synthetic sensor sits at identity; a real Level/forward
            // calibration (persisted) still takes precedence when present.
            mLean.setMounting(imu::Mounting::identity());
        }
        return std::make_unique<imu::SimImuSource>(
                mImuSimPath, [this] { return mLastSpeedMps.load(std::memory_order_relaxed); });
    }
    auto src = std::make_unique<imu::I2cImuSource>(mImuI2cPath, static_cast<uint8_t>(mImuAddr),
                                                   static_cast<uint8_t>(mBaroAddr));
    if (!src->open()) return nullptr;
    char id[16];
    snprintf(id, sizeof(id), "0x%02X", src->imuWhoAmI());
    LOG(INFO) << "IMU: ISM330DHCX family (WHO_AM_I " << id << ") on " << mImuI2cPath
              << ", barometer " << (src->hasBaro() ? "present" : "absent");
    return src;
}

void MotorcycleVehicleHardware::imuThread() {
    std::unique_ptr<imu::ImuSource> source;
    int64_t nextProbeNs = 0;
    bool loggedAbsent = false;
    while (mRunning) {
        int64_t now = elapsedRealtimeNano();
        if (!source) {
            if (now >= nextProbeNs) {
                nextProbeNs = now + kImuProbeNs;
                source = openImuSource();
                if (source) {
                    loggedAbsent = false;
                    mLastImuSampleNs = 0;
                    mLean.reset();
                    mImuLevelFailed = false;
                    mImuSourceBits = IMU_STATUS_PRESENT |
                                     (source->hasBaro() ? IMU_STATUS_BARO : 0) |
                                     (source->isSimulated() ? IMU_STATUS_SIMULATED : 0);
                    publishImuStatus(now);
                } else if (!loggedAbsent) {
                    LOG(INFO) << "IMU: nothing on " << mImuI2cPath << " and no " << mImuSimPath
                              << " - lean disabled, probing every 5 s";
                    loggedAbsent = true;
                }
            }
            if (!sleepUnlessStopping(1000)) break;
            continue;
        }
        imu::ImuSample s;
        float tempC = 0.0f;
        if (!source->read(&s, &tempC)) {
            LOG(WARNING) << "IMU: source '" << source->name() << "' lost";
            source.reset();
            mImuSourceBits = 0;
            mImuLevelFailed = false;
            publishImuStatus(now);
            mImuLog.close();
            nextProbeNs = now + 2LL * 1000000000LL;
            continue;
        }
        processImuSample(s, tempC, now);
        if (source->hasBaro() && now - mLastBaroReadNs >= kBaroPeriodNs) {
            mLastBaroReadNs = now;
            float pa = 0.0f, t = 0.0f;
            if (source->readBaro(&pa, &t)) {
                std::lock_guard<std::mutex> lock(mValuesMutex);
                auto setF = [&](int32_t prop, float v) {
                    auto& value = mCurrentValues[prop];
                    value.value.floatValues[0] = v;
                    value.timestamp = now;
                    notifyPropertyChange(prop, value);
                };
                setF(VENDOR_BARO_HPA, pa / 100.0f);
                setF(VENDOR_ALTITUDE_M, imu::Bmp280::altitudeM(pa));
                if (mImuLog.isOpen()) {
                    imu::ImuLogBaro b;
                    b.tS = now / 1e9;
                    b.pressurePa = pa;
                    b.tempC = t;
                    mImuLog.writeBaro(b);
                }
            }
        }
        if (!sleepUnlessStopping(kImuPeriodMs)) break;
    }
}

void MotorcycleVehicleHardware::processImuSample(const imu::ImuSample& s, float tempC,
                                                 int64_t nowNs) {
    int64_t lastFrame = mLastControllerFrameNs.load(std::memory_order_relaxed);
    bool speedValid = lastFrame != 0 && (nowNs - lastFrame) < 1000000000LL;
    float speed = mLastSpeedMps.load(std::memory_order_relaxed);

    // Level / clear commands from the Workshop.
    int req = mLevelRequest.exchange(-1);
    if (req == 1) {
        mImuLevelFailed = false;
        if (speedValid && speed > 0.5f) {
            mImuLevelFailed = true;  // the capture itself would also catch it
            LOG(WARNING) << "IMU level rejected: bike is moving";
        } else {
            mLevelCapture.reset();
            mLevelCapturing = true;
        }
    } else if (req == 0) {
        mLean.setMounting(imu::Mounting{});
        persistImuCalibration();
        mLevelCapturing = false;
        LOG(INFO) << "IMU calibration cleared";
    }
    if (mLevelCapturing) {
        mLevelCapture.add(s);
        if (mLevelCapture.done()) {
            mLevelCapturing = false;
            imu::Vec3 up;
            const char* why = "";
            if (mLevelCapture.result(&up, &why)) {
                imu::Mounting m;  // a fresh level always restarts forward learning
                m.setUp(up);
                mLean.setMounting(m);
                persistImuCalibration();
                LOG(INFO) << "IMU levelled: up = " << vec3ToString(up);
            } else {
                mImuLevelFailed = true;
                LOG(WARNING) << "IMU level rejected: " << why;
            }
        }
    }

    float dt = mLastImuSampleNs == 0 ? 0.0f : static_cast<float>(nowNs - mLastImuSampleNs) / 1e9f;
    mLastImuSampleNs = nowNs;
    if (dt > 0.0f) mLean.update(s, speed, speedValid, dt);
    if (mLean.takeForwardLearned()) {
        persistImuCalibration();
        LOG(INFO) << "IMU forward axis learned: " << vec3ToString(mLean.mounting().forward);
    }

    const auto& st = mLean.state();
    publishImuStatus(nowNs);
    if (st.valid && speedValid) trackRideLean(st.rollDeg, speed);
    captureImuSample(s, speed, speedValid, st, nowNs);

    if (st.valid && nowNs - mLastLeanPublishNs >= kLeanPublishNs) {
        mLastLeanPublishNs = nowNs;
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto setF = [&](int32_t prop, float v) {
            auto& value = mCurrentValues[prop];
            value.value.floatValues[0] = v;
            value.timestamp = nowNs;
            notifyPropertyChange(prop, value);
        };
        setF(VENDOR_LEAN_DEG, st.rollDeg);
        setF(VENDOR_PITCH_DEG, st.pitchDeg);
        setF(VENDOR_LAT_G, st.latG);
        setF(VENDOR_LONG_G, st.longG);
    }
    if (nowNs - mLastRawPublishNs >= kRawPublishNs) {
        mLastRawPublishNs = nowNs;
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& raw = mCurrentValues[VENDOR_IMU_RAW];
        raw.value.floatValues = {s.accelG.x, s.accelG.y, s.accelG.z,
                                 s.gyroDps.x, s.gyroDps.y, s.gyroDps.z};
        raw.timestamp = nowNs;
        notifyPropertyChange(VENDOR_IMU_RAW, raw);
        auto& t = mCurrentValues[VENDOR_IMU_TEMP_C];
        t.value.floatValues[0] = tempC;
        t.timestamp = nowNs;
        notifyPropertyChange(VENDOR_IMU_TEMP_C, t);
    }
}

void MotorcycleVehicleHardware::captureImuSample(const imu::ImuSample& s, float speedMps,
                                                 bool speedValid,
                                                 const imu::LeanEstimator::State& st,
                                                 int64_t nowNs) {
    // Rides with the CAN capture on also get the raw sensor stream, so a
    // real ride can be replayed through the estimator on the host
    // (motorcycle_imu_replay). Same directory, imu-<epoch>.log.
    if (!mCaptureEnabled.load(std::memory_order_relaxed)) {
        if (mImuLog.isOpen()) {
            LOG(INFO) << "IMU capture closed after " << mImuLog.bytes() << " bytes";
            mImuLog.close();
        }
        return;
    }
    if (!mImuLog.isOpen()) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        if (!mImuLog.open(mCaptureDir, static_cast<long>(tv.tv_sec))) {
            if (!mImuLogFailureLogged) {
                LOG(ERROR) << "IMU capture: cannot open " << mImuLog.path() << ": "
                           << strerror(errno);
                mImuLogFailureLogged = true;
            }
            return;
        }
        LOG(INFO) << "IMU capture started: " << mImuLog.path();
    }
    imu::ImuLogRecord r;
    r.tS = nowNs / 1e9;
    r.sample = s;
    r.speedMps = speedMps;
    r.speedValid = speedValid;
    r.rollDeg = st.rollDeg;
    r.pitchDeg = st.pitchDeg;
    r.status = mImuStatus;
    mImuLog.writeSample(r);
    mImuLog.syncIfDue(nowNs);
}

void MotorcycleVehicleHardware::publishImuStatus(int64_t nowNs) {
    // Assembled fresh every time and published only on change.
    const auto& m = mLean.mounting();
    int32_t status = mImuSourceBits;
    if (mImuSourceBits != 0) {
        if (mImuLevelFailed) status |= IMU_STATUS_LEVEL_FAILED;
        if (m.hasUp) status |= IMU_STATUS_LEVEL_SET;
        if (m.hasForward) status |= IMU_STATUS_FORWARD_SET;
        if (mLean.state().valid) status |= IMU_STATUS_VALID;
    }
    if (status == mImuStatus) return;
    mImuStatus = status;
    std::lock_guard<std::mutex> lock(mValuesMutex);
    auto& v = mCurrentValues[VENDOR_IMU_STATUS];
    v.value.int32Values[0] = mImuStatus;
    v.timestamp = nowNs;
    notifyPropertyChange(VENDOR_IMU_STATUS, v);
}

void MotorcycleVehicleHardware::trackRideLean(float rollDeg, float speedMps) {
    if (speedMps < kLeanTrackMinSpeedMps) return;
    std::lock_guard<std::mutex> lock(mRideMutex);
    if (!mRideActive) return;
    if (rollDeg > mRideMaxLeanR) mRideMaxLeanR = rollDeg;
    if (-rollDeg > mRideMaxLeanL) mRideMaxLeanL = -rollDeg;
}

}  // namespace android::hardware::automotive::vehicle::motorcycle
