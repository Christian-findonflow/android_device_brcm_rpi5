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

MotorcycleVehicleHardware::MotorcycleVehicleHardware() {
    LOG(INFO) << "MotorcycleVehicleHardware initializing...";
    
    // Load GPIO configuration from system properties
    loadGpioConfig();
    
    initPropertyConfigs();
    
    // Start CAN initialization in a separate thread to allow retries
    // CAN interfaces may not be available immediately at boot
    mRunning = true;
    mCanReaderThread = std::thread(&MotorcycleVehicleHardware::canReaderThread, this);
    LOG(INFO) << "CAN reader thread started (will retry connection)";
    
    // Start GPIO reader thread if any GPIO pins are configured
    if (mGpioLeftTurnPin >= 0 || mGpioRightTurnPin >= 0 || mGpioHighBeamPin >= 0) {
        mGpioReaderThread = std::thread(&MotorcycleVehicleHardware::gpioReaderThread, this);
        LOG(INFO) << "GPIO reader thread started";
    }
    
    // Start BMS polling thread for OBD2 queries
    mBmsPollingThread = std::thread(&MotorcycleVehicleHardware::bmsPollingThread, this);
    LOG(INFO) << "BMS polling thread started";
}

MotorcycleVehicleHardware::~MotorcycleVehicleHardware() {
    mRunning = false;
    mObd2ResponseCv.notify_all();  // Wake up BMS polling thread
    if (mCanReaderThread.joinable()) {
        mCanReaderThread.join();
    }
    if (mBmsPollingThread.joinable()) {
        mBmsPollingThread.join();
    }
    if (mGpioReaderThread.joinable()) {
        mGpioReaderThread.join();
    }
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
        value.value.floatValues.push_back(7200.0f);  // 72V * 100Ah = 7200Wh
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
        value.value.floatValues.push_back(0.0f);
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

    LOG(INFO) << "Initialized " << mPropertyConfigs.size() << " property configs";
}

bool MotorcycleVehicleHardware::openCanSocket() {
    mCanSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mCanSocket < 0) {
        LOG(ERROR) << "Failed to create CAN socket: " << strerror(errno);
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

    if (ioctl(mCanSocket, SIOCGIFINDEX, &ifr) < 0) {
        LOG(ERROR) << "Failed to get interface index for can0: " << strerror(errno);
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

    LOG(INFO) << "CAN socket opened on can0 (ifindex=" << ifr.ifr_ifindex << ")";
    return true;
}

void MotorcycleVehicleHardware::canReaderThread() {
    LOG(INFO) << "CAN reader thread starting, will attempt to open CAN socket...";
    
    // Retry opening CAN socket - interfaces may not be ready at boot
    const int maxRetries = 30;  // 30 seconds max wait
    for (int retry = 0; retry < maxRetries && mRunning; retry++) {
        if (openCanSocket()) {
            LOG(INFO) << "CAN socket opened successfully after " << retry << " retries";
            break;
        }
        LOG(WARNING) << "CAN socket not ready, retry " << (retry + 1) << "/" << maxRetries;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    if (mCanSocket < 0) {
        LOG(ERROR) << "Failed to open CAN socket after retries, running in simulation mode";
        return;
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
            LOG(ERROR) << "CAN read error: " << strerror(errno);
            break;
        }

        if (nbytes == sizeof(frame)) {
            frameCount++;
            if (frameCount % 100 == 1) {
                LOG(INFO) << "Received " << frameCount << " CAN frames, last ID=0x" 
                          << std::hex << (frame.can_id & CAN_EFF_MASK) << std::dec
                          << " extended=" << ((frame.can_id & CAN_EFF_FLAG) != 0);
            }
            processCanFrame(frame);
        }
    }

    LOG(INFO) << "CAN reader thread exiting, total frames=" << frameCount;
}

void MotorcycleVehicleHardware::processCanFrame(const struct can_frame& frame) {
    uint32_t canId = frame.can_id & CAN_EFF_MASK;
    bool isExtended = (frame.can_id & CAN_EFF_FLAG) != 0;

    if (isExtended) {
        if (canId == CAN_ID_CONTROLLER_STATUS) {
            processControllerStatus(frame.data);
        } else if (canId == CAN_ID_CONTROLLER_TEMPS) {
            processControllerTemps(frame.data);
        }
    } else {
        if (canId == CAN_ID_BMS) {
            processBmsData(frame.data);
        } else if (canId == CAN_ID_OBD2_RESPONSE) {
            // OBD2 response from BMS
            processObd2Response(frame.data, frame.can_dlc);
        }
    }
}

void MotorcycleVehicleHardware::processControllerStatus(const uint8_t* data) {
    // Byte 0-1: RPM (little-endian)
    int rpm = data[0] | (data[1] << 8);
    
    // Byte 2: Gear (0=P, 1=R, 2=N, 3=D)
    int gear = data[2];
    
    // Byte 3-4: Battery voltage (0.1V resolution)
    int voltageRaw = data[3] | (data[4] << 8);
    float voltage = voltageRaw * 0.1f;
    
    // Byte 5-6: Battery current (0.1A resolution, offset by 320A)
    int currentRaw = data[5] | (data[6] << 8);
    float current = (currentRaw - 3200) * 0.1f;
    
    // Byte 7: Error codes
    // int errors = data[7];

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

    // Update Gear
    {
        int vehicleGear = mapGearToVehicleGear(gear);
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& gearValue = mCurrentValues[static_cast<int32_t>(VehicleProperty::CURRENT_GEAR)];
        if (gearValue.value.int32Values[0] != vehicleGear) {
            gearValue.value.int32Values[0] = vehicleGear;
            gearValue.timestamp = timestamp;
            notifyPropertyChange(static_cast<int32_t>(VehicleProperty::CURRENT_GEAR), gearValue);
            
            auto& selValue = mCurrentValues[static_cast<int32_t>(VehicleProperty::GEAR_SELECTION)];
            selValue.value.int32Values[0] = vehicleGear;
            selValue.timestamp = timestamp;
            notifyPropertyChange(static_cast<int32_t>(VehicleProperty::GEAR_SELECTION), selValue);
        }
    }

    // Update Battery power (V * A = Watts)
    {
        float powerWatts = voltage * current;
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::EV_BATTERY_INSTANTANEOUS_CHARGE_RATE)];
        value.value.floatValues[0] = powerWatts;
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

    // Update Current (vendor property)
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[VENDOR_BATTERY_CURRENT];
        value.value.floatValues[0] = current;
        value.timestamp = timestamp;
        notifyPropertyChange(VENDOR_BATTERY_CURRENT, value);
    }
}

void MotorcycleVehicleHardware::processControllerTemps(const uint8_t* data) {
    // Byte 0: Controller temperature (°C)
    int controllerTemp = data[0];
    
    // Byte 1: Motor temperature (°C)
    int motorTemp = data[1];
    
    // Byte 2: Throttle (%)
    int throttle = data[2];

    int64_t timestamp = elapsedRealtimeNano();

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
    // BMS broadcast message 0x6B1 (from Neo CAN Matrix)
    // Byte 0: Amphours remaining
    int amphours = data[0];
    
    // Byte 1: Battery temperature (°C)
    int batteryTemp = data[1];
    
    // Byte 2: State of Charge (%)
    int soc = data[2];

    int64_t timestamp = elapsedRealtimeNano();

    // Update Battery SoC
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL)];
        value.value.floatValues[0] = static_cast<float>(soc);
        value.timestamp = timestamp;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL), value);
    }

    // Update Pack Amphours
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[VENDOR_PACK_AMPHOURS];
        value.value.floatValues[0] = static_cast<float>(amphours);
        value.timestamp = timestamp;
        notifyPropertyChange(VENDOR_PACK_AMPHOURS, value);
    }

    // Update Pack Temperature (use as average temp from BMS broadcast)
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[VENDOR_PACK_TEMP_AVG];
        value.value.floatValues[0] = static_cast<float>(batteryTemp);
        value.timestamp = timestamp;
        notifyPropertyChange(VENDOR_PACK_TEMP_AVG, value);
    }

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

    return true;
}

void MotorcycleVehicleHardware::processObd2Response(const uint8_t* data, uint8_t len) {
    if (len < 4) return;

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
        case BMS_PID_PACK_SOH: {
            // 1 byte, direct percentage
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_SOH, static_cast<float>(data[4]));
            }
            break;
        }
        case BMS_PID_PACK_CYCLES: {
            // 2 bytes, direct count
            if (dataLen >= 2) {
                int cycles = data[4] | (data[5] << 8);
                updateBmsPropertyInt(VENDOR_PACK_CYCLES, cycles);
            }
            break;
        }
        case BMS_PID_TEMP_HIGH: {
            // 1 byte, °C with -40 offset
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_TEMP_HIGH, static_cast<float>(data[4]) - 40.0f);
            }
            break;
        }
        case BMS_PID_TEMP_LOW: {
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_TEMP_LOW, static_cast<float>(data[4]) - 40.0f);
            }
            break;
        }
        case BMS_PID_TEMP_AVG: {
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_PACK_TEMP_AVG, static_cast<float>(data[4]) - 40.0f);
            }
            break;
        }
        case BMS_PID_HEATSINK_TEMP: {
            if (dataLen >= 1) {
                updateBmsProperty(VENDOR_HEATSINK_TEMP, static_cast<float>(data[4]) - 40.0f);
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
                int raw = data[4] | (data[5] << 8);
                updateBmsProperty(VENDOR_CELL_VOLTAGE_LOW, raw * 0.0001f);
            }
            break;
        }
        case BMS_PID_CELL_HIGH: {
            if (dataLen >= 2) {
                int raw = data[4] | (data[5] << 8);
                updateBmsProperty(VENDOR_CELL_VOLTAGE_HIGH, raw * 0.0001f);
            }
            break;
        }
        case BMS_PID_CELL_AVG: {
            if (dataLen >= 2) {
                int raw = data[4] | (data[5] << 8);
                updateBmsProperty(VENDOR_CELL_VOLTAGE_AVG, raw * 0.0001f);
            }
            break;
        }
        case BMS_PID_CELL_LOW_ID: {
            if (dataLen >= 2) {
                int cellId = data[4] | (data[5] << 8);
                updateBmsPropertyInt(VENDOR_CELL_LOW_ID, cellId);
            }
            break;
        }
        case BMS_PID_CELL_HIGH_ID: {
            if (dataLen >= 2) {
                int cellId = data[4] | (data[5] << 8);
                updateBmsPropertyInt(VENDOR_CELL_HIGH_ID, cellId);
            }
            break;
        }
        case BMS_PID_CHARGE_LIMIT: {
            // 2 bytes, direct amps
            if (dataLen >= 2) {
                int amps = data[4] | (data[5] << 8);
                updateBmsProperty(VENDOR_CHARGE_LIMIT, static_cast<float>(amps));
            }
            break;
        }
        case BMS_PID_DISCHARGE_LIMIT: {
            if (dataLen >= 2) {
                int amps = data[4] | (data[5] << 8);
                updateBmsProperty(VENDOR_DISCHARGE_LIMIT, static_cast<float>(amps));
            }
            break;
        }
        case BMS_PID_PACK_AMPHOURS: {
            // 2 bytes, 0.1Ah resolution
            if (dataLen >= 2) {
                int raw = data[4] | (data[5] << 8);
                updateBmsProperty(VENDOR_PACK_AMPHOURS, raw * 0.1f);
            }
            break;
        }
        case BMS_PID_PACK_RESISTANCE: {
            // 2 bytes, 0.01mOhm resolution
            if (dataLen >= 2) {
                int raw = data[4] | (data[5] << 8);
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
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    if (!mRunning) return;
    
    LOG(INFO) << "BMS polling thread active, starting OBD2 queries";
    
    // List of PIDs to poll (slower-changing data)
    const std::vector<uint16_t> pollPids = {
        BMS_PID_PACK_SOH,
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
    
    size_t pidIndex = 0;
    
    while (mRunning) {
        // Send request for current PID
        uint16_t pid = pollPids[pidIndex];
        
        if (sendObd2Request(pid)) {
            // Wait for response (with timeout)
            std::unique_lock<std::mutex> lock(mObd2Mutex);
            mObd2ResponseCv.wait_for(lock, std::chrono::milliseconds(100),
                [this] { return mObd2ResponseReceived || !mRunning; });
        }
        
        // Move to next PID
        pidIndex = (pidIndex + 1) % pollPids.size();
        
        // Delay between requests (100ms between each, full cycle ~1.6s)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    LOG(INFO) << "BMS polling thread exiting";
}

float MotorcycleVehicleHardware::calculateSpeedFromRpm(int rpm) const {
    // Speed (m/s) = RPM * wheel_circumference / (gear_ratio * 60)
    return static_cast<float>(rpm) * RPM_TO_MPS;
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

void MotorcycleVehicleHardware::notifyPropertyChange(int32_t propId, const VehiclePropValue& value) {
    static int speedNotifyCount = 0;
    if (mOnPropertyChangeCallback) {
        std::vector<VehiclePropValue> values = {value};
        // Log PERF_VEHICLE_SPEED specifically since app subscribes to it
        if (propId == 291504647) {  // PERF_VEHICLE_SPEED
            if (++speedNotifyCount % 30 == 1) {
                LOG(INFO) << "SPEED notification #" << speedNotifyCount << " value=" 
                          << (value.value.floatValues.empty() ? 0 : value.value.floatValues[0]);
            }
        }
        // Log turn signal and high beam notifications
        if (propId == 289408560 || propId == 289410562) {  // TURN_SIGNAL_STATE or HIGH_BEAM_LIGHTS_STATE
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
    // Most properties are read-only from CAN
    std::vector<SetValueResult> results;
    for (const auto& request : requests) {
        SetValueResult result;
        result.requestId = request.requestId;
        result.status = StatusCode::ACCESS_DENIED;
        results.push_back(result);
    }
    (*callback)(results);
    return StatusCode::OK;
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

void MotorcycleVehicleHardware::loadGpioConfig() {
    // Read GPIO configuration from vendor properties
    // These are set by the MotoDash app settings
    char propValue[PROPERTY_VALUE_MAX];
    
    if (property_get("persist.vendor.motodash.gpio.left_turn", propValue, "-1") > 0) {
        mGpioLeftTurnPin = atoi(propValue);
    }
    if (property_get("persist.vendor.motodash.gpio.right_turn", propValue, "-1") > 0) {
        mGpioRightTurnPin = atoi(propValue);
    }
    if (property_get("persist.vendor.motodash.gpio.high_beam", propValue, "-1") > 0) {
        mGpioHighBeamPin = atoi(propValue);
    }
    if (property_get("persist.vendor.motodash.gpio.active_low", propValue, "1") > 0) {
        mGpioActiveLow = (atoi(propValue) != 0);
    }
    
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

void MotorcycleVehicleHardware::gpioReaderThread() {
    LOG(INFO) << "GPIO reader thread starting with pins: left=" << mGpioLeftTurnPin 
              << " right=" << mGpioRightTurnPin << " highbeam=" << mGpioHighBeamPin;
    
    // Wait for GPIO chip to be available
    const int maxRetries = 30;
    for (int retry = 0; retry < maxRetries && mRunning; retry++) {
        if (openGpioChip()) {
            break;
        }
        LOG(WARNING) << "GPIO chip not ready, retry " << (retry + 1) << "/" << maxRetries;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    if (mGpioChipFd < 0) {
        LOG(ERROR) << "Failed to open GPIO chip after retries";
        return;
    }
    
    // Use GPIO character device ioctl interface
    // Request GPIO lines for input
    auto requestGpioLine = [this](int pin) -> int {
        if (pin < 0) return -1;
        
        struct gpio_v2_line_request req;
        memset(&req, 0, sizeof(req));
        req.offsets[0] = pin;
        req.num_lines = 1;
        req.config.flags = GPIO_V2_LINE_FLAG_INPUT;
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
    
    while (mRunning) {
        // Read GPIO states
        int leftRaw = readGpioLine(leftFd);
        int rightRaw = readGpioLine(rightFd);
        int highBeamRaw = readGpioLine(highBeamFd);
        
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
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

}  // namespace android::hardware::automotive::vehicle::motorcycle
