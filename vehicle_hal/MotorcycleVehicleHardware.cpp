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
#include <cstring>
#include <chrono>
#include <thread>

namespace android::hardware::automotive::vehicle::motorcycle {

using ::aidl::android::hardware::automotive::vehicle::VehicleGear;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyType;
using ::aidl::android::hardware::automotive::vehicle::VehicleUnit;

MotorcycleVehicleHardware::MotorcycleVehicleHardware() {
    LOG(INFO) << "MotorcycleVehicleHardware initializing...";
    initPropertyConfigs();
    
    // Start CAN initialization in a separate thread to allow retries
    // CAN interfaces may not be available immediately at boot
    mRunning = true;
    mCanReaderThread = std::thread(&MotorcycleVehicleHardware::canReaderThread, this);
    LOG(INFO) << "CAN reader thread started (will retry connection)";
}

MotorcycleVehicleHardware::~MotorcycleVehicleHardware() {
    mRunning = false;
    if (mCanReaderThread.joinable()) {
        mCanReaderThread.join();
    }
    if (mCanSocket >= 0) {
        close(mCanSocket);
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
    // Byte 0-1: Ah remaining (0.1Ah resolution)
    // int ahRaw = data[0] | (data[1] << 8);
    
    // Byte 2: Battery temperature (°C)
    // int batteryTemp = data[2];
    
    // Byte 3: State of Charge (%)
    int soc = data[3];

    int64_t timestamp = elapsedRealtimeNano();

    // Update Battery SoC
    {
        std::lock_guard<std::mutex> lock(mValuesMutex);
        auto& value = mCurrentValues[static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL)];
        value.value.floatValues[0] = static_cast<float>(soc);
        value.timestamp = timestamp;
        notifyPropertyChange(static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL), value);
    }
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
        (*mOnPropertyChangeCallback)(values);
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

}  // namespace android::hardware::automotive::vehicle::motorcycle
