/*
 * Copyright (C) 2024 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "GnssHalRpi5"

#include "Gnss.h"

#include <cutils/properties.h>
#include <inttypes.h>
#include <log/log.h>
#include <utils/Timers.h>

namespace aidl::android::hardware::gnss::implementation {

Gnss::Gnss() {
    ALOGI("Gnss HAL for Raspberry Pi 5 created");
}

Gnss::~Gnss() {
    close();
}

std::string Gnss::getGpsDevice() {
    char value[PROPERTY_VALUE_MAX] = {0};

    // Check vendor property first
    if (property_get("vendor.ser.gnss-uart", value, nullptr) > 0) {
        return std::string(value);
    }

    // Check debug property
    if (property_get("debug.location.gnss.devname", value, nullptr) > 0) {
        return std::string(value);
    }

    // Try common USB GPS device paths
    const char* devices[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"};
    for (const char* dev : devices) {
        if (access(dev, R_OK | W_OK) == 0) {
            ALOGI("Found GPS device: %s", dev);
            return std::string(dev);
        }
    }

    // Default to ttyACM0
    return "/dev/ttyACM0";
}

ndk::ScopedAStatus Gnss::setCallback(const std::shared_ptr<IGnssCallback>& callback) {
    ALOGD("setCallback");

    {
        std::lock_guard<std::mutex> lock(mMutex);
        mCallback = callback;
    }

    if (callback == nullptr) {
        ALOGE("Null callback");
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    // Report capabilities
    int capabilities = IGnssCallback::CAPABILITY_SCHEDULING |
                       IGnssCallback::CAPABILITY_SATELLITE_BLOCKLIST;
    auto status = callback->gnssSetCapabilitiesCb(capabilities);
    if (!status.isOk()) {
        ALOGE("Failed to set capabilities");
    }

    // Report system info
    IGnssCallback::GnssSystemInfo systemInfo = {
            .yearOfHw = 2024,
            .name = "U-blox 7 GPS/GLONASS",
    };
    status = callback->gnssSetSystemInfoCb(systemInfo);
    if (!status.isOk()) {
        ALOGE("Failed to set system info");
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::close() {
    ALOGD("close");

    mRunning = false;
    mActive = false;

    if (mThread.joinable()) {
        mThread.join();
    }

    mSerialPort.close();

    {
        std::lock_guard<std::mutex> lock(mMutex);
        mCallback = nullptr;
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::start() {
    ALOGD("start");

    if (mActive) {
        ALOGW("Already active");
        return ndk::ScopedAStatus::ok();
    }

    std::string device = getGpsDevice();
    ALOGI("Opening GPS device: %s", device.c_str());

    if (!mSerialPort.open(device, 9600)) {
        ALOGE("Failed to open GPS device: %s", device.c_str());
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_UNKNOWN_ERROR);
    }

    mActive = true;
    mRunning = true;

    // Report session begin
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mCallback) {
            mCallback->gnssStatusCb(IGnssCallback::GnssStatusValue::SESSION_BEGIN);
        }
    }

    // Start reading thread
    mThread = std::thread(&Gnss::threadFunc, this);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::stop() {
    ALOGD("stop");

    mActive = false;

    // Report session end
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mCallback) {
            mCallback->gnssStatusCb(IGnssCallback::GnssStatusValue::SESSION_END);
        }
    }

    return ndk::ScopedAStatus::ok();
}

void Gnss::threadFunc() {
    ALOGI("GPS thread started");

    char buffer[256];
    int64_t lastReportTime = 0;

    while (mRunning) {
        if (!mActive) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        int len = mSerialPort.readLine(buffer, sizeof(buffer), 100);
        if (len > 0) {
            std::string sentence(buffer);

            // Log NMEA sentences for debugging (only RMC and GGA)
            if (sentence.find("RMC") != std::string::npos || 
                sentence.find("GGA") != std::string::npos) {
                ALOGD("NMEA: %s", sentence.c_str());
            }

            // Parse NMEA
            if (mNmeaParser.parse(sentence)) {
                int64_t now = systemTime(SYSTEM_TIME_MONOTONIC) / 1000000LL;

                // Report location at configured interval
                if (mNmeaParser.hasValidFix() && (now - lastReportTime >= mIntervalMs)) {
                    GnssLocation location = mNmeaParser.getLocation();

                    std::lock_guard<std::mutex> lock(mMutex);
                    if (mCallback) {
                        auto status = mCallback->gnssLocationCb(location);
                        if (status.isOk()) {
                            ALOGD("Reported location: %.6f, %.6f",
                                  location.latitudeDegrees, location.longitudeDegrees);
                            lastReportTime = now;
                        }
                    }
                }
            }

            // Report NMEA to callback
            {
                std::lock_guard<std::mutex> lock(mMutex);
                if (mCallback) {
                    int64_t timestamp = systemTime(SYSTEM_TIME_MONOTONIC);
                    mCallback->gnssNmeaCb(timestamp, sentence);
                }
            }
        }
    }

    ALOGI("GPS thread stopped");
}

ndk::ScopedAStatus Gnss::injectTime(int64_t timeMs, int64_t /*timeReferenceMs*/,
                                    int /*uncertaintyMs*/) {
    ALOGD("injectTime: %" PRId64, timeMs);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::injectLocation(const GnssLocation& location) {
    ALOGD("injectLocation: %.6f, %.6f", location.latitudeDegrees, location.longitudeDegrees);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::injectBestLocation(const GnssLocation& /*location*/) {
    ALOGD("injectBestLocation");
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::deleteAidingData(GnssAidingData aidingDataFlags) {
    ALOGD("deleteAidingData: %d", static_cast<int>(aidingDataFlags));
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::setPositionMode(const PositionModeOptions& options) {
    ALOGD("setPositionMode: interval=%d", options.minIntervalMs);
    mIntervalMs = std::max(100, options.minIntervalMs);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::startSvStatus() {
    ALOGD("startSvStatus");
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::stopSvStatus() {
    ALOGD("stopSvStatus");
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::startNmea() {
    ALOGD("startNmea");
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::stopNmea() {
    ALOGD("stopNmea");
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::getExtensionPsds(std::shared_ptr<IGnssPsds>* /*iGnssPsds*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionGnssConfiguration(
        std::shared_ptr<IGnssConfiguration>* iGnssConfiguration) {
    if (mGnssConfiguration == nullptr) {
        mGnssConfiguration = ndk::SharedRefBase::make<GnssConfiguration>();
    }
    *iGnssConfiguration = mGnssConfiguration;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::getExtensionGnssMeasurement(
        std::shared_ptr<IGnssMeasurementInterface>* /*iGnssMeasurement*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionGnssPowerIndication(
        std::shared_ptr<IGnssPowerIndication>* /*iGnssPowerIndication*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionGnssBatching(
        std::shared_ptr<IGnssBatching>* /*iGnssBatching*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionGnssGeofence(
        std::shared_ptr<IGnssGeofence>* /*iGnssGeofence*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionGnssNavigationMessage(
        std::shared_ptr<IGnssNavigationMessageInterface>* /*iGnssNavigationMessage*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionAGnss(std::shared_ptr<IAGnss>* /*iAGnss*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionAGnssRil(std::shared_ptr<IAGnssRil>* /*iAGnssRil*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionGnssDebug(std::shared_ptr<IGnssDebug>* /*iGnssDebug*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionGnssVisibilityControl(
        std::shared_ptr<visibility_control::IGnssVisibilityControl>* /*iGnssVisibilityControl*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionGnssAntennaInfo(
        std::shared_ptr<IGnssAntennaInfo>* /*iGnssAntennaInfo*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Gnss::getExtensionMeasurementCorrections(
        std::shared_ptr<measurement_corrections::IMeasurementCorrectionsInterface>*
        /*iMeasurementCorrections*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}


}  // namespace aidl::android::hardware::gnss::implementation
