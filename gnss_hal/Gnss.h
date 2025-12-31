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

#pragma once

#include <aidl/android/hardware/gnss/BnGnss.h>
#include <atomic>
#include <mutex>
#include <thread>

#include "GnssConfiguration.h"
#include "NmeaParser.h"
#include "SerialPort.h"

namespace aidl::android::hardware::gnss::implementation {

class Gnss : public BnGnss {
  public:
    Gnss();
    ~Gnss();

    ndk::ScopedAStatus setCallback(const std::shared_ptr<IGnssCallback>& callback) override;
    ndk::ScopedAStatus close() override;

    ndk::ScopedAStatus getExtensionPsds(std::shared_ptr<IGnssPsds>* iGnssPsds) override;
    ndk::ScopedAStatus getExtensionGnssConfiguration(
            std::shared_ptr<IGnssConfiguration>* iGnssConfiguration) override;
    ndk::ScopedAStatus getExtensionGnssMeasurement(
            std::shared_ptr<IGnssMeasurementInterface>* iGnssMeasurement) override;
    ndk::ScopedAStatus getExtensionGnssPowerIndication(
            std::shared_ptr<IGnssPowerIndication>* iGnssPowerIndication) override;
    ndk::ScopedAStatus getExtensionGnssBatching(
            std::shared_ptr<IGnssBatching>* iGnssBatching) override;
    ndk::ScopedAStatus getExtensionGnssGeofence(
            std::shared_ptr<IGnssGeofence>* iGnssGeofence) override;
    ndk::ScopedAStatus getExtensionGnssNavigationMessage(
            std::shared_ptr<IGnssNavigationMessageInterface>* iGnssNavigationMessage) override;
    ndk::ScopedAStatus getExtensionAGnss(std::shared_ptr<IAGnss>* iAGnss) override;
    ndk::ScopedAStatus getExtensionAGnssRil(std::shared_ptr<IAGnssRil>* iAGnssRil) override;
    ndk::ScopedAStatus getExtensionGnssDebug(std::shared_ptr<IGnssDebug>* iGnssDebug) override;
    ndk::ScopedAStatus getExtensionGnssVisibilityControl(
            std::shared_ptr<visibility_control::IGnssVisibilityControl>* iGnssVisibilityControl)
            override;
    ndk::ScopedAStatus getExtensionGnssAntennaInfo(
            std::shared_ptr<IGnssAntennaInfo>* iGnssAntennaInfo) override;
    ndk::ScopedAStatus getExtensionMeasurementCorrections(
            std::shared_ptr<measurement_corrections::IMeasurementCorrectionsInterface>*
                    iMeasurementCorrections) override;

    ndk::ScopedAStatus start() override;
    ndk::ScopedAStatus stop() override;
    ndk::ScopedAStatus injectTime(int64_t timeMs, int64_t timeReferenceMs,
                                  int uncertaintyMs) override;
    ndk::ScopedAStatus injectLocation(const GnssLocation& location) override;
    ndk::ScopedAStatus injectBestLocation(const GnssLocation& location) override;
    ndk::ScopedAStatus deleteAidingData(GnssAidingData aidingDataFlags) override;
    ndk::ScopedAStatus setPositionMode(const PositionModeOptions& options) override;
    ndk::ScopedAStatus startSvStatus() override;
    ndk::ScopedAStatus stopSvStatus() override;
    ndk::ScopedAStatus startNmea() override;
    ndk::ScopedAStatus stopNmea() override;

  private:
    void threadFunc();
    std::string getGpsDevice();

    std::shared_ptr<IGnssCallback> mCallback;
    std::shared_ptr<GnssConfiguration> mGnssConfiguration;
    std::mutex mMutex;

    std::thread mThread;
    std::atomic<bool> mRunning{false};
    std::atomic<bool> mActive{false};
    std::atomic<int> mIntervalMs{1000};

    SerialPort mSerialPort;
    NmeaParser mNmeaParser;
    
    // Store last valid location for passive provider support
    GnssLocation mLastLocation;
    std::atomic<bool> mHasLastLocation{false};
    
};

}  // namespace aidl::android::hardware::gnss::implementation
