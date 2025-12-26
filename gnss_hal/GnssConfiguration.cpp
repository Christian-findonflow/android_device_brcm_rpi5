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

#include "GnssConfiguration.h"
#include <log/log.h>

namespace aidl::android::hardware::gnss::implementation {

ndk::ScopedAStatus GnssConfiguration::setSuplVersion(int version) {
    ALOGD("setSuplVersion: %d", version);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus GnssConfiguration::setSuplMode(int mode) {
    ALOGD("setSuplMode: %d", mode);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus GnssConfiguration::setLppProfile(int lppProfile) {
    ALOGD("setLppProfile: %d", lppProfile);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus GnssConfiguration::setGlonassPositioningProtocol(int protocol) {
    ALOGD("setGlonassPositioningProtocol: %d", protocol);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus GnssConfiguration::setEmergencySuplPdn(bool enable) {
    ALOGD("setEmergencySuplPdn: %d", enable);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus GnssConfiguration::setEsExtensionSec(int emergencyExtensionSeconds) {
    ALOGD("setEsExtensionSec: %d", emergencyExtensionSeconds);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus GnssConfiguration::setBlocklist(
        const std::vector<BlocklistedSource>& blocklist) {
    ALOGD("setBlocklist: %zu entries", blocklist.size());
    return ndk::ScopedAStatus::ok();
}

}  // namespace aidl::android::hardware::gnss::implementation
