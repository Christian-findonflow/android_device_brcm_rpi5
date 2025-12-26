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

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <log/log.h>

#include "Gnss.h"

using aidl::android::hardware::gnss::implementation::Gnss;

int main() {
    ALOGI("GNSS HAL for Raspberry Pi 5 starting");

    ABinderProcess_setThreadPoolMaxThreadCount(1);

    std::shared_ptr<Gnss> gnss = ndk::SharedRefBase::make<Gnss>();

    const std::string instance = std::string() + Gnss::descriptor + "/default";
    binder_status_t status =
            AServiceManager_addService(gnss->asBinder().get(), instance.c_str());

    if (status != STATUS_OK) {
        ALOGE("Failed to register GNSS HAL service: %d", status);
        return 1;
    }

    ALOGI("GNSS HAL service registered: %s", instance.c_str());

    ABinderProcess_joinThreadPool();

    return 0;
}
