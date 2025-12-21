/*
 * Copyright (C) 2024 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "MotorcycleVehicleService"

#include "MotorcycleVehicleHardware.h"

#include <DefaultVehicleHal.h>
#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

using ::android::hardware::automotive::vehicle::DefaultVehicleHal;
using ::android::hardware::automotive::vehicle::motorcycle::MotorcycleVehicleHardware;

int main(int /* argc */, char* /* argv */[]) {
    LOG(INFO) << "Motorcycle Vehicle HAL service starting...";

    auto hardware = std::make_unique<MotorcycleVehicleHardware>();
    auto vhal = ndk::SharedRefBase::make<DefaultVehicleHal>(std::move(hardware));

    LOG(INFO) << "Registering Vehicle HAL service...";

    const std::string instance = std::string(DefaultVehicleHal::descriptor) + "/default";
    binder_status_t status = AServiceManager_addService(vhal->asBinder().get(), instance.c_str());

    if (status != STATUS_OK) {
        LOG(FATAL) << "Failed to register Vehicle HAL service: " << status;
        return 1;
    }

    LOG(INFO) << "Motorcycle Vehicle HAL service ready";

    ABinderProcess_setThreadPoolMaxThreadCount(4);
    ABinderProcess_startThreadPool();
    ABinderProcess_joinThreadPool();

    LOG(WARNING) << "Vehicle HAL service exiting";
    return 0;
}
