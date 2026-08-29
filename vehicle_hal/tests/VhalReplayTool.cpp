/*
 * Copyright (C) 2024 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host replay tool: runs the real motorcycle Vehicle HAL decode stack on the
 * development machine against a local (virtual) CAN interface, printing every
 * property update it would deliver to Android. See TESTING.md for the vcan
 * setup and canplayer/cansend recipes.
 */

#include "MotorcycleVehicleHardware.h"

#include <csignal>
#include <cstdio>
#include <string>
#include <unordered_map>

using ::aidl::android::hardware::automotive::vehicle::VehiclePropValue;
using ::aidl::android::hardware::automotive::vehicle::VehicleProperty;
using ::android::hardware::automotive::vehicle::IVehicleHardware;
using ::android::hardware::automotive::vehicle::motorcycle::MotorcycleVehicleHardware;

namespace vmc = ::android::hardware::automotive::vehicle::motorcycle;

namespace {

volatile std::sig_atomic_t gStop = 0;

void onSignal(int) {
    gStop = 1;
}

const char* propName(int32_t propId) {
    static const std::unordered_map<int32_t, const char*> kNames = {
            {static_cast<int32_t>(VehicleProperty::PERF_VEHICLE_SPEED), "PERF_VEHICLE_SPEED (m/s)"},
            {static_cast<int32_t>(VehicleProperty::ENGINE_RPM), "ENGINE_RPM"},
            {static_cast<int32_t>(VehicleProperty::CURRENT_GEAR), "CURRENT_GEAR"},
            {static_cast<int32_t>(VehicleProperty::GEAR_SELECTION), "GEAR_SELECTION"},
            {static_cast<int32_t>(VehicleProperty::EV_BATTERY_LEVEL), "EV_BATTERY_LEVEL (%)"},
            {static_cast<int32_t>(VehicleProperty::EV_BATTERY_INSTANTANEOUS_CHARGE_RATE),
             "EV_CHARGE_RATE (W)"},
            {static_cast<int32_t>(VehicleProperty::ENGINE_COOLANT_TEMP), "CONTROLLER_TEMP (C)"},
            {static_cast<int32_t>(VehicleProperty::ENGINE_OIL_TEMP), "MOTOR_TEMP (C)"},
            {static_cast<int32_t>(VehicleProperty::TURN_SIGNAL_LIGHT_STATE), "TURN_SIGNAL"},
            {static_cast<int32_t>(VehicleProperty::HIGH_BEAM_LIGHTS_STATE), "HIGH_BEAM"},
            {vmc::VENDOR_BATTERY_VOLTAGE, "VENDOR_BATTERY_VOLTAGE (V)"},
            {vmc::VENDOR_BATTERY_CURRENT, "VENDOR_BATTERY_CURRENT (A)"},
            {vmc::VENDOR_THROTTLE_POSITION, "VENDOR_THROTTLE (%)"},
            {vmc::VENDOR_PACK_SOH, "PACK_SOH (%)"},
            {vmc::VENDOR_PACK_TEMP_AVG, "PACK_TEMP_AVG (C)"},
            {vmc::VENDOR_PACK_TEMP_HIGH, "PACK_TEMP_HIGH (C)"},
            {vmc::VENDOR_PACK_TEMP_LOW, "PACK_TEMP_LOW (C)"},
            {vmc::VENDOR_CELL_VOLTAGE_LOW, "CELL_V_LOW (V)"},
            {vmc::VENDOR_CELL_VOLTAGE_HIGH, "CELL_V_HIGH (V)"},
            {vmc::VENDOR_CELL_VOLTAGE_AVG, "CELL_V_AVG (V)"},
            {vmc::VENDOR_PACK_CYCLES, "PACK_CYCLES"},
            {vmc::VENDOR_PACK_AMPHOURS, "PACK_AMPHOURS (Ah)"},
            {vmc::VENDOR_CHARGE_LIMIT, "CHARGE_LIMIT (A)"},
            {vmc::VENDOR_DISCHARGE_LIMIT, "DISCHARGE_LIMIT (A)"},
            {vmc::VENDOR_CFG_WHEEL_CIRCUMFERENCE, "CFG_WHEEL_CIRCUMFERENCE (m)"},
            {vmc::VENDOR_CFG_GEAR_RATIO, "CFG_GEAR_RATIO"},
    };
    auto it = kNames.find(propId);
    return it != kNames.end() ? it->second : nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    std::string iface = argc > 1 ? argv[1] : "vcan0";

    std::printf("Motorcycle VHAL replay tool\n");
    std::printf("Binding HAL decode stack to CAN interface: %s\n\n", iface.c_str());
    std::printf("Example frames (using can-utils):\n");
    std::printf("  # Controller: gear D, 3000 rpm, 72.0 V, 25.5 A\n");
    std::printf("  cansend %s 10261022#00.30.B8.0B.D0.02.FF.00\n", iface.c_str());
    std::printf("  # Controller temps: 45 C controller, 60 C motor, 55%% throttle\n");
    std::printf("  cansend %s 10261023#2D.3C.00.00.37.00.00.00\n", iface.c_str());
    std::printf("  # BMS broadcast: SOC 18%%, pack temp 11 C\n");
    std::printf("  cansend %s 6B1#00.63.00.12.03.02.00.33\n", iface.c_str());
    std::printf("  # Replay a real capture:\n");
    std::printf("  canplayer -I ride.candump %s=can1\n\n", iface.c_str());

    MotorcycleVehicleHardware hardware(iface);

    hardware.registerOnPropertyChangeEvent(
            std::make_unique<const IVehicleHardware::PropertyChangeCallback>(
                    [](std::vector<VehiclePropValue> values) {
                        for (const auto& v : values) {
                            const char* name = propName(v.prop);
                            if (name == nullptr) continue;  // skip unmapped ids
                            if (!v.value.floatValues.empty()) {
                                std::printf("%-32s = %.3f\n", name, v.value.floatValues[0]);
                            } else if (!v.value.int32Values.empty()) {
                                std::printf("%-32s = %d\n", name, v.value.int32Values[0]);
                            }
                            std::fflush(stdout);
                        }
                    }));

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    while (!gStop) {
        sleep(1);
    }
    std::printf("\nStopping.\n");
    return 0;
}
