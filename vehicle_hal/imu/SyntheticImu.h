/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generates physically consistent IMU samples for a motorcycle in a given
 * state (lean, pitch, speed, roll rate, longitudinal acceleration). Used by
 * the estimator's unit tests and by the HAL's simulator source, so the
 * exact same physics that proves the filter also drives the emulator.
 */
#pragma once

#include "imu/Vec3.h"

namespace android::hardware::automotive::vehicle::motorcycle::imu {

struct Scenario {
    float leanDeg = 0.0f;        // + = right
    float pitchDeg = 0.0f;       // + = nose up
    float speedMps = 0.0f;
    float rollRateDps = 0.0f;    // + = rolling right
    float longAccelMps2 = 0.0f;  // + = accelerating
};

class SyntheticImu {
  public:
    // "mount" places the sensor on the bike (sensor axes expressed through
    // the vehicle frame); identity = X forward, Y left, Z up.
    explicit SyntheticImu(Mounting mount = Mounting::identity()) : mMount(mount) {}

    // Yaw rate (rad/s, + = left turn) that balances a bike at this lean and
    // speed: tan(lean) = v * yaw / g. Zero below walking pace (a leaned,
    // stationary bike is simply resting on something).
    static float balancedYawRate(float leanDeg, float speedMps);

    ImuSample sample(const Scenario& sc) const;

  private:
    Mounting mMount;
};

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
