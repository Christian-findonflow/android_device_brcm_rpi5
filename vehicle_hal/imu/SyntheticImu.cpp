/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 */
#include "imu/SyntheticImu.h"

#include <cmath>

namespace android::hardware::automotive::vehicle::motorcycle::imu {

float SyntheticImu::balancedYawRate(float leanDeg, float speedMps) {
    if (speedMps < 0.5f) return 0.0f;
    return -kGravityMps2 * std::tan(leanDeg * kDegToRad) / speedMps;
}

ImuSample SyntheticImu::sample(const Scenario& sc) const {
    float phi = sc.leanDeg * kDegToRad;
    float theta = sc.pitchDeg * kDegToRad;
    float sp = std::sin(phi), cp = std::cos(phi);
    float st = std::sin(theta), ct = std::cos(theta);
    float yaw = balancedYawRate(sc.leanDeg, sc.speedMps);

    // World-frame specific force: forward accel, centripetal (v*yaw, + left),
    // and gravity reaction (+up). Rotate world -> vehicle by pitch about Y
    // then roll about X.
    Vec3 fw(sc.longAccelMps2, sc.speedMps * yaw, kGravityMps2);
    Vec3 afterPitch(fw.x * ct - fw.z * st, fw.y, fw.x * st + fw.z * ct);
    Vec3 fv(afterPitch.x, afterPitch.y * cp + afterPitch.z * sp,
            -afterPitch.y * sp + afterPitch.z * cp);

    // Angular rate: the world-vertical yaw rate seen in the vehicle frame,
    // plus the roll rate about the vehicle X axis.
    Vec3 ww(0.0f, 0.0f, yaw);
    Vec3 wAfterPitch(-ww.z * st, 0.0f, ww.z * ct);
    Vec3 wv(wAfterPitch.x + sc.rollRateDps * kDegToRad, wAfterPitch.z * sp, wAfterPitch.z * cp);

    ImuSample s;
    s.accelG = mMount.toSensor(fv) * (1.0f / kGravityMps2);
    s.gyroDps = mMount.toSensor(wv) * kRadToDeg;
    return s;
}

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
