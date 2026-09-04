/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal 3-vector plus the sensor-to-vehicle mounting frame shared by the
 * lean estimator, the synthetic IMU and the HAL. No Android dependencies:
 * everything here is host-testable.
 */
#pragma once

#include <cmath>

namespace android::hardware::automotive::vehicle::motorcycle::imu {

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float k) const { return {x * k, y * k, z * k}; }
    Vec3& operator+=(const Vec3& o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float norm() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        float n = norm();
        return n > 1e-9f ? (*this) * (1.0f / n) : Vec3{};
    }
};

inline constexpr float kGravityMps2 = 9.80665f;
inline constexpr float kDegToRad = 0.017453292519943295f;
inline constexpr float kRadToDeg = 57.29577951308232f;

// One accelerometer + gyroscope reading in the SENSOR frame. Accel is
// specific force in g (reads +1 g along whichever sensor axis points up when
// the bike stands still); gyro is angular rate in deg/s.
struct ImuSample {
    Vec3 accelG;
    Vec3 gyroDps;
};

// How the sensor sits on the bike. The vehicle frame is X forward, Y left,
// Z up (right-handed), so a positive roll about X is a lean to the RIGHT.
// "up" comes from the Workshop Level capture (bike upright and still);
// "forward" is learned from the first straight-line accelerations.
struct Mounting {
    Vec3 up;
    Vec3 forward;
    bool hasUp = false;
    bool hasForward = false;

    bool complete() const { return hasUp && hasForward; }
    Vec3 left() const { return up.cross(forward); }
    Vec3 toVehicle(const Vec3& s) const { return {forward.dot(s), left().dot(s), up.dot(s)}; }
    Vec3 toSensor(const Vec3& v) const { return forward * v.x + left() * v.y + up * v.z; }

    static Mounting identity() {
        Mounting m;
        m.up = {0.0f, 0.0f, 1.0f};
        m.forward = {1.0f, 0.0f, 0.0f};
        m.hasUp = m.hasForward = true;
        return m;
    }

    // Sets the up axis; an existing forward axis is re-projected so the two
    // stay orthogonal (and dropped if it collapses onto the new up).
    bool setUp(const Vec3& u) {
        if (u.norm() < 1e-6f) return false;
        up = u.normalized();
        hasUp = true;
        if (hasForward) {
            Vec3 f = forward - up * forward.dot(up);
            if (f.norm() < 0.5f) {
                hasForward = false;
            } else {
                forward = f.normalized();
            }
        }
        return true;
    }

    // Sets the forward axis from any vector with a usable horizontal part.
    bool setForward(const Vec3& f) {
        if (!hasUp) return false;
        Vec3 h = f - up * f.dot(up);
        if (h.norm() < 0.5f * f.norm() || h.norm() < 1e-6f) return false;
        forward = h.normalized();
        hasForward = true;
        return true;
    }
};

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
