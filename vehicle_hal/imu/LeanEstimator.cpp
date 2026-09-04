/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 */
#include "imu/LeanEstimator.h"

#include <algorithm>
#include <cmath>

namespace android::hardware::automotive::vehicle::motorcycle::imu {

namespace {
float wrapPi(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}
}  // namespace

LeanEstimator::LeanEstimator() : LeanEstimator(Config()) {}

LeanEstimator::LeanEstimator(const Config& cfg) : mCfg(cfg) {}

void LeanEstimator::setMounting(const Mounting& m) {
    mMount = m;
    reset();
}

void LeanEstimator::reset() {
    State fresh;
    fresh.gyroBiasDps = mState.gyroBiasDps;  // bias is a sensor property, keep it
    mState = fresh;
    mRoll = mPitch = 0.0f;
    mSettled = 0.0f;
    mHaveSpeed = false;
    mLongAccel = 0.0f;
    mFwdSum = Vec3{};
    mFwdWeight = 0.0f;
    mForwardLearned = false;
}

bool LeanEstimator::takeForwardLearned() {
    bool v = mForwardLearned;
    mForwardLearned = false;
    return v;
}

void LeanEstimator::update(const ImuSample& s, float speedMps, bool speedValid, float dt) {
    if (dt <= 0.0f || dt > 0.5f) {
        // A gap (sensor hiccup, service restart): do not integrate across it.
        mPrevSpeed = speedMps;
        mHaveSpeed = speedValid;
        return;
    }

    // 1. Gyro bias: learn it whenever the bike is demonstrably still. Rate
    //    gyros drift with temperature, and a bike stops often enough.
    float rawGyroNorm = (s.gyroDps - mState.gyroBiasDps).norm();
    mState.stationary = speedValid && speedMps < mCfg.stationarySpeedMps &&
                        rawGyroNorm < mCfg.stationaryGyroDps;
    if (mState.stationary) {
        float k = dt / (mCfg.biasTauS + dt);
        mState.gyroBiasDps += (s.gyroDps - mState.gyroBiasDps) * k;
    }
    Vec3 gyroDps = s.gyroDps - mState.gyroBiasDps;

    // 2. Longitudinal acceleration from the CAN speed (the accelerometer
    //    cannot separate it from pitch on its own).
    float aLong = 0.0f;
    if (speedValid && mHaveSpeed) {
        float raw = (speedMps - mPrevSpeed) / dt;
        float k = dt / (mCfg.longAccelTauS + dt);
        mLongAccel += (raw - mLongAccel) * k;
        aLong = mLongAccel;
    } else {
        mLongAccel = 0.0f;
    }
    mPrevSpeed = speedMps;
    mHaveSpeed = speedValid;
    mState.longG = aLong / kGravityMps2;

    // 3. Mounting: nothing can be said before Level; before the forward
    //    axis is known we can only learn it.
    if (!mMount.hasUp) {
        mState.valid = false;
        return;
    }
    if (!mMount.hasForward) {
        if (speedValid) learnForward(s, aLong, gyroDps, dt);
        mState.valid = false;
        return;
    }

    Vec3 f = mMount.toVehicle(s.accelG) * kGravityMps2;  // specific force, m/s^2
    Vec3 w = mMount.toVehicle(gyroDps) * kDegToRad;       // rad/s

    // 4. Gyro propagation (roll about X; pitch from the Y/Z rates).
    float sp = std::sin(mRoll), cp = std::cos(mRoll);
    mRoll += w.x * dt;
    mPitch += (w.y * cp - w.z * sp) * dt;

    // 5. Accel correction. The centripetal term depends on the yaw rate
    //    about the world vertical, which itself depends on the roll we are
    //    solving for - so iterate the fixed point from the current roll.
    //    From upright in an established 30 deg corner it lands within a
    //    degree in three passes; when riding into a corner the gyro has
    //    already carried the roll and the first pass is consistent.
    //    Without wheel speed the lateral term is unknown and the accel is
    //    only trusted when the bike is not yawing.
    float rollAcc = mRoll;
    Vec3 g;
    for (int it = 0; it < 4; it++) {
        float s_ = std::sin(rollAcc), c_ = std::cos(rollAcc);
        float yawIt = w.y * s_ + w.z * c_;
        float aLatIt = speedValid ? speedMps * yawIt : 0.0f;
        Vec3 h(aLong, aLatIt * c_, -aLatIt * s_);
        g = f - h;
        rollAcc = std::atan2(g.y, g.z);
        if (!speedValid) break;
    }
    float gn = g.norm();
    float yawNow = w.y * std::sin(mRoll) + w.z * std::cos(mRoll);
    bool normOk = std::fabs(gn - kGravityMps2) < mCfg.trustBandG * kGravityMps2;
    bool straightEnough = speedValid || std::fabs(yawNow) * kRadToDeg < mCfg.noSpeedYawLimitDps;
    if (normOk && straightEnough) {
        float pitchAcc = std::atan2(-g.x, std::sqrt(g.y * g.y + g.z * g.z));
        bool settling = mSettled < mCfg.settleS;
        float tauR = settling ? mCfg.snapTauS : mCfg.rollTauS;
        float tauP = settling ? mCfg.snapTauS : mCfg.pitchTauS;
        mRoll += (dt / (tauR + dt)) * wrapPi(rollAcc - mRoll);
        mPitch += (dt / (tauP + dt)) * wrapPi(pitchAcc - mPitch);
    }
    mRoll = wrapPi(mRoll);
    mPitch = wrapPi(mPitch);
    mSettled += dt;

    float yaw = w.y * std::sin(mRoll) + w.z * std::cos(mRoll);
    float aLat = speedValid ? speedMps * yaw : 0.0f;
    mState.rollDeg = mRoll * kRadToDeg;
    mState.pitchDeg = mPitch * kRadToDeg;
    mState.latG = -aLat / kGravityMps2;  // + = right
    mState.yawRateDps = yaw * kRadToDeg;
    mState.valid = mSettled >= 0.5f;
}

void LeanEstimator::learnForward(const ImuSample& s, float aLong, const Vec3& gyroDps, float dt) {
    // While accelerating or braking in a straight line, the horizontal part
    // of the specific force points along the direction of travel. Weight
    // each sample by the speed change it represents so a long steady pull
    // outvotes a moment of lateral noise.
    float yawDps = std::fabs(gyroDps.dot(mMount.up));
    if (std::fabs(aLong) < mCfg.learnMinAccelMps2 || yawDps > mCfg.learnMaxYawDps) return;
    Vec3 f = s.accelG * kGravityMps2;
    Vec3 h = f - mMount.up * f.dot(mMount.up);
    if (h.norm() < 0.3f) return;
    Vec3 d = h.normalized() * (aLong > 0.0f ? 1.0f : -1.0f);
    float weight = std::fabs(aLong) * dt;
    mFwdSum += d * weight;
    mFwdWeight += weight;
    mState.forwardProgress = std::min(1.0f, mFwdWeight / mCfg.learnNeededMps);
    if (mFwdWeight >= mCfg.learnNeededMps) {
        if (mMount.setForward(mFwdSum)) {
            mForwardLearned = true;
            mSettled = 0.0f;  // let the attitude snap in from scratch
        }
        mFwdSum = Vec3{};
        mFwdWeight = 0.0f;
    }
}

// ---------------------------------------------------------------------------

void LevelCapture::add(const ImuSample& s) {
    mAccelSum += s.accelG;
    mGyroSum += s.gyroDps;
    float n = s.accelG.norm();
    mMinNorm = std::min(mMinNorm, n);
    mMaxNorm = std::max(mMaxNorm, n);
    mCount++;
}

bool LevelCapture::result(Vec3* up, const char** reason) const {
    if (mCount < kSamplesNeeded) {
        *reason = "not enough samples";
        return false;
    }
    Vec3 meanAccel = mAccelSum * (1.0f / mCount);
    Vec3 meanGyro = mGyroSum * (1.0f / mCount);
    float n = meanAccel.norm();
    if (n < 0.85f || n > 1.15f) {
        *reason = "accelerometer does not read 1 g - is the sensor moving or faulty?";
        return false;
    }
    if (meanGyro.norm() > 5.0f) {
        *reason = "bike is turning or shaking";
        return false;
    }
    if (mMaxNorm - mMinNorm > 0.25f) {
        *reason = "bike is not still";
        return false;
    }
    *up = meanAccel.normalized();
    *reason = "";
    return true;
}

void LevelCapture::reset() {
    mAccelSum = Vec3{};
    mGyroSum = Vec3{};
    mMinNorm = 1e9f;
    mMaxNorm = 0.0f;
    mCount = 0;
}

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
