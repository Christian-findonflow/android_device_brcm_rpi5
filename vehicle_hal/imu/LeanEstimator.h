/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Vehicle-aware lean (roll) and pitch estimation for a motorcycle.
 *
 * Why not a plain IMU filter: in a balanced corner the apparent gravity lines
 * up with the bike, so an accelerometer reads ~0 deg of lean mid-bend and any
 * IMU-only AHRS survives the corner purely on gyro integration. This filter
 * uses what the bike already knows - wheel speed from the CAN bus - to remove
 * the centripetal (speed x yaw rate) and longitudinal (dv/dt) accelerations
 * from the accelerometer before estimating gravity. The corrected gravity
 * vector is then valid in the middle of a corner too, and it anchors the
 * gyro-integrated roll with a short time constant. Pure code: no clocks, no
 * hardware, fully driven by the caller (see tests/LeanEstimatorTest.cpp).
 */
#pragma once

#include "imu/Vec3.h"

namespace android::hardware::automotive::vehicle::motorcycle::imu {

class LeanEstimator {
  public:
    struct Config {
        float rollTauS = 1.0f;         // accel correction time constant for roll
        float pitchTauS = 3.0f;        // ... and pitch (slower: dv/dt is noisier)
        float snapTauS = 0.15f;        // fast convergence during the settle window
        float settleS = 2.0f;
        float trustBandG = 0.12f;      // corrected |g| must be within this fraction of g
        float noSpeedYawLimitDps = 2.0f;  // without CAN speed, accel is trusted only when straight
        float stationarySpeedMps = 0.3f;
        float stationaryGyroDps = 3.0f;
        float biasTauS = 3.0f;
        float longAccelTauS = 0.3f;
        float learnMinAccelMps2 = 0.8f;   // forward-axis learning gates
        float learnMaxYawDps = 4.0f;
        float learnNeededMps = 12.0f;     // accumulated |dv| before forward is declared
    };

    struct State {
        bool valid = false;        // mounting complete and the filter has settled
        bool stationary = false;
        float rollDeg = 0.0f;      // + = leaning right
        float pitchDeg = 0.0f;     // + = nose up
        float latG = 0.0f;         // ground-referenced lateral accel, + = right turn
        float longG = 0.0f;        // + = accelerating
        float yawRateDps = 0.0f;   // + = turning left
        Vec3 gyroBiasDps;
        float forwardProgress = 0.0f;  // 0..1 while the forward axis is being learned
    };

    LeanEstimator();
    explicit LeanEstimator(const Config& cfg);

    void setMounting(const Mounting& m);
    const Mounting& mounting() const { return mMount; }
    const State& state() const { return mState; }
    void reset();

    // One sensor sample. speedMps is the CAN wheel speed; speedValid is false
    // when the controller link is down (the filter then falls back to
    // straight-line-only accel trust). dtS is the time since the last sample.
    void update(const ImuSample& s, float speedMps, bool speedValid, float dtS);

    // True once, after the forward axis has just been learned, so the caller
    // can persist the completed mounting.
    bool takeForwardLearned();

  private:
    void learnForward(const ImuSample& s, float aLong, const Vec3& gyroDps, float dt);

    Config mCfg;
    Mounting mMount;
    State mState;
    float mRoll = 0.0f;   // radians
    float mPitch = 0.0f;  // radians
    float mSettled = 0.0f;
    float mPrevSpeed = 0.0f;
    bool mHaveSpeed = false;
    float mLongAccel = 0.0f;
    Vec3 mFwdSum;
    float mFwdWeight = 0.0f;
    bool mForwardLearned = false;
};

// Averages samples taken with the bike upright and still to find the sensor
// axis that points up. Rejects captures taken while moving or on a shaking
// bike so a bad Level press cannot poison the calibration.
class LevelCapture {
  public:
    static constexpr int kSamplesNeeded = 100;
    void add(const ImuSample& s);
    bool done() const { return mCount >= kSamplesNeeded; }
    int count() const { return mCount; }
    // Returns false (and a reason) if the capture is unusable.
    bool result(Vec3* up, const char** reason) const;
    void reset();

  private:
    Vec3 mAccelSum;
    Vec3 mGyroSum;
    float mMinNorm = 1e9f;
    float mMaxNorm = 0.0f;
    int mCount = 0;
};

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
