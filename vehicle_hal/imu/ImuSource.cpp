/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 */
#include "imu/ImuSource.h"

#include <sys/stat.h>

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace android::hardware::automotive::vehicle::motorcycle::imu {

I2cImuSource::I2cImuSource(std::string devPath, uint8_t imuAddr, uint8_t baroAddr)
    : mDevPath(std::move(devPath)), mImu(&mBus, imuAddr), mBaro(&mBus, baroAddr) {}

bool I2cImuSource::open() {
    if (!mBus.open(mDevPath)) return false;
    if (!mImu.init()) {
        mBus.close();
        return false;
    }
    mHasBaro = mBaro.init();
    return true;
}

bool I2cImuSource::read(ImuSample* s, float* tempC) {
    return mImu.read(s, tempC);
}

bool I2cImuSource::readBaro(float* pressurePa, float* tempC) {
    return mHasBaro && mBaro.read(pressurePa, tempC);
}

// ---------------------------------------------------------------------------

SimImuSource::SimImuSource(std::string path, std::function<float()> speedProvider, Mounting mount)
    : mPath(std::move(path)), mSpeed(std::move(speedProvider)), mImu(mount) {}

bool SimImuSource::parseScenario(const std::string& text, Scenario* sc, bool* hasSpeed,
                                 float* altitudeM) {
    Scenario out;
    bool speed = false;
    float alt = 0.0f;
    std::istringstream in(text);
    std::string tok;
    int recognised = 0;
    while (in >> tok) {
        auto eq = tok.find('=');
        if (eq == std::string::npos) continue;
        std::string key = tok.substr(0, eq);
        float v = strtof(tok.c_str() + eq + 1, nullptr);
        if (key == "lean") out.leanDeg = v;
        else if (key == "pitch") out.pitchDeg = v;
        else if (key == "rollrate") out.rollRateDps = v;
        else if (key == "along") out.longAccelMps2 = v;
        else if (key == "alt") alt = v;
        else if (key == "speed") { out.speedMps = v; speed = true; }
        else continue;
        recognised++;
    }
    if (recognised == 0) return false;
    *sc = out;
    *hasSpeed = speed;
    *altitudeM = alt;
    return true;
}

bool SimImuSource::refresh() {
    struct stat st;
    if (stat(mPath.c_str(), &st) != 0) return false;
    if (st.st_mtime == mLastMtime) return true;
    std::ifstream f(mPath);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    Scenario sc;
    bool hasSpeed;
    float alt;
    if (parseScenario(ss.str(), &sc, &hasSpeed, &alt)) {
        mScenario = sc;
        mHasSpeed = hasSpeed;
        mAltitudeM = alt;
    }
    mLastMtime = st.st_mtime;
    return true;
}

bool SimImuSource::read(ImuSample* s, float* tempC) {
    // stat() once per ~half second at 100 Hz, not per sample.
    if (mReadsSinceCheck-- <= 0) {
        if (!refresh()) {
            mReadsSinceCheck = 0;  // stay failed until the file is back
            return false;
        }
        mReadsSinceCheck = 50;
    }
    Scenario sc = mScenario;
    if (!mHasSpeed && mSpeed) sc.speedMps = mSpeed();
    *s = mImu.sample(sc);
    if (tempC) *tempC = 31.0f;
    return true;
}

bool SimImuSource::readBaro(float* pressurePa, float* tempC) {
    *pressurePa = Bmp280::pressureAtAltitude(mAltitudeM);
    if (tempC) *tempC = 22.0f;
    return true;
}

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
