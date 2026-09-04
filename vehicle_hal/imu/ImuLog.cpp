/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 */
#include "imu/ImuLog.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

namespace android::hardware::automotive::vehicle::motorcycle::imu {

namespace {
constexpr const char* kHeader =
        "# motodash imu log v1 | I t ax ay az(g) gx gy gz(deg/s) speed(m/s) speedValid "
        "roll pitch(deg, live estimate) status | B t pressure(Pa) temp(C)\n";
constexpr int64_t kSyncIntervalNs = 1000000000LL;
}  // namespace

ImuLogWriter::~ImuLogWriter() {
    close();
}

bool ImuLogWriter::open(const std::string& dir, long epochS) {
    close();
    mPath = dir + "/imu-" + std::to_string(epochS) + ".log";
    mFd = ::open(mPath.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0640);
    if (mFd < 0) return false;
    mBytes = 0;
    mCapped = false;
    mLastSyncNs = 0;
    return writeLine(kHeader, static_cast<int>(strlen(kHeader)));
}

int ImuLogWriter::formatSample(const ImuLogRecord& r, char* buf, size_t len) {
    return snprintf(buf, len, "I %.3f %.5f %.5f %.5f %.3f %.3f %.3f %.2f %d %.2f %.2f %d\n", r.tS,
                    r.sample.accelG.x, r.sample.accelG.y, r.sample.accelG.z, r.sample.gyroDps.x,
                    r.sample.gyroDps.y, r.sample.gyroDps.z, r.speedMps, r.speedValid ? 1 : 0,
                    r.rollDeg, r.pitchDeg, r.status);
}

int ImuLogWriter::formatBaro(const ImuLogBaro& b, char* buf, size_t len) {
    return snprintf(buf, len, "B %.3f %.1f %.2f\n", b.tS, b.pressurePa, b.tempC);
}

bool ImuLogWriter::writeSample(const ImuLogRecord& r) {
    char line[160];
    int n = formatSample(r, line, sizeof(line));
    return n > 0 && writeLine(line, n);
}

bool ImuLogWriter::writeBaro(const ImuLogBaro& b) {
    char line[64];
    int n = formatBaro(b, line, sizeof(line));
    return n > 0 && writeLine(line, n);
}

bool ImuLogWriter::writeLine(const char* line, int len) {
    if (mFd < 0 || mCapped) return false;
    if (mBytes + len > kMaxBytes) {
        mCapped = true;  // stopping beats filling /data
        return false;
    }
    if (::write(mFd, line, len) != len) return false;
    mBytes += len;
    return true;
}

void ImuLogWriter::syncIfDue(int64_t nowNs) {
    if (mFd >= 0 && nowNs - mLastSyncNs > kSyncIntervalNs) {
        fdatasync(mFd);
        mLastSyncNs = nowNs;
    }
}

void ImuLogWriter::close() {
    if (mFd >= 0) {
        fdatasync(mFd);
        ::close(mFd);
        mFd = -1;
    }
}

// ---------------------------------------------------------------------------

bool ImuLogReader::open(const std::string& path) {
    mIn.open(path);
    return mIn.is_open();
}

bool ImuLogReader::parseLine(const std::string& line, ImuLogRecord* r, ImuLogBaro* b,
                             bool* isBaro) {
    if (line.size() < 3 || line[0] == '#') return false;
    if (line[0] == 'I') {
        ImuLogRecord rec;
        int valid = 0, status = 0;
        int n = sscanf(line.c_str() + 1, "%lf %f %f %f %f %f %f %f %d %f %f %d", &rec.tS,
                       &rec.sample.accelG.x, &rec.sample.accelG.y, &rec.sample.accelG.z,
                       &rec.sample.gyroDps.x, &rec.sample.gyroDps.y, &rec.sample.gyroDps.z,
                       &rec.speedMps, &valid, &rec.rollDeg, &rec.pitchDeg, &status);
        if (n != 12) return false;
        rec.speedValid = valid != 0;
        rec.status = status;
        *r = rec;
        *isBaro = false;
        return true;
    }
    if (line[0] == 'B') {
        ImuLogBaro baro;
        if (sscanf(line.c_str() + 1, "%lf %f %f", &baro.tS, &baro.pressurePa, &baro.tempC) != 3) {
            return false;
        }
        *b = baro;
        *isBaro = true;
        return true;
    }
    return false;
}

bool ImuLogReader::next(ImuLogRecord* r, ImuLogBaro* b, bool* isBaro) {
    std::string line;
    while (std::getline(mIn, line)) {
        if (parseLine(line, r, b, isBaro)) return true;
    }
    return false;
}

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
