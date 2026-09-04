/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Raw IMU capture, the lean-sensor counterpart of the CAN capture: every
 * sample the estimator saw (sensor frame, the CAN speed it was given and
 * what it concluded) so a real ride can be replayed on the host against a
 * re-tuned filter. Text, one record per line:
 *   I <t s> <ax> <ay> <az> <gx> <gy> <gz> <speed> <speedValid> <roll> <pitch> <status>
 *   B <t s> <pressure Pa> <temp C>
 * t is the monotonic clock (CLOCK_BOOTTIME seconds), so replay dt is exact.
 */
#pragma once

#include <cstdint>
#include <fstream>
#include <string>

#include "imu/Vec3.h"

namespace android::hardware::automotive::vehicle::motorcycle::imu {

struct ImuLogRecord {
    double tS = 0.0;
    ImuSample sample;
    float speedMps = 0.0f;
    bool speedValid = false;
    float rollDeg = 0.0f;   // what the estimator reported live
    float pitchDeg = 0.0f;
    int32_t status = 0;     // VENDOR_IMU_STATUS bits
};

struct ImuLogBaro {
    double tS = 0.0;
    float pressurePa = 0.0f;
    float tempC = 0.0f;
};

class ImuLogWriter {
  public:
    static constexpr uint64_t kMaxBytes = 128ULL * 1024 * 1024;  // ~4 h at 100 Hz
    ~ImuLogWriter();
    // Creates <dir>/imu-<epochS>.log with a header line.
    bool open(const std::string& dir, long epochS);
    bool isOpen() const { return mFd >= 0; }
    const std::string& path() const { return mPath; }
    uint64_t bytes() const { return mBytes; }
    bool writeSample(const ImuLogRecord& r);
    bool writeBaro(const ImuLogBaro& b);
    // fdatasync at most once a second: key-off is a power cut.
    void syncIfDue(int64_t nowNs);
    void close();

    static int formatSample(const ImuLogRecord& r, char* buf, size_t len);
    static int formatBaro(const ImuLogBaro& b, char* buf, size_t len);

  private:
    bool writeLine(const char* line, int len);
    int mFd = -1;
    std::string mPath;
    uint64_t mBytes = 0;
    int64_t mLastSyncNs = 0;
    bool mCapped = false;
};

class ImuLogReader {
  public:
    bool open(const std::string& path);
    // Next record; *isBaro tells which output was filled. False at EOF.
    bool next(ImuLogRecord* r, ImuLogBaro* b, bool* isBaro);
    static bool parseLine(const std::string& line, ImuLogRecord* r, ImuLogBaro* b, bool* isBaro);

  private:
    std::ifstream mIn;
};

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
