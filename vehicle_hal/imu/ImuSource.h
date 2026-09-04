/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Where IMU samples come from: the real sensors on /dev/i2c-1, or a
 * scenario file that drives the synthetic IMU (emulator, e2e test, demo).
 */
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "imu/I2cBus.h"
#include "imu/Sensors.h"
#include "imu/SyntheticImu.h"
#include "imu/Vec3.h"

namespace android::hardware::automotive::vehicle::motorcycle::imu {

class ImuSource {
  public:
    virtual ~ImuSource() = default;
    virtual const char* name() const = 0;
    virtual bool isSimulated() const { return false; }
    virtual bool hasBaro() const { return false; }
    // One 6-axis sample in the sensor frame; false = source lost.
    virtual bool read(ImuSample* s, float* tempC) = 0;
    virtual bool readBaro(float* /*pressurePa*/, float* /*tempC*/) { return false; }
};

class I2cImuSource : public ImuSource {
  public:
    I2cImuSource(std::string devPath, uint8_t imuAddr, uint8_t baroAddr);
    // Opens the bus and probes both chips; the IMU is required, the
    // barometer optional. False = nothing usable on the bus.
    bool open();
    const char* name() const override { return "i2c"; }
    bool hasBaro() const override { return mHasBaro; }
    bool read(ImuSample* s, float* tempC) override;
    bool readBaro(float* pressurePa, float* tempC) override;
    uint8_t imuWhoAmI() const { return mImu.whoAmI(); }

  private:
    std::string mDevPath;
    LinuxI2cBus mBus;
    Ism330dhcx mImu;
    Bmp280 mBaro;
    bool mHasBaro = false;
};

// Scenario file, whitespace-separated key=value tokens:
//   lean=30 pitch=0 rollrate=0 along=0 alt=120 [speed=15.6]
// speed defaults to the bike's live CAN speed (speedProvider) so the
// synthetic corner is always consistent with what the HAL believes. The
// file is re-read when it changes; if it disappears the source ends.
class SimImuSource : public ImuSource {
  public:
    SimImuSource(std::string path, std::function<float()> speedProvider,
                 Mounting mount = Mounting::identity());
    const char* name() const override { return "sim"; }
    bool isSimulated() const override { return true; }
    bool hasBaro() const override { return true; }
    bool read(ImuSample* s, float* tempC) override;
    bool readBaro(float* pressurePa, float* tempC) override;
    const Scenario& scenario() const { return mScenario; }

    static bool parseScenario(const std::string& text, Scenario* sc, bool* hasSpeed,
                              float* altitudeM);

  private:
    bool refresh();

    std::string mPath;
    std::function<float()> mSpeed;
    SyntheticImu mImu;
    Scenario mScenario;
    bool mHasSpeed = false;
    float mAltitudeM = 0.0f;
    long mLastMtime = -1;
    int mReadsSinceCheck = 0;
};

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
