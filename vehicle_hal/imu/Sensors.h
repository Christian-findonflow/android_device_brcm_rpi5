/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register-level drivers for the two dash sensors, spoken directly over
 * /dev/i2c-1. The kernel's own lsm6dsx driver was rejected on purpose: with
 * no interrupt line on the Qwiic cable it cannot stream, and its one-shot
 * sysfs path powers the sensor up and down around every axis read (tens of
 * ms each), which caps a 6-axis poll near 10 Hz. Talking to the chip
 * ourselves gives a clean 100 Hz burst read and a driver the host tests can
 * exercise byte for byte.
 */
#pragma once

#include <cstdint>

#include "imu/I2cBus.h"
#include "imu/Vec3.h"

namespace android::hardware::automotive::vehicle::motorcycle::imu {

// ST ISM330DHCX 6-axis IMU (Adafruit 4502). Same register map as the rest
// of the LSM6DSx family for everything used here.
class Ism330dhcx {
  public:
    static constexpr uint8_t kDefaultAddr = 0x6A;  // 0x6B with the SDO/ADDR jumper
    static constexpr uint8_t kWhoAmI = 0x6B;

    static constexpr uint8_t REG_WHO_AM_I = 0x0F;
    static constexpr uint8_t REG_CTRL1_XL = 0x10;
    static constexpr uint8_t REG_CTRL2_G = 0x11;
    static constexpr uint8_t REG_CTRL3_C = 0x12;
    static constexpr uint8_t REG_STATUS = 0x1E;
    static constexpr uint8_t REG_OUT_TEMP_L = 0x20;  // temp(2) gyro(6) accel(6) contiguous

    // 104 Hz, +-4 g, LPF2 at ODR/4 (vibration); 104 Hz, +-500 dps; BDU + auto increment.
    static constexpr uint8_t CTRL1_XL_VALUE = 0x4A;
    static constexpr uint8_t CTRL2_G_VALUE = 0x44;
    static constexpr uint8_t CTRL3_C_VALUE = 0x44;
    static constexpr float kAccelGPerLsb = 0.000122f;   // +-4 g full scale
    static constexpr float kGyroDpsPerLsb = 0.0175f;    // +-500 dps full scale
    static constexpr int kBurstLen = 14;

    Ism330dhcx(I2cTransport* bus, uint8_t addr = kDefaultAddr) : mBus(bus), mAddr(addr) {}
    // Probes WHO_AM_I and configures the chip. Returns false if absent.
    bool init();
    uint8_t whoAmI() const { return mWhoAmI; }
    bool read(ImuSample* s, float* tempC);
    static void decode(const uint8_t* burst, ImuSample* s, float* tempC);

  private:
    I2cTransport* mBus;
    uint8_t mAddr;
    uint8_t mWhoAmI = 0;
};

// Bosch BMP280 barometer (Adafruit 2651). Compensation per the datasheet's
// integer reference implementation; a BME280 answers the same calls.
class Bmp280 {
  public:
    static constexpr uint8_t kDefaultAddr = 0x77;  // 0x76 with SDO to GND
    static constexpr uint8_t kIdBmp280 = 0x58;
    static constexpr uint8_t kIdBme280 = 0x60;

    static constexpr uint8_t REG_CALIB = 0x88;  // 24 bytes dig_T1..dig_P9
    static constexpr uint8_t REG_ID = 0xD0;
    static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
    static constexpr uint8_t REG_CONFIG = 0xF5;
    static constexpr uint8_t REG_PRESS_MSB = 0xF7;  // press(3) temp(3) contiguous
    // t_sb 125 ms, IIR filter 4; temp x2, pressure x16, normal mode.
    static constexpr uint8_t CONFIG_VALUE = 0x48;
    static constexpr uint8_t CTRL_MEAS_VALUE = 0x57;

    struct Calib {
        uint16_t T1 = 0;
        int16_t T2 = 0, T3 = 0;
        uint16_t P1 = 0;
        int16_t P2 = 0, P3 = 0, P4 = 0, P5 = 0, P6 = 0, P7 = 0, P8 = 0, P9 = 0;
    };

    Bmp280(I2cTransport* bus, uint8_t addr = kDefaultAddr) : mBus(bus), mAddr(addr) {}
    bool init();
    uint8_t chipId() const { return mId; }
    const Calib& calib() const { return mCalib; }
    bool read(float* pressurePa, float* tempC);

    static Calib parseCalib(const uint8_t* raw24);
    static void compensate(const Calib& c, int32_t adcT, int32_t adcP, float* tempC,
                           float* pressurePa);
    // ISA barometric altitude relative to a reference pressure.
    static float altitudeM(float pressurePa, float referencePa = 101325.0f);
    static float pressureAtAltitude(float altitudeM, float referencePa = 101325.0f);

  private:
    I2cTransport* mBus;
    uint8_t mAddr;
    uint8_t mId = 0;
    Calib mCalib;
};

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
