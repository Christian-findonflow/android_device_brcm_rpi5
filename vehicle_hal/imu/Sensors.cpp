/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 */
#include "imu/Sensors.h"

#include <cmath>

namespace android::hardware::automotive::vehicle::motorcycle::imu {

namespace {
int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}
uint16_t ule16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
}  // namespace

// ---------------------------------------------------------------------------
// ISM330DHCX
// ---------------------------------------------------------------------------

bool Ism330dhcx::init() {
    uint8_t id = 0;
    if (!mBus->readRegs(mAddr, REG_WHO_AM_I, &id, 1)) return false;
    // 0x6B ISM330DHCX; the LSM6DSO/DSM/DS3 answer 0x6C/0x6A/0x69 and share
    // this register subset, so a substitute board still works.
    if (id != kWhoAmI && id != 0x6C && id != 0x6A && id != 0x69) return false;
    mWhoAmI = id;
    return mBus->writeReg(mAddr, REG_CTRL3_C, CTRL3_C_VALUE) &&
           mBus->writeReg(mAddr, REG_CTRL1_XL, CTRL1_XL_VALUE) &&
           mBus->writeReg(mAddr, REG_CTRL2_G, CTRL2_G_VALUE);
}

bool Ism330dhcx::read(ImuSample* s, float* tempC) {
    uint8_t buf[kBurstLen];
    if (!mBus->readRegs(mAddr, REG_OUT_TEMP_L, buf, kBurstLen)) return false;
    decode(buf, s, tempC);
    return true;
}

void Ism330dhcx::decode(const uint8_t* b, ImuSample* s, float* tempC) {
    if (tempC) *tempC = 25.0f + le16(b) / 256.0f;
    s->gyroDps = Vec3(le16(b + 2), le16(b + 4), le16(b + 6)) * kGyroDpsPerLsb;
    s->accelG = Vec3(le16(b + 8), le16(b + 10), le16(b + 12)) * kAccelGPerLsb;
}

// ---------------------------------------------------------------------------
// BMP280
// ---------------------------------------------------------------------------

bool Bmp280::init() {
    uint8_t id = 0;
    if (!mBus->readRegs(mAddr, REG_ID, &id, 1)) return false;
    if (id != kIdBmp280 && id != kIdBme280) return false;
    mId = id;
    uint8_t raw[24];
    if (!mBus->readRegs(mAddr, REG_CALIB, raw, sizeof(raw))) return false;
    mCalib = parseCalib(raw);
    if (mCalib.T1 == 0 || mCalib.P1 == 0) return false;  // blank calibration = not a real chip
    return mBus->writeReg(mAddr, REG_CONFIG, CONFIG_VALUE) &&
           mBus->writeReg(mAddr, REG_CTRL_MEAS, CTRL_MEAS_VALUE);
}

Bmp280::Calib Bmp280::parseCalib(const uint8_t* r) {
    Calib c;
    c.T1 = ule16(r + 0);
    c.T2 = le16(r + 2);
    c.T3 = le16(r + 4);
    c.P1 = ule16(r + 6);
    c.P2 = le16(r + 8);
    c.P3 = le16(r + 10);
    c.P4 = le16(r + 12);
    c.P5 = le16(r + 14);
    c.P6 = le16(r + 16);
    c.P7 = le16(r + 18);
    c.P8 = le16(r + 20);
    c.P9 = le16(r + 22);
    return c;
}

bool Bmp280::read(float* pressurePa, float* tempC) {
    uint8_t b[6];
    if (!mBus->readRegs(mAddr, REG_PRESS_MSB, b, sizeof(b))) return false;
    int32_t adcP = (static_cast<int32_t>(b[0]) << 12) | (static_cast<int32_t>(b[1]) << 4) | (b[2] >> 4);
    int32_t adcT = (static_cast<int32_t>(b[3]) << 12) | (static_cast<int32_t>(b[4]) << 4) | (b[5] >> 4);
    if (adcP == 0x80000 || adcT == 0x80000) return false;  // measurement not ready
    compensate(mCalib, adcT, adcP, tempC, pressurePa);
    return true;
}

void Bmp280::compensate(const Calib& c, int32_t adcT, int32_t adcP, float* tempC,
                        float* pressurePa) {
    // Bosch BMP280 datasheet 3.11.3 / 8.1: 32-bit temperature, 64-bit pressure.
    int32_t var1 = ((((adcT >> 3) - (static_cast<int32_t>(c.T1) << 1))) * static_cast<int32_t>(c.T2)) >> 11;
    int32_t var2 = (((((adcT >> 4) - static_cast<int32_t>(c.T1)) *
                      ((adcT >> 4) - static_cast<int32_t>(c.T1))) >> 12) *
                    static_cast<int32_t>(c.T3)) >> 14;
    int32_t tFine = var1 + var2;
    int32_t t = (tFine * 5 + 128) >> 8;  // 0.01 degC
    if (tempC) *tempC = t / 100.0f;

    int64_t v1 = static_cast<int64_t>(tFine) - 128000;
    int64_t v2 = v1 * v1 * static_cast<int64_t>(c.P6);
    v2 = v2 + ((v1 * static_cast<int64_t>(c.P5)) << 17);
    v2 = v2 + (static_cast<int64_t>(c.P4) << 35);
    v1 = ((v1 * v1 * static_cast<int64_t>(c.P3)) >> 8) + ((v1 * static_cast<int64_t>(c.P2)) << 12);
    v1 = (((static_cast<int64_t>(1) << 47) + v1)) * static_cast<int64_t>(c.P1) >> 33;
    if (v1 == 0) {
        if (pressurePa) *pressurePa = 0.0f;
        return;
    }
    int64_t p = 1048576 - adcP;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = (static_cast<int64_t>(c.P9) * (p >> 13) * (p >> 13)) >> 25;
    v2 = (static_cast<int64_t>(c.P8) * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (static_cast<int64_t>(c.P7) << 4);
    if (pressurePa) *pressurePa = p / 256.0f;  // Q24.8 Pa
}

float Bmp280::altitudeM(float pressurePa, float referencePa) {
    if (pressurePa <= 0.0f || referencePa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - std::pow(pressurePa / referencePa, 0.190295f));
}

float Bmp280::pressureAtAltitude(float altitudeM, float referencePa) {
    return referencePa * std::pow(1.0f - altitudeM / 44330.0f, 5.255f);
}

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
