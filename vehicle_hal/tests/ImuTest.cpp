/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for the lean estimator (driven by the synthetic IMU), the
 * sensor drivers (against a fake I2C bus) and the simulator source.
 */
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "imu/ImuLog.h"
#include "imu/ImuSource.h"
#include "imu/LeanEstimator.h"
#include "imu/Sensors.h"
#include "imu/SyntheticImu.h"

namespace android::hardware::automotive::vehicle::motorcycle::imu {
namespace {

constexpr float kDt = 0.01f;  // 100 Hz

// Runs a steady scenario through the estimator for `seconds`.
void run(LeanEstimator& est, const SyntheticImu& imu, const Scenario& sc, float seconds,
         bool speedValid = true, Vec3 gyroBias = Vec3{}) {
    int n = static_cast<int>(seconds / kDt);
    for (int i = 0; i < n; i++) {
        ImuSample s = imu.sample(sc);
        s.gyroDps += gyroBias;
        est.update(s, sc.speedMps, speedValid, kDt);
    }
}

// Rolls the bike from its current lean to `targetDeg` at `rateDps`, the way a
// rider would, so the gyro sees the transition.
void rollTo(LeanEstimator& est, const SyntheticImu& imu, Scenario& sc, float targetDeg,
            float rateDps = 40.0f) {
    float dir = targetDeg > sc.leanDeg ? 1.0f : -1.0f;
    sc.rollRateDps = dir * rateDps;
    while (dir * (targetDeg - sc.leanDeg) > 0.0f) {
        sc.leanDeg += sc.rollRateDps * kDt;
        est.update(imu.sample(sc), sc.speedMps, true, kDt);
    }
    sc.leanDeg = targetDeg;
    sc.rollRateDps = 0.0f;
}

// A sensor bolted in at an awkward angle: up and forward are arbitrary
// orthogonal unit vectors in the sensor frame.
Mounting awkwardMounting() {
    Mounting m;
    m.setUp(Vec3(0.20f, -0.35f, 0.91f));
    m.setForward(Vec3(0.80f, 0.55f, 0.10f));
    return m;
}

// ---------------------------------------------------------------------------
// Estimator
// ---------------------------------------------------------------------------

TEST(LeanEstimator, UprightStraightReadsZero) {
    SyntheticImu imu;
    LeanEstimator est;
    est.setMounting(Mounting::identity());
    Scenario sc;
    sc.speedMps = 15.0f;
    run(est, imu, sc, 3.0f);
    EXPECT_TRUE(est.state().valid);
    EXPECT_NEAR(est.state().rollDeg, 0.0f, 0.2f);
    EXPECT_NEAR(est.state().pitchDeg, 0.0f, 0.2f);
    EXPECT_NEAR(est.state().latG, 0.0f, 0.01f);
}

TEST(LeanEstimator, BalancedCornerReadsTrueLean) {
    // The classic failure: mid-corner the accelerometer feels no lateral
    // force at all. Confirm the physics first, then that the estimator
    // still reports the real lean because it knows the wheel speed.
    SyntheticImu imu;
    Scenario sc;
    sc.leanDeg = 30.0f;
    sc.speedMps = 15.0f;
    ImuSample s = imu.sample(sc);
    EXPECT_NEAR(s.accelG.y, 0.0f, 1e-3f);                       // no lateral force felt
    EXPECT_NEAR(s.accelG.z, 1.0f / std::cos(30.0f * kDegToRad), 1e-3f);  // 1.155 g load

    LeanEstimator est;
    est.setMounting(Mounting::identity());
    run(est, imu, sc, 3.0f);
    EXPECT_NEAR(est.state().rollDeg, 30.0f, 1.0f);
    EXPECT_NEAR(est.state().latG, std::tan(30.0f * kDegToRad), 0.03f);  // +right
    EXPECT_LT(est.state().yawRateDps, 0.0f);                              // right turn

    // Flip to a left-hander through the gyro and settle there.
    rollTo(est, imu, sc, -25.0f);
    run(est, imu, sc, 2.0f);
    EXPECT_NEAR(est.state().rollDeg, -25.0f, 1.0f);
    EXPECT_LT(est.state().latG, 0.0f);
}

TEST(LeanEstimator, EnteringACornerFollowsTheGyro) {
    SyntheticImu imu;
    LeanEstimator est;
    est.setMounting(Mounting::identity());
    Scenario sc;
    sc.speedMps = 15.0f;
    run(est, imu, sc, 2.0f);
    // Roll in at 30 deg/s for one second, then hold.
    sc.rollRateDps = 30.0f;
    float worst = 0.0f;
    for (int i = 0; i < 100; i++) {
        sc.leanDeg = 30.0f * (i + 1) / 100.0f;
        est.update(imu.sample(sc), sc.speedMps, true, kDt);
        worst = std::max(worst, std::fabs(est.state().rollDeg - sc.leanDeg));
    }
    EXPECT_LT(worst, 3.0f);
    sc.rollRateDps = 0.0f;
    run(est, imu, sc, 1.0f);
    EXPECT_NEAR(est.state().rollDeg, 30.0f, 1.0f);
}

TEST(LeanEstimator, LinkLossMidCornerKeepsTheLean) {
    // Without wheel speed the centripetal term is unknown; the estimator
    // must not "correct" a real 30 deg lean toward the 0 deg the raw
    // accelerometer suggests.
    SyntheticImu imu;
    LeanEstimator est;
    est.setMounting(Mounting::identity());
    Scenario sc;
    sc.leanDeg = 30.0f;
    sc.speedMps = 15.0f;
    run(est, imu, sc, 3.0f);
    ASSERT_NEAR(est.state().rollDeg, 30.0f, 1.0f);
    run(est, imu, sc, 5.0f, /*speedValid=*/false);
    EXPECT_NEAR(est.state().rollDeg, 30.0f, 2.0f);
}

TEST(LeanEstimator, AwkwardMountingWithLevelAndLearnedForward) {
    Mounting truth = awkwardMounting();
    SyntheticImu imu(truth);
    LeanEstimator est;

    // Level: bike upright and still.
    LevelCapture cap;
    Scenario still;
    for (int i = 0; i < LevelCapture::kSamplesNeeded; i++) cap.add(imu.sample(still));
    Vec3 up;
    const char* why = nullptr;
    ASSERT_TRUE(cap.result(&up, &why)) << why;
    EXPECT_GT(up.dot(truth.up), std::cos(0.5f * kDegToRad));
    Mounting m;
    m.setUp(up);
    est.setMounting(m);
    EXPECT_FALSE(est.state().valid);

    // Pull away in a straight line: 2 m/s^2 for 8 s.
    Scenario sc;
    sc.longAccelMps2 = 2.0f;
    for (int i = 0; i < 800; i++) {
        sc.speedMps = 2.0f * (i + 1) * kDt;
        est.update(imu.sample(sc), sc.speedMps, true, kDt);
    }
    EXPECT_TRUE(est.takeForwardLearned());
    EXPECT_TRUE(est.mounting().complete());
    EXPECT_GT(est.mounting().forward.dot(truth.forward), std::cos(1.0f * kDegToRad));

    // And now a corner reads correctly through the rotated sensor.
    sc.longAccelMps2 = 0.0f;
    sc.speedMps = 16.0f;
    sc.leanDeg = 30.0f;
    run(est, imu, sc, 3.0f);
    EXPECT_TRUE(est.state().valid);
    EXPECT_NEAR(est.state().rollDeg, 30.0f, 1.0f);
}

TEST(LeanEstimator, BrakingLearnsForwardToo) {
    Mounting truth = awkwardMounting();
    SyntheticImu imu(truth);
    LeanEstimator est;
    Mounting m;
    m.setUp(truth.up);
    est.setMounting(m);
    Scenario sc;
    sc.longAccelMps2 = -3.0f;
    for (int i = 0; i < 600; i++) {
        sc.speedMps = 18.0f - 3.0f * (i + 1) * kDt;
        est.update(imu.sample(sc), sc.speedMps, true, kDt);
    }
    EXPECT_TRUE(est.takeForwardLearned());
    EXPECT_GT(est.mounting().forward.dot(truth.forward), std::cos(1.0f * kDegToRad));
}

TEST(LeanEstimator, GyroBiasLearnedAtStandstillAndDriftHeldStraight) {
    SyntheticImu imu;
    LeanEstimator est;
    est.setMounting(Mounting::identity());
    Vec3 bias(2.0f, -1.0f, 0.5f);
    Scenario still;
    run(est, imu, still, 10.0f, true, bias);
    EXPECT_NEAR(est.state().gyroBiasDps.x, 2.0f, 0.2f);
    EXPECT_NEAR(est.state().gyroBiasDps.y, -1.0f, 0.2f);
    EXPECT_NEAR(est.state().gyroBiasDps.z, 0.5f, 0.2f);
    // Long straight with the bias still present: the accel anchor holds it.
    Scenario sc;
    sc.speedMps = 20.0f;
    run(est, imu, sc, 60.0f, true, bias);
    EXPECT_NEAR(est.state().rollDeg, 0.0f, 1.0f);
}

TEST(LeanEstimator, HardBrakingDoesNotTiltRoll) {
    SyntheticImu imu;
    LeanEstimator est;
    est.setMounting(Mounting::identity());
    Scenario sc;
    sc.speedMps = 25.0f;
    run(est, imu, sc, 2.0f);
    sc.longAccelMps2 = -6.0f;  // 0.6 g stop
    float worst = 0.0f;
    for (int i = 0; i < 300; i++) {
        sc.speedMps = 25.0f - 6.0f * (i + 1) * kDt;
        est.update(imu.sample(sc), sc.speedMps, true, kDt);
        worst = std::max(worst, std::fabs(est.state().rollDeg));
    }
    EXPECT_LT(worst, 0.5f);
    EXPECT_NEAR(est.state().longG, -6.0f / kGravityMps2, 0.05f);
}

TEST(LeanEstimator, SideStandLeanAtStandstillIsReported) {
    // Bike resting on its stand: a real 12 deg lean with no speed. The HAL
    // must gate the ride's max-lean statistic on speed, not the estimator.
    SyntheticImu imu;
    LeanEstimator est;
    est.setMounting(Mounting::identity());
    Scenario sc;
    sc.leanDeg = -12.0f;
    run(est, imu, sc, 3.0f);
    EXPECT_NEAR(est.state().rollDeg, -12.0f, 0.5f);
    EXPECT_TRUE(est.state().stationary);
}

TEST(LeanEstimator, GapInSamplesIsNotIntegrated) {
    SyntheticImu imu;
    LeanEstimator est;
    est.setMounting(Mounting::identity());
    Scenario sc;
    sc.speedMps = 15.0f;
    run(est, imu, sc, 2.0f);
    ImuSample s = imu.sample(sc);
    s.gyroDps.x = 100.0f;  // a huge rate across a 2 s hole must be ignored
    est.update(s, sc.speedMps, true, 2.0f);
    EXPECT_NEAR(est.state().rollDeg, 0.0f, 0.5f);
}

TEST(LevelCapture, RejectsMovementAndBadGravity) {
    SyntheticImu imu;
    LevelCapture cap;
    Scenario sc;
    for (int i = 0; i < LevelCapture::kSamplesNeeded; i++) {
        ImuSample s = imu.sample(sc);
        s.gyroDps.z = 20.0f;
        cap.add(s);
    }
    Vec3 up;
    const char* why = nullptr;
    EXPECT_FALSE(cap.result(&up, &why));
    EXPECT_STRNE(why, "");

    cap.reset();
    for (int i = 0; i < LevelCapture::kSamplesNeeded; i++) {
        ImuSample s = imu.sample(sc);
        s.accelG = s.accelG * 0.5f;  // half a g: broken sensor or wrong scale
        cap.add(s);
    }
    EXPECT_FALSE(cap.result(&up, &why));
    EXPECT_FALSE(cap.done() && cap.count() == 0);
}

TEST(Mounting, FramesAreRightHanded) {
    Mounting m = Mounting::identity();
    Vec3 l = m.left();
    EXPECT_NEAR(l.y, 1.0f, 1e-6f);
    // A right lean rotates the world-up vector toward the bike's LEFT side
    // in the vehicle frame, which is what makes +roll = right.
    SyntheticImu imu;
    Scenario sc;
    sc.leanDeg = 20.0f;
    ImuSample s = imu.sample(sc);
    EXPECT_GT(s.accelG.y, 0.0f);
    EXPECT_NEAR(s.accelG.norm(), 1.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// Drivers on a fake bus
// ---------------------------------------------------------------------------

class FakeI2c : public I2cTransport {
  public:
    std::map<uint8_t, std::map<uint8_t, uint8_t>> regs;  // addr -> reg -> value
    std::vector<std::pair<uint8_t, uint8_t>> writes;     // (reg, value) in order
    bool fail = false;

    bool transfer(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen) override {
        if (fail || regs.find(addr) == regs.end()) return false;
        auto& dev = regs[addr];
        if (wlen == 2 && rlen == 0) {
            dev[w[0]] = w[1];
            writes.emplace_back(w[0], w[1]);
            return true;
        }
        if (wlen == 1) {
            for (size_t i = 0; i < rlen; i++) {
                uint8_t reg = static_cast<uint8_t>(w[0] + i);
                auto it = dev.find(reg);
                r[i] = it == dev.end() ? 0 : it->second;
            }
            return true;
        }
        return false;
    }
    void put16(uint8_t addr, uint8_t reg, int16_t v) {
        regs[addr][reg] = static_cast<uint8_t>(v & 0xFF);
        regs[addr][reg + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    }
};

TEST(Ism330dhcx, ProbesConfiguresAndDecodes) {
    FakeI2c bus;
    bus.regs[0x6A][Ism330dhcx::REG_WHO_AM_I] = 0x6B;
    // Temperature 27.5 C, gyro (10, -20, 30) dps, accel (0.1, 0.2, 1.0) g.
    bus.put16(0x6A, 0x20, static_cast<int16_t>(2.5f * 256));
    bus.put16(0x6A, 0x22, static_cast<int16_t>(10.0f / 0.0175f));
    bus.put16(0x6A, 0x24, static_cast<int16_t>(-20.0f / 0.0175f));
    bus.put16(0x6A, 0x26, static_cast<int16_t>(30.0f / 0.0175f));
    bus.put16(0x6A, 0x28, static_cast<int16_t>(0.1f / 0.000122f));
    bus.put16(0x6A, 0x2A, static_cast<int16_t>(0.2f / 0.000122f));
    bus.put16(0x6A, 0x2C, static_cast<int16_t>(1.0f / 0.000122f));

    Ism330dhcx imu(&bus);
    ASSERT_TRUE(imu.init());
    EXPECT_EQ(imu.whoAmI(), 0x6B);
    ASSERT_EQ(bus.writes.size(), 3u);
    EXPECT_EQ(bus.writes[0], std::make_pair(Ism330dhcx::REG_CTRL3_C, Ism330dhcx::CTRL3_C_VALUE));
    EXPECT_EQ(bus.writes[1], std::make_pair(Ism330dhcx::REG_CTRL1_XL, Ism330dhcx::CTRL1_XL_VALUE));
    EXPECT_EQ(bus.writes[2], std::make_pair(Ism330dhcx::REG_CTRL2_G, Ism330dhcx::CTRL2_G_VALUE));

    ImuSample s;
    float temp = 0.0f;
    ASSERT_TRUE(imu.read(&s, &temp));
    EXPECT_NEAR(temp, 27.5f, 0.01f);
    EXPECT_NEAR(s.gyroDps.x, 10.0f, 0.02f);
    EXPECT_NEAR(s.gyroDps.y, -20.0f, 0.02f);
    EXPECT_NEAR(s.gyroDps.z, 30.0f, 0.02f);
    EXPECT_NEAR(s.accelG.x, 0.1f, 0.001f);
    EXPECT_NEAR(s.accelG.y, 0.2f, 0.001f);
    EXPECT_NEAR(s.accelG.z, 1.0f, 0.001f);
}

TEST(Ism330dhcx, AbsentOrWrongChipFailsProbe) {
    FakeI2c bus;
    Ism330dhcx none(&bus);
    EXPECT_FALSE(none.init());
    bus.regs[0x6A][Ism330dhcx::REG_WHO_AM_I] = 0x58;  // that's a BMP280 answering
    Ism330dhcx wrong(&bus);
    EXPECT_FALSE(wrong.init());
    EXPECT_TRUE(bus.writes.empty());
}

Bmp280::Calib datasheetCalib() {
    // Example calibration from the Bosch BMP280 datasheet compensation walk-through.
    Bmp280::Calib c;
    c.T1 = 27504;
    c.T2 = 26435;
    c.T3 = -1000;
    c.P1 = 36477;
    c.P2 = -10685;
    c.P3 = 3024;
    c.P4 = 2855;
    c.P5 = 140;
    c.P6 = -7;
    c.P7 = 15500;
    c.P8 = -14600;
    c.P9 = 6000;
    return c;
}

TEST(Bmp280, CompensationMatchesDatasheetExample) {
    float t = 0.0f, p = 0.0f;
    Bmp280::compensate(datasheetCalib(), 519888, 415148, &t, &p);
    EXPECT_NEAR(t, 25.08f, 0.02f);
    EXPECT_NEAR(p, 100653.27f, 2.0f);
}

TEST(Bmp280, ProbeReadsCalibrationAndPressure) {
    FakeI2c bus;
    auto& dev = bus.regs[0x77];
    dev[Bmp280::REG_ID] = Bmp280::kIdBmp280;
    Bmp280::Calib c = datasheetCalib();
    auto put = [&](uint8_t reg, int32_t v) {
        dev[reg] = static_cast<uint8_t>(v & 0xFF);
        dev[reg + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };
    put(0x88, c.T1); put(0x8A, c.T2); put(0x8C, c.T3);
    put(0x8E, c.P1); put(0x90, c.P2); put(0x92, c.P3); put(0x94, c.P4);
    put(0x96, c.P5); put(0x98, c.P6); put(0x9A, c.P7); put(0x9C, c.P8); put(0x9E, c.P9);
    // adc_P 415148 = 0x65 5A C0 (20-bit left-aligned), adc_T 519888 = 0x7E ED 00
    dev[0xF7] = 0x65; dev[0xF8] = 0x5A; dev[0xF9] = 0xC0;
    dev[0xFA] = 0x7E; dev[0xFB] = 0xED; dev[0xFC] = 0x00;

    Bmp280 baro(&bus);
    ASSERT_TRUE(baro.init());
    EXPECT_EQ(baro.calib().P9, 6000);
    ASSERT_EQ(bus.writes.size(), 2u);
    EXPECT_EQ(bus.writes[0], std::make_pair(Bmp280::REG_CONFIG, Bmp280::CONFIG_VALUE));
    EXPECT_EQ(bus.writes[1], std::make_pair(Bmp280::REG_CTRL_MEAS, Bmp280::CTRL_MEAS_VALUE));
    float pa = 0.0f, t = 0.0f;
    ASSERT_TRUE(baro.read(&pa, &t));
    EXPECT_NEAR(pa, 100653.27f, 2.0f);
    EXPECT_NEAR(t, 25.08f, 0.02f);
}

TEST(Bmp280, AltitudeRoundTrips) {
    EXPECT_NEAR(Bmp280::altitudeM(101325.0f), 0.0f, 0.01f);
    EXPECT_NEAR(Bmp280::altitudeM(89875.0f), 1000.0f, 5.0f);  // ISA 1000 m
    EXPECT_NEAR(Bmp280::altitudeM(Bmp280::pressureAtAltitude(350.0f)), 350.0f, 0.5f);
}

// ---------------------------------------------------------------------------
// Simulator source
// ---------------------------------------------------------------------------

TEST(SimImuSource, ScenarioFileDrivesConsistentSamples) {
    Scenario sc;
    bool hasSpeed = true;
    float alt = -1.0f;
    ASSERT_TRUE(SimImuSource::parseScenario("lean=30\npitch=2 alt=120 junk\n", &sc, &hasSpeed, &alt));
    EXPECT_FLOAT_EQ(sc.leanDeg, 30.0f);
    EXPECT_FLOAT_EQ(sc.pitchDeg, 2.0f);
    EXPECT_FALSE(hasSpeed);
    EXPECT_FLOAT_EQ(alt, 120.0f);
    EXPECT_FALSE(SimImuSource::parseScenario("nothing here", &sc, &hasSpeed, &alt));

    std::string path = std::string(testing::TempDir()) + "/imu_sim_test";
    {
        std::ofstream f(path);
        f << "lean=30 alt=120\n";
    }
    float speed = 15.0f;
    SimImuSource src(path, [&] { return speed; });
    ImuSample s;
    ASSERT_TRUE(src.read(&s, nullptr));
    EXPECT_NEAR(s.accelG.y, 0.0f, 1e-3f);  // a balanced corner at the live speed
    EXPECT_LT(s.gyroDps.z, -15.0f);         // yawing right (21.6 dps x cos 30)
    float pa = 0.0f;
    ASSERT_TRUE(src.readBaro(&pa, nullptr));
    EXPECT_NEAR(Bmp280::altitudeM(pa), 120.0f, 0.5f);

    // The estimator fed from the sim converges to the scenario's lean.
    LeanEstimator est;
    est.setMounting(Mounting::identity());
    for (int i = 0; i < 300; i++) {
        ASSERT_TRUE(src.read(&s, nullptr));
        est.update(s, speed, true, kDt);
    }
    EXPECT_NEAR(est.state().rollDeg, 30.0f, 1.0f);

    std::remove(path.c_str());
    for (int i = 0; i < 60; i++) {
        if (!src.read(&s, nullptr)) break;
    }
    EXPECT_FALSE(src.read(&s, nullptr));  // file gone = source gone
}

// ---------------------------------------------------------------------------
// Raw capture format
// ---------------------------------------------------------------------------

TEST(ImuLog, RoundTrip) {
    std::string path;
    {
        ImuLogWriter w;
        ASSERT_TRUE(w.open(std::string(testing::TempDir()), 1757000000L));
        path = w.path();
        EXPECT_NE(path.find("imu-1757000000.log"), std::string::npos);
        ImuLogRecord r;
        r.tS = 12.345;
        r.sample.accelG = Vec3(0.01234f, -0.5f, 0.86603f);
        r.sample.gyroDps = Vec3(-1.5f, 22.125f, 0.0f);
        r.speedMps = 15.55f;
        r.speedValid = true;
        r.rollDeg = 29.9f;
        r.pitchDeg = -1.25f;
        r.status = 63;
        ASSERT_TRUE(w.writeSample(r));
        ImuLogBaro b;
        b.tS = 12.35;
        b.pressurePa = 100653.3f;
        b.tempC = 25.08f;
        ASSERT_TRUE(w.writeBaro(b));
        r.tS = 12.355;
        r.speedValid = false;
        ASSERT_TRUE(w.writeSample(r));
        w.syncIfDue(20000000000LL);
        EXPECT_GT(w.bytes(), 100u);
    }
    ImuLogReader reader;
    ASSERT_TRUE(reader.open(path));
    ImuLogRecord r;
    ImuLogBaro b;
    bool isBaro = true;
    ASSERT_TRUE(reader.next(&r, &b, &isBaro));
    EXPECT_FALSE(isBaro);
    EXPECT_NEAR(r.tS, 12.345, 1e-6);
    EXPECT_NEAR(r.sample.accelG.x, 0.01234f, 1e-5f);
    EXPECT_NEAR(r.sample.accelG.z, 0.86603f, 1e-5f);
    EXPECT_NEAR(r.sample.gyroDps.y, 22.125f, 1e-3f);
    EXPECT_NEAR(r.speedMps, 15.55f, 1e-3f);
    EXPECT_TRUE(r.speedValid);
    EXPECT_NEAR(r.rollDeg, 29.9f, 1e-3f);
    EXPECT_NEAR(r.pitchDeg, -1.25f, 1e-3f);
    EXPECT_EQ(r.status, 63);
    ASSERT_TRUE(reader.next(&r, &b, &isBaro));
    EXPECT_TRUE(isBaro);
    EXPECT_NEAR(b.pressurePa, 100653.3f, 0.1f);
    EXPECT_NEAR(b.tempC, 25.08f, 1e-3f);
    ASSERT_TRUE(reader.next(&r, &b, &isBaro));
    EXPECT_FALSE(isBaro);
    EXPECT_FALSE(r.speedValid);
    EXPECT_FALSE(reader.next(&r, &b, &isBaro));
    std::remove(path.c_str());

    // Garbage and comments are skipped, not misread.
    EXPECT_FALSE(ImuLogReader::parseLine("# header", &r, &b, &isBaro));
    EXPECT_FALSE(ImuLogReader::parseLine("I 1.0 0.1", &r, &b, &isBaro));
    EXPECT_FALSE(ImuLogReader::parseLine("", &r, &b, &isBaro));
}

}  // namespace
}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
