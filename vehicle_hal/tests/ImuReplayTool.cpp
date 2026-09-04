/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * motorcycle_imu_replay: runs the lean estimator over a raw IMU capture
 * (/data/vendor/motodash/imu-<epoch>.log, written alongside the CAN
 * capture) on the host, so filter constants can be tuned against a real
 * ride and compared with what the bike showed live.
 *
 * Usage: motorcycle_imu_replay <imu.log> [options]
 *   --up x,y,z --fwd x,y,z   mounting from persist.vendor.motodash.imu.{up,fwd}
 *   --level                  take the first 100 samples as the Level capture
 *   --identity               sensor axes = vehicle axes (default when nothing given)
 *   --roll-tau S  --pitch-tau S  --band G  --bias-tau S   estimator constants
 *   --csv <out.csv>          per-sample output for plotting
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "imu/ImuLog.h"
#include "imu/LeanEstimator.h"

using namespace android::hardware::automotive::vehicle::motorcycle::imu;

namespace {

bool parseVec(const char* s, Vec3* v) {
    return sscanf(s, "%f,%f,%f", &v->x, &v->y, &v->z) == 3;
}

void usage() {
    fprintf(stderr, "usage: motorcycle_imu_replay <imu.log> [--up x,y,z --fwd x,y,z | --level | "
                    "--identity] [--roll-tau S] [--pitch-tau S] [--band G] [--bias-tau S] "
                    "[--csv out.csv]\n");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 2;
    }
    std::string logPath = argv[1];
    Mounting mount;
    bool level = false;
    LeanEstimator::Config cfg;
    std::string csvPath;
    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        auto val = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                fprintf(stderr, "%s needs a value\n", name);
                exit(2);
            }
            return argv[++i];
        };
        if (a == "--up") {
            Vec3 v;
            if (!parseVec(val("--up"), &v) || !mount.setUp(v)) return 2;
        } else if (a == "--fwd") {
            Vec3 v;
            if (!parseVec(val("--fwd"), &v) || !mount.setForward(v)) return 2;
        } else if (a == "--level") {
            level = true;
        } else if (a == "--identity") {
            mount = Mounting::identity();
        } else if (a == "--roll-tau") {
            cfg.rollTauS = strtof(val(a.c_str()), nullptr);
        } else if (a == "--pitch-tau") {
            cfg.pitchTauS = strtof(val(a.c_str()), nullptr);
        } else if (a == "--band") {
            cfg.trustBandG = strtof(val(a.c_str()), nullptr);
        } else if (a == "--bias-tau") {
            cfg.biasTauS = strtof(val(a.c_str()), nullptr);
        } else if (a == "--csv") {
            csvPath = val("--csv");
        } else {
            usage();
            return 2;
        }
    }
    if (!mount.hasUp && !level) mount = Mounting::identity();

    ImuLogReader reader;
    if (!reader.open(logPath)) {
        fprintf(stderr, "cannot open %s\n", logPath.c_str());
        return 1;
    }
    FILE* csv = nullptr;
    if (!csvPath.empty()) {
        csv = fopen(csvPath.c_str(), "w");
        if (!csv) {
            fprintf(stderr, "cannot write %s\n", csvPath.c_str());
            return 1;
        }
        fprintf(csv, "t,speed,ax,ay,az,gx,gy,gz,roll_live,roll_replay,pitch_replay,lat_g,valid\n");
    }

    LeanEstimator est(cfg);
    est.setMounting(mount);
    LevelCapture cap;
    ImuLogRecord r;
    ImuLogBaro b;
    bool isBaro;
    double lastT = -1.0, firstT = -1.0, forwardAt = -1.0;
    long samples = 0, validSamples = 0, compared = 0;
    double sumSq = 0.0, maxDiff = 0.0;
    float maxL = 0.0f, maxR = 0.0f, maxLiveL = 0.0f, maxLiveR = 0.0f;
    float minPa = 1e9f, maxPa = 0.0f;
    while (reader.next(&r, &b, &isBaro)) {
        if (isBaro) {
            if (b.pressurePa < minPa) minPa = b.pressurePa;
            if (b.pressurePa > maxPa) maxPa = b.pressurePa;
            continue;
        }
        samples++;
        if (firstT < 0) firstT = r.tS;
        if (level && !cap.done()) {
            cap.add(r.sample);
            if (cap.done()) {
                Vec3 up;
                const char* why = "";
                if (cap.result(&up, &why)) {
                    Mounting m;
                    m.setUp(up);
                    est.setMounting(m);
                    printf("level: up = %.4f,%.4f,%.4f\n", up.x, up.y, up.z);
                } else {
                    fprintf(stderr, "level rejected: %s\n", why);
                    return 1;
                }
            }
        }
        float dt = lastT < 0 ? 0.0f : static_cast<float>(r.tS - lastT);
        lastT = r.tS;
        if (dt > 0.0f) est.update(r.sample, r.speedMps, r.speedValid, dt);
        if (est.takeForwardLearned()) {
            forwardAt = r.tS - firstT;
            const Vec3& f = est.mounting().forward;
            printf("forward learned at t+%.1fs: %.4f,%.4f,%.4f\n", forwardAt, f.x, f.y, f.z);
        }
        const auto& st = est.state();
        if (st.valid) {
            validSamples++;
            if (r.speedMps > 2.0f) {
                if (st.rollDeg > maxR) maxR = st.rollDeg;
                if (-st.rollDeg > maxL) maxL = -st.rollDeg;
                if (r.rollDeg > maxLiveR) maxLiveR = r.rollDeg;
                if (-r.rollDeg > maxLiveL) maxLiveL = -r.rollDeg;
            }
            if (r.status & (1 << 5)) {  // live estimate was valid too
                double d = st.rollDeg - r.rollDeg;
                sumSq += d * d;
                if (d < 0) d = -d;
                if (d > maxDiff) maxDiff = d;
                compared++;
            }
        }
        if (csv) {
            fprintf(csv, "%.3f,%.2f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%d\n", r.tS,
                    r.speedMps, r.sample.accelG.x, r.sample.accelG.y, r.sample.accelG.z,
                    r.sample.gyroDps.x, r.sample.gyroDps.y, r.sample.gyroDps.z, r.rollDeg,
                    st.rollDeg, st.pitchDeg, st.latG, st.valid ? 1 : 0);
        }
    }
    if (csv) fclose(csv);
    if (samples == 0) {
        fprintf(stderr, "no samples in %s\n", logPath.c_str());
        return 1;
    }
    double span = lastT - firstT;
    printf("%ld samples over %.1f s (%.1f Hz), estimate valid %.0f%% of the time\n", samples, span,
           span > 0 ? samples / span : 0.0, 100.0 * validSamples / samples);
    printf("mounting: up %s, forward %s%s\n", est.mounting().hasUp ? "set" : "unset",
           est.mounting().hasForward ? "set" : "unset",
           forwardAt >= 0 ? " (learned during this log)" : "");
    printf("max lean replay: %.1f L / %.1f R   live on the bike: %.1f L / %.1f R\n", maxL, maxR,
           maxLiveL, maxLiveR);
    if (compared > 0) {
        printf("replay vs live roll: rms %.2f deg, worst %.2f deg over %ld samples\n",
               std::sqrt(sumSq / compared), maxDiff, compared);
    }
    const auto& bias = est.state().gyroBiasDps;
    printf("gyro bias learned: %.2f %.2f %.2f deg/s\n", bias.x, bias.y, bias.z);
    if (maxPa > 0.0f) {
        printf("barometer: %.1f .. %.1f hPa\n", minPa / 100.0f, maxPa / 100.0f);
    }
    return 0;
}
