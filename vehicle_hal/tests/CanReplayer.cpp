/*
 * Copyright (C) 2024 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * moto_can_replay: send CAN frames from the command line or replay a
 * candump -l log at its original timing. Ships on simulator builds so test
 * traffic can be fed over adb (the platform has no can-utils).
 *
 * Usage:
 *   moto_can_replay <iface> <ID#HEXDATA> [more frames...]
 *   moto_can_replay <iface> -f <candump.log> [-s <speed>]
 *
 * ID#HEXDATA follows cansend conventions: more than 3 hex digits of ID means
 * an extended (29-bit) frame; data pairs may be '.'-separated.
 * candump -l log lines look like: (1700000000.123456) can1 10261022#0030B80B
 */

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

int openSocket(const char* iface) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("socket(PF_CAN)");
        return -1;
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "no such CAN interface: %s\n", iface);
        close(sock);
        return -1;
    }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return -1;
    }
    return sock;
}

// Parses "ID#HEXDATA" (cansend style). Returns false on malformed input.
bool parseFrame(const std::string& text, struct can_frame* frame) {
    auto hash = text.find('#');
    if (hash == std::string::npos || hash == 0) return false;

    std::string idStr = text.substr(0, hash);
    std::string dataStr = text.substr(hash + 1);

    char* end = nullptr;
    unsigned long id = strtoul(idStr.c_str(), &end, 16);
    if (*end != '\0' || id > CAN_EFF_MASK) return false;

    memset(frame, 0, sizeof(*frame));
    frame->can_id = id;
    if (idStr.size() > 3) frame->can_id |= CAN_EFF_FLAG;  // cansend convention

    std::string hex;
    for (char c : dataStr) {
        if (c == '.') continue;  // allow dotted pairs
        if (!isxdigit(static_cast<unsigned char>(c))) return false;
        hex += c;
    }
    if (hex.size() % 2 != 0 || hex.size() > 16) return false;

    frame->can_dlc = hex.size() / 2;
    for (int i = 0; i < frame->can_dlc; i++) {
        frame->data[i] = static_cast<uint8_t>(strtoul(hex.substr(i * 2, 2).c_str(), nullptr, 16));
    }
    return true;
}

bool sendFrame(int sock, const struct can_frame& frame) {
    if (write(sock, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("write");
        return false;
    }
    return true;
}

int replayLog(int sock, const char* path, double speed) {
    FILE* f = fopen(path, "r");
    if (f == nullptr) {
        perror(path);
        return 1;
    }
    char line[512];
    double lastTs = -1;
    long sent = 0;
    while (fgets(line, sizeof(line), f) != nullptr) {
        // (timestamp) iface ID#DATA
        double ts;
        char frameText[128];
        if (sscanf(line, "(%lf) %*s %127s", &ts, frameText) != 2) continue;

        struct can_frame frame;
        if (!parseFrame(frameText, &frame)) continue;

        if (lastTs >= 0 && ts > lastTs) {
            useconds_t delay = static_cast<useconds_t>((ts - lastTs) * 1e6 / speed);
            if (delay > 0) usleep(delay);
        }
        lastTs = ts;

        if (sendFrame(sock, frame)) sent++;
    }
    fclose(f);
    printf("replayed %ld frames\n", sent);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr,
                "usage: %s <iface> <ID#HEXDATA> [more frames...]\n"
                "       %s <iface> -f <candump.log> [-s <speed>]\n",
                argv[0], argv[0]);
        return 1;
    }

    int sock = openSocket(argv[1]);
    if (sock < 0) return 1;

    int ret = 0;
    if (strcmp(argv[2], "-f") == 0) {
        if (argc < 4) {
            fprintf(stderr, "-f requires a log file\n");
            ret = 1;
        } else {
            double speed = 1.0;
            if (argc >= 6 && strcmp(argv[4], "-s") == 0) {
                speed = atof(argv[5]);
                if (speed <= 0) speed = 1.0;
            }
            ret = replayLog(sock, argv[3], speed);
        }
    } else {
        for (int i = 2; i < argc; i++) {
            struct can_frame frame;
            if (!parseFrame(argv[i], &frame)) {
                fprintf(stderr, "malformed frame: %s\n", argv[i]);
                ret = 1;
                continue;
            }
            if (!sendFrame(sock, frame)) ret = 1;
        }
    }
    close(sock);
    return ret;
}
