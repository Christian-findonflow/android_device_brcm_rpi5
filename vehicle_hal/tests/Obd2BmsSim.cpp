/*
 * Copyright (C) 2024 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * moto_bms_sim: answers Orion-style OBD2 Mode 0x22 requests on a CAN
 * interface with plausible pack data, so the BMS polling path (and the
 * screens it feeds) can be exercised on vcan without a battery.
 *
 * Usage: moto_bms_sim <iface>
 */

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

constexpr uint32_t kRequestId = 0x7E0;
constexpr uint32_t kResponseId = 0x7E8;

// value16 is the raw 16-bit little-endian payload; len 1 sends only the low
// byte. Values chosen to look like a healthy warm pack mid-discharge.
struct PidValue {
    uint16_t pid;
    uint8_t len;
    uint16_t value16;
};

constexpr PidValue kPids[] = {
        {0xF00A, 2, 60},      // charge limit (A)
        {0xF00B, 2, 300},     // discharge limit (A)
        {0xF00C, 2, 255},     // signed pack current, 25.5A (0.1A)
        {0xF00D, 2, 720},     // pack voltage, 72.0V (0.1V)
        {0xF00F, 1, 120},     // SoC, 60% (0.5%)
        {0xF010, 2, 683},     // pack amphours, 68.3Ah (0.1Ah)
        {0xF011, 2, 450},     // pack resistance, 4.5mOhm (0.01)
        {0xF012, 1, 80},      // depth of discharge, 40% (0.5%)
        {0xF013, 1, 97},      // SoH %
        {0xF018, 2, 143},     // cycles
        {0xF028, 1, 66},      // high temp, 26C (-40 offset)
        {0xF029, 1, 62},      // low temp, 22C
        {0xF02A, 1, 64},      // avg temp, 24C
        {0xF02B, 1, 2},       // fan speed
        {0xF02D, 1, 71},      // heatsink, 31C
        {0xF032, 2, 36480},   // low cell, 3.648V (0.0001)
        {0xF033, 2, 36620},   // high cell, 3.662V
        {0xF034, 2, 36550},   // avg cell, 3.655V
        {0xF03D, 2, 12},      // high cell id
        {0xF03E, 2, 3},       // low cell id
};

}  // namespace

int main(int argc, char** argv) {
    const char* iface = argc > 1 ? argv[1] : "vcan0";

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("socket(PF_CAN)");
        return 1;
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "no such CAN interface: %s\n", iface);
        return 1;
    }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    printf("moto_bms_sim answering 0x7E0 mode 0x22 on %s\n", iface);
    long answered = 0;

    struct can_frame frame;
    while (read(sock, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
        if ((frame.can_id & CAN_EFF_MASK) != kRequestId) continue;
        if (frame.can_dlc < 4 || frame.data[1] != 0x22) continue;

        uint16_t pid = (frame.data[2] << 8) | frame.data[3];
        for (const auto& entry : kPids) {
            if (entry.pid != pid) continue;

            struct can_frame reply;
            memset(&reply, 0, sizeof(reply));
            reply.can_id = kResponseId;
            reply.can_dlc = 8;
            reply.data[0] = 3 + entry.len;
            reply.data[1] = 0x62;
            reply.data[2] = frame.data[2];
            reply.data[3] = frame.data[3];
            reply.data[4] = entry.value16 & 0xFF;
            if (entry.len == 2) {
                reply.data[5] = (entry.value16 >> 8) & 0xFF;
            }
            if (write(sock, &reply, sizeof(reply)) == static_cast<ssize_t>(sizeof(reply))) {
                if (++answered % 20 == 1) {
                    printf("answered %ld requests (last PID 0x%04X)\n", answered, pid);
                    fflush(stdout);
                }
            }
            break;
        }
    }
    close(sock);
    return 0;
}
