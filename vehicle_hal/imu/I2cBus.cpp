/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 */
#include "imu/I2cBus.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace android::hardware::automotive::vehicle::motorcycle::imu {

LinuxI2cBus::~LinuxI2cBus() {
    close();
}

bool LinuxI2cBus::open(const std::string& path) {
    close();
    mFd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    return mFd >= 0;
}

void LinuxI2cBus::close() {
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
}

bool LinuxI2cBus::transfer(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r,
                           size_t rlen) {
    if (mFd < 0) return false;
    struct i2c_msg msgs[2];
    int n = 0;
    if (wlen > 0) {
        msgs[n].addr = addr;
        msgs[n].flags = 0;
        msgs[n].len = static_cast<uint16_t>(wlen);
        msgs[n].buf = const_cast<uint8_t*>(w);
        n++;
    }
    if (rlen > 0) {
        msgs[n].addr = addr;
        msgs[n].flags = I2C_M_RD;
        msgs[n].len = static_cast<uint16_t>(rlen);
        msgs[n].buf = r;
        n++;
    }
    if (n == 0) return true;
    struct i2c_rdwr_ioctl_data data;
    data.msgs = msgs;
    data.nmsgs = n;
    return ::ioctl(mFd, I2C_RDWR, &data) == n;
}

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
