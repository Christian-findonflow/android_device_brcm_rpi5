/*
 * Copyright (C) 2026 MotoDash Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C transport abstraction: the sensor drivers talk to an I2cTransport so
 * the host tests can substitute a fake bus that serves canned registers.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace android::hardware::automotive::vehicle::motorcycle::imu {

class I2cTransport {
  public:
    virtual ~I2cTransport() = default;
    // Writes wlen bytes to the device, then (if rlen > 0) reads rlen bytes,
    // as one combined transaction (repeated start, single STOP).
    virtual bool transfer(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r,
                          size_t rlen) = 0;

    bool readRegs(uint8_t addr, uint8_t reg, uint8_t* out, size_t n) {
        return transfer(addr, &reg, 1, out, n);
    }
    bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
        uint8_t b[2] = {reg, val};
        return transfer(addr, b, 2, nullptr, 0);
    }
};

// /dev/i2c-N through the I2C_RDWR ioctl (CONFIG_I2C_CHARDEV=y on the Pi).
class LinuxI2cBus : public I2cTransport {
  public:
    LinuxI2cBus() = default;
    ~LinuxI2cBus() override;
    bool open(const std::string& path);
    void close();
    bool isOpen() const { return mFd >= 0; }
    bool transfer(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r,
                  size_t rlen) override;

  private:
    int mFd = -1;
};

}  // namespace android::hardware::automotive::vehicle::motorcycle::imu
