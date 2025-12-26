/*
 * Copyright (C) 2024 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "GnssHalRpi5"

#include "SerialPort.h"

#include <errno.h>
#include <fcntl.h>
#include <log/log.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>

namespace aidl::android::hardware::gnss::implementation {

SerialPort::SerialPort() {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& device, int baudRate) {
    mFd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (mFd < 0) {
        ALOGE("Failed to open %s: %s", device.c_str(), strerror(errno));
        return false;
    }

    // Save original termios
    if (tcgetattr(mFd, &mOriginalTermios) < 0) {
        ALOGE("tcgetattr failed: %s", strerror(errno));
        ::close(mFd);
        mFd = -1;
        return false;
    }
    mTermiosRestored = false;

    // Configure serial port
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    speed_t speed;
    switch (baudRate) {
        case 4800: speed = B4800; break;
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default: speed = B9600; break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, ignore modem control
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                  // 8 data bits
    tty.c_cflag &= ~PARENB;              // No parity
    tty.c_cflag &= ~CSTOPB;              // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;             // No hardware flow control

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100ms timeout

    if (tcsetattr(mFd, TCSANOW, &tty) < 0) {
        ALOGE("tcsetattr failed: %s", strerror(errno));
        ::close(mFd);
        mFd = -1;
        return false;
    }

    ALOGI("Opened serial port %s at %d baud", device.c_str(), baudRate);
    return true;
}

void SerialPort::close() {
    if (mFd >= 0) {
        if (!mTermiosRestored) {
            tcsetattr(mFd, TCSANOW, &mOriginalTermios);
            mTermiosRestored = true;
        }
        ::close(mFd);
        mFd = -1;
    }
}

int SerialPort::readLine(char* buffer, size_t maxLen, int timeoutMs) {
    if (mFd < 0 || buffer == nullptr || maxLen == 0) {
        return -1;
    }

    struct pollfd pfd;
    pfd.fd = mFd;
    pfd.events = POLLIN;

    size_t pos = 0;
    while (pos < maxLen - 1) {
        int ret = poll(&pfd, 1, timeoutMs);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (ret == 0) {
            // Timeout
            break;
        }

        char c;
        ssize_t n = read(mFd, &c, 1);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            return -1;
        }
        if (n == 0) continue;

        if (c == '\n') {
            buffer[pos] = '\0';
            return pos;
        }
        if (c != '\r') {
            buffer[pos++] = c;
        }
    }

    buffer[pos] = '\0';
    return pos;
}

}  // namespace aidl::android::hardware::gnss::implementation
