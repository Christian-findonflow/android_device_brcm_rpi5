// gpioscan: request the header lines that carry no other function on this
// build (BCM 4 5 6 12 13 17 18 19 22 23 26 27; SPI0 7-11, I2C 2/3, CAN INT
// 24/25, UART 14/15 and the VHAL's 16/20/21 are left alone - requesting a
// muxed pin as GPIO steals it from its driver, which killed the CAN HAT's
// SPI on 2026-09-12) with a pull-up and print the levels once per second,
// so an open-collector optoisolator output can be located without knowing
// its pin. Usage: gpioscan [seconds]
#include <fcntl.h>
#include <linux/gpio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    int secs = argc > 1 ? atoi(argv[1]) : 10;
    int chip = open("/dev/gpiochip0", O_RDONLY);
    if (chip < 0) { perror("open /dev/gpiochip0"); return 1; }
    static const int kFree[] = {4, 5, 6, 12, 13, 17, 18, 19, 22, 23, 26, 27};
    int fds[28]; for (int p = 0; p < 28; p++) fds[p] = -1;
    for (int p : kFree) {
        struct gpio_v2_line_request req; memset(&req, 0, sizeof(req));
        req.offsets[0] = p; req.num_lines = 1;
        req.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
        strncpy(req.consumer, "gpioscan", sizeof(req.consumer) - 1);
        if (ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req) == 0) fds[p] = req.fd;
    }
    for (int t = 0; t < secs; t++) {
        printf("t=%2d ", t);
        for (int p : kFree) {
            if (fds[p] < 0) { printf("%d=- ", p); continue; }
            struct gpio_v2_line_values v; memset(&v, 0, sizeof(v)); v.mask = 1;
            if (ioctl(fds[p], GPIO_V2_LINE_GET_VALUES_IOCTL, &v) == 0) printf("%d=%d ", p, (int)(v.bits & 1));
            else printf("%d=? ", p);
        }
        printf("\n"); fflush(stdout); sleep(1);
    }
    return 0;
}
