#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "pigpio.h"

#include "ads1115.h"

#define ERR_CHECK(ret)             \
    if ((ret) < 0) {               \
        printf("error:%d\n", ret); \
        return 1;                  \
    }

int main(void) {
    int ret = 0;
    ret = gpioInitialise();
    ERR_CHECK(ret);
    ret = gpioSetMode(4, PI_INPUT);
    ERR_CHECK(ret);
    ret = gpioSetPullUpDown(4, PI_PUD_UP);
    ERR_CHECK(ret);
    ret = i2cOpen(1, 0x48, 0);
    ERR_CHECK(ret);
    unsigned int hndl = ret;
    ads1115_settings_t settings = {
        .mode = ads1115_mode_single_shot,
        .gain = ads1115_gain_2p048v,
        .rate = ads1115_rate_8sps,
        .channel = ads1115_channel_ain1_gnd,
        .pin_polarity = ads1115_pin_polarity_active_low,
        .comparator_latch = ads1115_comparator_latch_disable,
        .comparator_mode = ads1115_comparator_mode_traditional,
        .comparator_queue = ads1115_comparator_queue_1,
    };
    ret = ads1115_set_settings(hndl, &settings);
    ERR_CHECK(ret);
    ads1115_threshold_settings_t threshold_settings = {
        .low = 1,
        .high = -1,
    };
    ret = ads1115_set_threshold_settings(hndl, &threshold_settings);
    ERR_CHECK(ret);
    ret = ads1115_start_conversion(hndl);
    ERR_CHECK(ret);
    while (gpioRead(4) != PI_OFF) {
        usleep(100);
    }
    int16_t data = 0;
    ret = ads1115_get_data(hndl, &data);
    ERR_CHECK(ret);
    double voltage = 0;
    ads1115_compensate_data(data, &settings, &voltage);
    printf("voltage : %f\n", voltage);
    ret = i2cClose(hndl);
    ERR_CHECK(ret);
    ret = gpioSetPullUpDown(4, PI_PUD_OFF);
    ERR_CHECK(ret);
    gpioTerminate();
    return 0;
}
