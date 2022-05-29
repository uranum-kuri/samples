#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "pigpio.h"

#include "bme280.h"

#define ERR_CHECK(ret)             \
    if ((ret) < 0) {               \
        printf("error:%d\n", ret); \
        return 1;                  \
    }

int main(void) {
    int ret = 0;
    ret = gpioInitialise();
    ERR_CHECK(ret);
    ret = spiOpen(0, 100000, 0);
    ERR_CHECK(ret);
    unsigned int hndl = ret;
    ret = bme280_reset(hndl);
    ERR_CHECK(ret);
    uint8_t status = 0;
    do {
        usleep(1);
        ret = bme280_get_status(hndl, &status);
        ERR_CHECK(ret);
    } while (bme280_status_im_update(status));
    uint8_t chip_id = 0;
    ret = bme280_get_chip_id(hndl, &chip_id);
    ERR_CHECK(ret);
    if (!bme280_chip_id_valid(chip_id)) {
        printf("chip id invalid\n");
        return 1;
    }
    printf("chip id: %x\n", chip_id);
    bme280_calib_data_t calib_data;
    ret = bme280_get_calib_data(hndl, &calib_data);
    ERR_CHECK(ret);
    bme280_settings_t settings = {.osr_temp = bme280_osr_temp_x16,
                                  .osr_pres = bme280_osr_pres_x16,
                                  .osr_hum = bme280_osr_hum_x16,
                                  .filter = bme280_filter_16};
    ret = bme280_set_settings(hndl, &settings);
    ERR_CHECK(ret);
    ret = bme280_set_mode(hndl, bme280_mode_forced);
    ERR_CHECK(ret);
    do {
        usleep(1000);
        ret = bme280_get_status(hndl, &status);
        ERR_CHECK(ret);
    } while (bme280_status_measuring(status));
    bme280_raw_data_t raw_data;
    ret = bme280_get_raw_data(hndl, &raw_data);
    ERR_CHECK(ret);
    bme280_data_t data;
    bme280_compensate_data(&raw_data, &calib_data, &data);
    printf("temperature: %lf\n", data.temperature);
    printf("pressure   : %lf\n", data.pressure);
    printf("humidity   : %lf\n", data.humidity);
    bme280_int_data_t data_int;
    bme280_compensate_data_int(&raw_data, &calib_data, &data_int);
    printf("temperature: %d\n", data_int.temperature);
    printf("pressure   : %u\n", data_int.pressure);
    printf("humidity   : %u\n", data_int.humidity);
    ret = spiClose(hndl);
    ERR_CHECK(ret);
    gpioTerminate();
    return 0;
}
