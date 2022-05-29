/**
 * MIT License
 *
 * Copyright (c) 2022 uranum
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file bme280.c
 * @brief bme280 driver source file
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "pigpio.h"

#include "bme280.h"

/// @name register address
/// @{

#define RESET_ADDRESS UINT8_C(0xE0)    ///< reset register address
#define CHIP_ID_ADDRESS UINT8_C(0xD0)  ///< id register address
#define STATUS_ADDRESS UINT8_C(0xF3)   ///< status register address

#define CONFIG_ADDRESS UINT8_C(0xF5)     ///< config register address
#define CTRL_MEAS_ADDRESS UINT8_C(0xF4)  ///< ctrl_meas register address
#define CTRL_HUM_ADDRESS UINT8_C(0xF2)   ///< ctrl_hum register address

#define DATA_ADDRESS UINT8_C(0xF7)  ///< data registers address
#define CALIB_1_ADDRESS \
    UINT8_C(0x88)  ///< 1st of 2 calibration registers address
#define CALIB_2_ADDRESS \
    UINT8_C(0xE1)  ///< 2nd of 2 calibration registers address

/// @}

/// @name register length
/// @{

#define DATA_LENGTH (8)  ///< data registers address

#define CALIB_1_LENGHT (25)  ///< 1st of 2 calibration registers length
#define CALIB_2_LENGTH (7)   ///< 2nd of 2 calibration registers length

/// @}

/// @name bit mask for settings
/// @{

#define OSR_TEMP_MASK UINT8_C(0xE0)      ///< bit mask for bme280_osr_temp_t
#define OSR_PRES_MASK UINT8_C(0x1C)      ///< bit mask for bme280_osr_pres_t
#define OSR_HUM_MASK UINT8_C(0x07)       ///< bit mask for bme280_osr_hum_t
#define STANDBY_TIME_MASK UINT8_C(0xE0)  ///< bit mask for bme280_standby_time_t
#define FILTER_MASK UINT8_C(0x1C)        ///< bit mask for bme280_filter_t
#define MODE_MASK UINT8_C(0x03)          ///< bit mask for bme280_mode_t

/// @}

/// @name bit offset for settings
/// @{

#define OSR_TEMP_OFFSET (5)      ///< bit offset of bme280_osr_temp_t
#define OSR_PRES_OFFSET (2)      ///< bit offset of bme280_osr_pres_t
#define OSR_HUM_OFFSET (0)       ///< bit offset of bme280_osr_hum_t
#define STANDBY_TIME_OFFSET (5)  ///< bit offset of bme280_standby_time_t
#define FILTER_OFFSET (3)        ///< bit offset of bme280_filter_t
#define MODE_OFFSET (0)          ///< bit offset of bme280_mode_t

/// @}

/// @name bit mask for status
/// @{

#define STATUS_MEASURING UINT8_C(0x08)  ///< status measuring
#define STATUS_IM_UPDATE UINT8_C(0x01)  ///< status im_update

/// @}

/// @name command constant
/// @{

#define RESET_COMMAND UINT8_C(0xB6)  ///< reset command

/// @}

/// @name bme280 constant
/// @{

#define CHIP_ID UINT8_C(0x60)  ///< bme280 chip id

/// @}

/// @name decimal measurement limit of the sensor
/// @{

#define TEMPERATURE_MAX (85.0)   ///< maximum measurable temperature
#define TEMPERATURE_MIN (-40.0)  ///< minimum measurable temperature
#define PRESSURE_MAX (1100.0)    ///< maximum measurable pressure
#define PRESSURE_MIN (300.0)     ///< minimum measurable pressure
#define HUMIDITY_MAX (100.0)     ///< maximum measurable humidity
#define HUMIDITY_MIN (0.0)       ///< minimum measurable humidity

/// @}

/// @name integer measurement limit of the sensor
/// @{

#define TEMPERATURE_INT_MAX (8500)   ///< maximum measurable temperature
#define TEMPERATURE_INT_MIN (-4000)  ///< minimum measurable temperature
#define PRESSURE_INT_MAX (11000000)  ///< maximum measurable pressure
#define PRESSURE_INT_MIN (3000000)   ///< minimum measurable pressure
#define HUMIDITY_INT_MAX (102400)    ///< maximum measurable humidity

/// @}

/// @name private functions
/// @{

/**
 * @brief writes the given data to the register address of the sensor
 * @param[in] hndl handle for the spi device
 * @param[in] addr the register address to write
 * @param[in] data the data to write to the register
 * @return the status code returned by pigpio library
 */
static int bme280_set_reg(unsigned hndl, uint8_t addr, uint8_t data);

/**
 * @brief reads the data from the register address of the sensor
 * @param[in] hndl handle for the spi device
 * @param[in] addr the register address to read
 * @param[in] len the data length to read
 * @param[out] data the data from the register
 * @return the status code returned by pigpio library
 */
static int bme280_get_reg(unsigned hndl, uint8_t addr, uint8_t len,
                          uint8_t* data);

/**
 * @brief compensate the temperature data from raw_data
 * @param[in] raw_data raw data
 * @param[in] calib_data calibration coefficient data
 * @param[out] t_fine the intermediate temperature coefficient
 * @return compensated temperature data
 */
static double bme280_compensate_temperature(
    const bme280_raw_data_t* raw_data, const bme280_calib_data_t* calib_data,
    int32_t* t_fine);

/**
 * @brief compensate the pressure data from raw_data
 * @param[in] raw_data raw data
 * @param[in] calib_data calibration coefficient data
 * @param[in] t_fine the intermediate temperature coefficient
 * @return compensated pressure data
 */
static double bme280_compensate_pressure(const bme280_raw_data_t* raw_data,
                                         const bme280_calib_data_t* calib_data,
                                         const int32_t* t_fine);

/**
 * @brief compensate the humidity data from raw_data
 * @param[in] raw_data raw data
 * @param[in] calib_data calibration coefficient data
 * @param[in] t_fine the intermediate temperature coefficient
 * @return compensated humidity data
 */
static double bme280_compensate_humidity(const bme280_raw_data_t* raw_data,
                                         const bme280_calib_data_t* calib_data,
                                         const int32_t* t_fine);

/**
 * @brief compensate the temperature data from raw_data
 * @param[in] raw_data raw data
 * @param[in] calib_data calibration coefficient data
 * @param[out] t_fine the intermediate temperature coefficient
 * @return compensated temperature data
 */
static int32_t bme280_compensate_temperature_int(
    const bme280_raw_data_t* raw_data, const bme280_calib_data_t* calib_data,
    int32_t* t_fine);

/**
 * @brief compensate the pressure data from raw_data
 * @param[in] raw_data raw data
 * @param[in] calib_data calibration coefficient data
 * @param[in] t_fine the intermediate temperature coefficient
 * @return compensated pressure data
 */
static uint32_t bme280_compensate_pressure_int(
    const bme280_raw_data_t* raw_data, const bme280_calib_data_t* calib_data,
    const int32_t* t_fine);

/**
 * @brief compensate the humidity data from raw_data
 * @param[in] raw_data raw data
 * @param[in] calib_data calibration coefficient data
 * @param[in] t_fine the intermediate temperature coefficient
 * @return compensated humidity data
 */
static uint32_t bme280_compensate_humidity_int(
    const bme280_raw_data_t* raw_data, const bme280_calib_data_t* calib_data,
    const int32_t* t_fine);

/// @}

int bme280_reset(unsigned int hndl) {
    return bme280_set_reg(hndl, RESET_ADDRESS, RESET_COMMAND);
}

int bme280_get_chip_id(unsigned int hndl, uint8_t* chip_id) {
    if (chip_id == NULL) {
        return 1;
    }
    int ret = 0;
    ret = bme280_get_reg(hndl, CHIP_ID_ADDRESS, 1, chip_id);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

bool bme280_chip_id_valid(uint8_t chip_id) {
    return chip_id == CHIP_ID;
}

int bme280_get_status(unsigned int hndl, uint8_t* status) {
    if (status == NULL) {
        return 1;
    }
    int ret = 0;
    ret = bme280_get_reg(hndl, STATUS_ADDRESS, 1, status);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

bool bme280_status_measuring(uint8_t status) {
    return status & STATUS_MEASURING;
}

bool bme280_status_im_update(uint8_t status) {
    return status & STATUS_IM_UPDATE;
}

int bme280_get_calib_data(unsigned int hndl, bme280_calib_data_t* calib_data) {
    if (calib_data == NULL) {
        return 1;
    }
    int ret = 0;
    uint8_t buf[CALIB_1_LENGHT + CALIB_2_LENGTH];
    ret = bme280_get_reg(hndl, CALIB_1_ADDRESS, CALIB_1_LENGHT, buf);
    if (ret < 0) {
        return ret;
    }
    ret = bme280_get_reg(hndl, CALIB_2_ADDRESS, CALIB_2_LENGTH,
                         buf + CALIB_1_LENGHT);
    if (ret < 0) {
        return ret;
    }
    calib_data->dig_t1 = (uint16_t)buf[1] << 8 | buf[0];
    calib_data->dig_t2 = (int16_t)(buf[3] << 8 | buf[2]);
    calib_data->dig_t3 = (int16_t)(buf[5] << 8 | buf[4]);
    calib_data->dig_p1 = (uint16_t)buf[7] << 8 | buf[6];
    calib_data->dig_p2 = (int16_t)(buf[9] << 8 | buf[8]);
    calib_data->dig_p3 = (int16_t)(buf[11] << 8 | buf[10]);
    calib_data->dig_p4 = (int16_t)(buf[13] << 8 | buf[12]);
    calib_data->dig_p5 = (int16_t)(buf[15] << 8 | buf[14]);
    calib_data->dig_p6 = (int16_t)(buf[17] << 8 | buf[16]);
    calib_data->dig_p7 = (int16_t)(buf[19] << 8 | buf[18]);
    calib_data->dig_p8 = (int16_t)(buf[21] << 8 | buf[20]);
    calib_data->dig_p9 = (int16_t)(buf[23] << 8 | buf[22]);
    calib_data->dig_h1 = buf[24];
    calib_data->dig_h2 = (int16_t)(buf[26] << 8 | buf[25]);
    calib_data->dig_h3 = buf[27];
    calib_data->dig_h4 = (int16_t)(buf[28] << 4 | (buf[29] & 0x0F));
    calib_data->dig_h5 = (int16_t)(buf[30] << 4 | buf[29] >> 4);
    calib_data->dig_h6 = (int8_t)buf[31];
    return 0;
}

int bme280_set_settings(unsigned int hndl, const bme280_settings_t* settings) {
    if (settings == NULL) {
        return 1;
    }
    int ret = 0;
    uint8_t buf[3];
    ret = bme280_get_reg(hndl, CONFIG_ADDRESS, 1, buf);
    if (ret < 0) {
        return ret;
    }
    ret = bme280_get_reg(hndl, CTRL_MEAS_ADDRESS, 1, buf + 1);
    if (ret < 0) {
        return ret;
    }
    ret = bme280_get_reg(hndl, CTRL_HUM_ADDRESS, 1, buf + 2);
    if (ret < 0) {
        return ret;
    }
    buf[0] = (buf[0] & ~STANDBY_TIME_MASK) | settings->standby_time
                                                 << STANDBY_TIME_OFFSET;
    buf[0] = (buf[0] & ~FILTER_MASK) | settings->filter << FILTER_OFFSET;
    buf[1] = (buf[1] & ~OSR_TEMP_MASK) | settings->osr_temp << OSR_TEMP_OFFSET;
    buf[1] = (buf[1] & ~OSR_PRES_MASK) | settings->osr_pres << OSR_PRES_OFFSET;
    buf[2] = (buf[2] & ~OSR_HUM_MASK) | settings->osr_hum << OSR_HUM_OFFSET;
    ret = bme280_set_reg(hndl, CONFIG_ADDRESS, buf[0]);
    if (ret < 0) {
        return ret;
    }
    ret = bme280_set_reg(hndl, CTRL_MEAS_ADDRESS, buf[1]);
    if (ret < 0) {
        return ret;
    }
    ret = bme280_set_reg(hndl, CTRL_HUM_ADDRESS, buf[2]);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

int bme280_get_settings(unsigned int hndl, bme280_settings_t* settings) {
    if (settings == NULL) {
        return 1;
    }
    int ret = 0;
    uint8_t buf[3];
    ret = bme280_get_reg(hndl, CONFIG_ADDRESS, 1, buf);
    if (ret < 0) {
        return ret;
    }
    ret = bme280_get_reg(hndl, CTRL_MEAS_ADDRESS, 1, buf + 1);
    if (ret < 0) {
        return ret;
    }
    ret = bme280_get_reg(hndl, CTRL_HUM_ADDRESS, 1, buf + 2);
    if (ret < 0) {
        return ret;
    }
    settings->standby_time =
        (buf[0] & STANDBY_TIME_MASK) >> STANDBY_TIME_OFFSET;
    settings->filter = (buf[0] & FILTER_MASK) >> FILTER_OFFSET;
    settings->osr_temp = (buf[1] & OSR_TEMP_MASK) >> OSR_TEMP_OFFSET;
    settings->osr_pres = (buf[1] & OSR_PRES_MASK) >> OSR_PRES_OFFSET;
    settings->osr_hum = (buf[2] & OSR_HUM_MASK) >> OSR_HUM_OFFSET;
    return 0;
}

int bme280_set_mode(unsigned int hndl, bme280_mode_t mode) {
    int ret = 0;
    uint8_t buf = 0;
    ret = bme280_get_reg(hndl, CTRL_MEAS_ADDRESS, 1, &buf);
    if (ret < 0) {
        return ret;
    }
    buf = (buf & ~MODE_MASK) | mode << MODE_OFFSET;
    ret = bme280_set_reg(hndl, CTRL_MEAS_ADDRESS, buf);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

int bme280_get_mode(unsigned int hndl, bme280_mode_t* mode) {
    if (mode == NULL) {
        return 1;
    }
    int ret = 0;
    uint8_t buf = 0;
    ret = bme280_get_reg(hndl, CTRL_MEAS_ADDRESS, 1, &buf);
    if (ret < 0) {
        return ret;
    }
    *mode = (buf & MODE_MASK) >> MODE_OFFSET;
    return 0;
}

int bme280_get_raw_data(unsigned int hndl, bme280_raw_data_t* raw_data) {
    if (raw_data == NULL) {
        return 1;
    }
    int ret = 0;
    uint8_t buf[DATA_LENGTH];
    ret = bme280_get_reg(hndl, DATA_ADDRESS, DATA_LENGTH, buf);
    if (ret < 0) {
        return ret;
    }
    raw_data->pressure = buf[0] << 12 | buf[1] << 4 | buf[2] >> 4;
    raw_data->temperature = buf[3] << 12 | buf[4] << 4 | buf[5] >> 4;
    raw_data->humidity = buf[6] << 8 | buf[7];
    return 0;
}

int bme280_compensate_data(const bme280_raw_data_t* raw_data,
                           const bme280_calib_data_t* calib_data,
                           bme280_data_t* data) {
    if (raw_data == NULL || calib_data == NULL || data == NULL) {
        return 1;
    }
    int32_t t_fine = 0;
    data->temperature =
        bme280_compensate_temperature(raw_data, calib_data, &t_fine);
    data->pressure = bme280_compensate_pressure(raw_data, calib_data, &t_fine);
    data->humidity = bme280_compensate_humidity(raw_data, calib_data, &t_fine);
    return 0;
}

int bme280_compensate_data_int(const bme280_raw_data_t* raw_data,
                               const bme280_calib_data_t* calib_data,
                               bme280_int_data_t* data) {
    if (raw_data == NULL || calib_data == NULL || data == NULL) {
        return 1;
    }
    int32_t t_fine = 0;
    data->temperature =
        bme280_compensate_temperature_int(raw_data, calib_data, &t_fine);
    data->pressure =
        bme280_compensate_pressure_int(raw_data, calib_data, &t_fine);
    data->humidity =
        bme280_compensate_humidity_int(raw_data, calib_data, &t_fine);
    return 0;
}

static int bme280_set_reg(unsigned hndl, uint8_t addr, uint8_t data) {
    int ret = 0;
    uint8_t buf[2];
    buf[0] = addr & 0x7F;
    buf[1] = data;
    ret = spiWrite(hndl, (char*)buf, 2);
    return ret;
}

static int bme280_get_reg(unsigned hndl, uint8_t addr, uint8_t len,
                          uint8_t* data) {
    int ret = 0;
    uint8_t* tx_buf = (uint8_t*)malloc(sizeof(uint8_t) * (len + 1));
    uint8_t* rx_buf = (uint8_t*)malloc(sizeof(uint8_t) * (len + 1));
    tx_buf[0] = addr | 0x80;
    memset(tx_buf + 1, 0x00, sizeof(uint8_t) * len);
    ret = spiXfer(hndl, (char*)tx_buf, (char*)rx_buf, len + 1);
    memcpy(data, rx_buf + 1, sizeof(uint8_t) * len);
    free(tx_buf);
    free(rx_buf);
    return ret;
}

static double bme280_compensate_temperature(
    const bme280_raw_data_t* raw_data, const bme280_calib_data_t* calib_data,
    int32_t* t_fine) {
    double var1 = 0;
    double var2 = 0;
    double temperature = 0;
    var1 = ((double)raw_data->temperature) / 16384.0 -
           ((double)calib_data->dig_t1) / 1024.0;
    var1 = var1 * ((double)calib_data->dig_t2);
    var2 = (((double)raw_data->temperature) / 131072.0 -
            ((double)calib_data->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_t3);
    *t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;
    if (temperature < TEMPERATURE_MIN) {
        temperature = TEMPERATURE_MIN;
    } else if (temperature > TEMPERATURE_MAX) {
        temperature = TEMPERATURE_MAX;
    }
    return temperature;
}

static double bme280_compensate_pressure(const bme280_raw_data_t* raw_data,
                                         const bme280_calib_data_t* calib_data,
                                         const int32_t* t_fine) {
    double var1 = 0;
    double var2 = 0;
    double var3 = 0;
    double pressure = 0;
    var1 = ((double)*t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_data->dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data->dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data->dig_p4) * 65536.0);
    var1 = ((((double)calib_data->dig_p3) * var1 * var1 / 524288.0) +
            ((double)calib_data->dig_p2) * var1) /
           524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_p1);
    if (var1 > 0.0) {
        var3 = 1048576.0 - (double)raw_data->pressure;
        var3 = (var3 - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data->dig_p9) * var3 * var3 / 2147483648.0;
        var2 = var3 * ((double)calib_data->dig_p8) / 32768.0;
        pressure =
            (var3 + (var1 + var2 + ((double)calib_data->dig_p7)) / 16.0) / 100;
        if (pressure < PRESSURE_MIN) {
            pressure = PRESSURE_MIN;
        } else if (pressure > PRESSURE_MAX) {
            pressure = PRESSURE_MAX;
        }
    } else {
        pressure = PRESSURE_MIN;
    }
    return pressure;
}

static double bme280_compensate_humidity(const bme280_raw_data_t* raw_data,
                                         const bme280_calib_data_t* calib_data,
                                         const int32_t* t_fine) {
    double var1 = 0;
    double var2 = 0;
    double var3 = 0;
    double var4 = 0;
    double var5 = 0;
    double var6 = 0;
    double humidity = 0;
    var1 = ((double)*t_fine) - 76800.0;
    var2 = (((double)calib_data->dig_h4) * 64.0 +
            (((double)calib_data->dig_h5) / 16384.0) * var1);
    var3 = raw_data->humidity - var2;
    var4 = ((double)calib_data->dig_h2) / 65536.0;
    var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);
    if (humidity < HUMIDITY_MIN) {
        humidity = HUMIDITY_MIN;
    } else if (humidity > HUMIDITY_MAX) {
        humidity = HUMIDITY_MAX;
    }
    return humidity;
}

static int32_t bme280_compensate_temperature_int(
    const bme280_raw_data_t* raw_data, const bme280_calib_data_t* calib_data,
    int32_t* t_fine) {
    int32_t var1 = 0;
    int32_t var2 = 0;
    int32_t temperature = 0;
    var1 = ((int32_t)(raw_data->temperature / 8)) -
           ((int32_t)calib_data->dig_t1 * 2);
    var1 = (var1 * ((int32_t)calib_data->dig_t2)) / 2048;
    var2 =
        (int32_t)((raw_data->temperature / 16) - ((int32_t)calib_data->dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_t3)) / 16384;
    *t_fine = var1 + var2;
    temperature = (*t_fine * 5 + 128) / 256;
    if (temperature < TEMPERATURE_INT_MIN) {
        temperature = TEMPERATURE_INT_MIN;
    } else if (temperature > TEMPERATURE_INT_MAX) {
        temperature = TEMPERATURE_INT_MAX;
    }
    return temperature;
}

static uint32_t bme280_compensate_pressure_int(
    const bme280_raw_data_t* raw_data, const bme280_calib_data_t* calib_data,
    const int32_t* t_fine) {
    int64_t var1 = 0;
    int64_t var2 = 0;
    int64_t var3 = 0;
    uint32_t pressure = 0;
    var1 = ((int64_t)*t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data->dig_p6;
    var2 = var2 + ((var1 * (int64_t)calib_data->dig_p5) * 131072);
    var2 = var2 + (((int64_t)calib_data->dig_p4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)calib_data->dig_p3) / 256) +
           ((var1 * ((int64_t)calib_data->dig_p2) * 4096));
    var1 = ((((int64_t)1) * 140737488355328) + var1) *
           ((int64_t)calib_data->dig_p1) / 8589934592;
    if (var1 != 0) {
        var3 = 1048576 - raw_data->pressure;
        var3 = (((var3 * 2147483648) - var2) * 3125) / var1;
        var1 = (((int64_t)calib_data->dig_p9) * (var3 / 8192) * (var3 / 8192)) /
               33554432;
        var2 = (((int64_t)calib_data->dig_p8) * var3) / 524288;
        var3 =
            ((var3 + var1 + var2) / 256) + (((int64_t)calib_data->dig_p7) * 16);
        pressure = (uint32_t)(((var3 / 2) * 100) / 128);
        if (pressure < PRESSURE_INT_MIN) {
            pressure = PRESSURE_INT_MIN;
        } else if (pressure > PRESSURE_INT_MAX) {
            pressure = PRESSURE_INT_MAX;
        }
    } else {
        pressure = PRESSURE_INT_MIN;
    }
    return pressure;
}

static uint32_t bme280_compensate_humidity_int(
    const bme280_raw_data_t* raw_data, const bme280_calib_data_t* calib_data,
    const int32_t* t_fine) {
    int32_t var1 = 0;
    int32_t var2 = 0;
    int32_t var3 = 0;
    int32_t var4 = 0;
    int32_t var5 = 0;
    uint32_t humidity = 0;
    var1 = *t_fine - ((int32_t)76800);
    var2 = (int32_t)(raw_data->humidity * 16384);
    var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
    var4 = ((int32_t)calib_data->dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);
    if (humidity > HUMIDITY_INT_MAX) {
        humidity = HUMIDITY_INT_MAX;
    }
    return humidity;
}
