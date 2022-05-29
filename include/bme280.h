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
 * @file bme280.h
 * @brief bme280 driver header file
 */

#ifndef BME280_H
#define BME280_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief oversampling of temperature data
 */
typedef enum {
    bme280_osr_temp_skip = 0x00,  ///< skipped
    bme280_osr_temp_x1 = 0x01,    ///< oversampling x 1
    bme280_osr_temp_x2 = 0x02,    ///< oversampling x 2
    bme280_osr_temp_x4 = 0x03,    ///< oversampling x 4
    bme280_osr_temp_x8 = 0x04,    ///< oversampling x 8
    bme280_osr_temp_x16 = 0x05,   ///< oversampling x 16
} bme280_osr_temp_t;

/**
 * @brief oversampling of pressure data
 */
typedef enum {
    bme280_osr_pres_skip = 0x00,  ///< skipped
    bme280_osr_pres_x1 = 0x01,    ///< oversampling x 1
    bme280_osr_pres_x2 = 0x02,    ///< oversampling x 2
    bme280_osr_pres_x4 = 0x03,    ///< oversampling x 4
    bme280_osr_pres_x8 = 0x04,    ///< oversampling x 8
    bme280_osr_pres_x16 = 0x05,   ///< oversampling x 16
} bme280_osr_pres_t;

/**
 * @brief oversampling of humidity data
 */
typedef enum {
    bme280_osr_hum_skip = 0x00,  ///< skipped
    bme280_osr_hum_x1 = 0x01,    ///< oversampling x 1
    bme280_osr_hum_x2 = 0x02,    ///< oversampling x 2
    bme280_osr_hum_x4 = 0x03,    ///< oversampling x 4
    bme280_osr_hum_x8 = 0x04,    ///< oversampling x 8
    bme280_osr_hum_x16 = 0x05,   ///< oversampling x 16
} bme280_osr_hum_t;

/**
 * @brief inactive duration standby_time in normal mode
 */
typedef enum {
    bme280_standby_time_0p5ms = 0x00,   ///< 0.5ms
    bme280_standby_time_62p5ms = 0x01,  ///< 62.5ms
    bme280_standby_time_125ms = 0x02,   ///< 125ms
    bme280_standby_time_250ms = 0x03,   ///< 250ms
    bme280_standby_time_500ms = 0x04,   ///< 500ms
    bme280_standby_time_1000ms = 0x05,  ///< 1000ms
    bme280_standby_time_10ms = 0x06,    ///< 10ms
    bme280_standby_time_20ms = 0x07,    ///< 20ms
} bme280_standby_time_t;

/**
 * @brief the time constant of the IIR filter
 */
typedef enum {
    bme280_filter_off = 0x00,  ///< filter off
    bme280_filter_2 = 0x01,    ///< coefficient 2
    bme280_filter_4 = 0x02,    ///< coefficient 4
    bme280_filter_8 = 0x03,    ///< coefficient 8
    bme280_filter_16 = 0x04,   ///< coefficient 16
} bme280_filter_t;

/**
 * @brief the sensor mode of the device
 */
typedef enum {
    bme280_mode_sleep = 0x00,   ///< sleep mode
    bme280_mode_forced = 0x01,  ///< forced mode
    bme280_mode_normal = 0x03,  ///< normal mode
} bme280_mode_t;

/**
 * @brief sensor settings
 */
typedef struct {
    bme280_osr_temp_t osr_temp;          ///< temperature oversampling
    bme280_osr_pres_t osr_pres;          ///< pressure oversampling
    bme280_osr_hum_t osr_hum;            ///< humidity oversampling
    bme280_standby_time_t standby_time;  ///< standby time
    bme280_filter_t filter;              ///< filter coefficient
} bme280_settings_t;

/**
 * @brief calibration coefficient data
 */
typedef struct {
    uint16_t dig_t1;  ///< calibration coefficient for the temperature sensor
    int16_t dig_t2;   ///< calibration coefficient for the temperature sensor
    int16_t dig_t3;   ///< calibration coefficient for the temperature sensor
    uint16_t dig_p1;  ///< calibration coefficient for the pressure sensor
    int16_t dig_p2;   ///< calibration coefficient for the pressure sensor
    int16_t dig_p3;   ///< calibration coefficient for the pressure sensor
    int16_t dig_p4;   ///< calibration coefficient for the pressure sensor
    int16_t dig_p5;   ///< calibration coefficient for the pressure sensor
    int16_t dig_p6;   ///< calibration coefficient for the pressure sensor
    int16_t dig_p7;   ///< calibration coefficient for the pressure sensor
    int16_t dig_p8;   ///< calibration coefficient for the pressure sensor
    int16_t dig_p9;   ///< calibration coefficient for the pressure sensor
    uint8_t dig_h1;   ///< calibration coefficient for the humidity sensor
    int16_t dig_h2;   ///< calibration coefficient for the humidity sensor
    uint8_t dig_h3;   ///< calibration coefficient for the humidity sensor
    int16_t dig_h4;   ///< calibration coefficient for the humidity sensor
    int16_t dig_h5;   ///< calibration coefficient for the humidity sensor
    int8_t dig_h6;    ///< calibration coefficient for the humidity sensor
} bme280_calib_data_t;

/**
 * @brief raw data from register
 */
typedef struct {
    uint32_t temperature;  ///< temperature raw data
    uint32_t pressure;     ///< pressure raw data
    uint16_t humidity;     ///< humidity raw data
} bme280_raw_data_t;

/**
 * @brief compensated decimal data
 */
typedef struct {
    double temperature;  ///< corrected temperature data
    double pressure;     ///< corrected pressure data
    double humidity;     ///< corrected humidity data
} bme280_data_t;

/**
 * @brief compensated integer data
 */
typedef struct {
    uint32_t pressure;    ///< corrected temperature data
    int32_t temperature;  ///< corrected pressure data
    uint32_t humidity;    ///< corrected humidity data
} bme280_int_data_t;

/**
 * @brief soft reset the sensor registers to default values
 * @param[in] hndl handle for the spi device
 * @return the status code returned by pigpio library
 */
int bme280_reset(unsigned int hndl);

/**
 * @brief get the chip ID of the sensor
 * @param[in] hndl handle for the spi device
 * @param[out] chip_id the chip ID of the sensor
 * @return the status code returned by pigpio library or 1 if the chip_id
 * pointer is NULL
 */
int bme280_get_chip_id(unsigned int hndl, uint8_t* chip_id);

/**
 * @brief determine if the chip ID is valid or not
 * @param[in] chip_id chip ID as returned by a call to bme280_get_chip_id
 * @return result
 */
bool bme280_chip_id_valid(uint8_t chip_id);

/**
 * @brief get the status of the sensor
 * @param[in] hndl handle for the spi device
 * @param[out] status status of the sensor
 * @return the status code returned by pigpio library or 1 if the status pointer
 * is NULL
 */
int bme280_get_status(unsigned int hndl, uint8_t* status);

/**
 * @brief determine if the status is measuring or not
 * @param[in] status status as returned by a call to bme280_get_status
 * @return result
 */
bool bme280_status_measuring(uint8_t status);

/**
 * @brief determine if the status is im_update or not
 * @param[in] status status as returned by a call to bme280_get_status
 * @return result
 */
bool bme280_status_im_update(uint8_t status);

/**
 * @brief get the calibration coefficient data
 * @param[in] hndl handle for the spi device
 * @param[out] calib_data calibration coefficient data
 * @return the status code returned by pigpio library or 1 if the calib_data
 * pointer is NULL
 */
int bme280_get_calib_data(unsigned int hndl, bme280_calib_data_t* calib_data);

/**
 * @brief set the sensor settings
 * @param[in] hndl handle for the spi device
 * @param[in] settings sensor settings
 * @return the status code returned by pigpio library or 1 if the settings
 * pointer is NULL
 */
int bme280_set_settings(unsigned int hndl, const bme280_settings_t* settings);

/**
 * @brief get the sensor settings
 * @param[in] hndl handle for the spi device
 * @param[out] settings sensor settings
 * @return the status code retuned by pigpio library or 1 if the settings
 * pointer is NULL
 */
int bme280_get_settings(unsigned int hndl, bme280_settings_t* settings);

/**
 * @brief set the sensor mode
 * @param[in] hndl handle for the spi device
 * @param[in] mode the sensor mode
 * @return the status code returned by pigpio library
 */
int bme280_set_mode(unsigned int hndl, bme280_mode_t mode);

/**
 * @brief get the sensor mode
 * @param[in] hndl handle for the spi device
 * @param[out] mode the sensor mode
 * @return the status code returned by pigpio library or 1 if the mode pointer
 * is NULL
 */
int bme280_get_mode(unsigned int hndl, bme280_mode_t* mode);

/**
 * @brief get raw data from the sensor
 * @param[in] hndl handle for the spi device
 * @param[out] raw_data raw data
 * @return the status code returned by pigpio library or 1 if the raw_data
 * pointer is NULL
 */
int bme280_get_raw_data(unsigned int hndl, bme280_raw_data_t* raw_data);

/**
 * @brief compensate the data from raw_data
 * @param[in] raw_data raw data
 * @param[in] calib_data calibration coefficient data
 * @param[out] data compensated data
 * @return status code 1 if the parameter pointers are NULL
 */
int bme280_compensate_data(const bme280_raw_data_t* raw_data,
                           const bme280_calib_data_t* calib_data,
                           bme280_data_t* data);

/**
 * @brief compensate the data from raw_data
 * @param[in] raw_data raw data
 * @param[in] calib_data calibration coefficient data
 * @param[out] data compensated data
 * @return status code 1 if the parameter pointers are NULL
 */
int bme280_compensate_data_int(const bme280_raw_data_t* raw_data,
                               const bme280_calib_data_t* calib_data,
                               bme280_int_data_t* data);

#ifdef __cplusplus
}
#endif

#endif
