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
 * @file ads1115.h
 * @brief ads1115 driver header file
 */

#ifndef ADS1115_H
#define ADS1115_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief device operating mode
 */
typedef enum {
    ads1115_mode_continuous = 0x00,   ///< continuous conversion mode
    ads1115_mode_single_shot = 0x01,  ///< single shot mode or power down state
} ads1115_mode_t;

/**
 * @brief programmable gain amplifier configuration
 */
typedef enum {
    ads1115_gain_6p144v = 0x00,  ///< 6.144v range
    ads1115_gain_4p096v = 0x01,  ///< 4.096v range
    ads1115_gain_2p048v = 0x02,  ///< 2.048v range
    ads1115_gain_1p024v = 0x03,  ///< 1.024v range
    ads1115_gain_0p512v = 0x04,  ///< 0.512v range
    ads1115_gain_0p256v = 0x05,  ///< 0.256v range
} ads1115_gain_t;

/**
 * @brief data rate
 */
typedef enum {
    ads1115_rate_8sps = 0x00,    ///< 8 sample per second
    ads1115_rate_16sps = 0x01,   ///< 16 sample per second
    ads1115_rate_32sps = 0x02,   ///< 32 sample per second
    ads1115_rate_64sps = 0x03,   ///< 64 sample per second
    ads1115_rate_128sps = 0x04,  ///< 128 sample per second
    ads1115_rate_250sps = 0x05,  ///< 250 sample per second
    ads1115_rate_475sps = 0x06,  ///< 475 sample per second
    ads1115_rate_860sps = 0x07,  ///< 860 sample per second
} ads1115_rate_t;

/**
 * @brief input multiplexer configuration
 */
typedef enum {
    ads1115_channel_ain0_ain1 = 0x00,  ///< ain0 and ain1 pins
    ads1115_channel_ain0_ain3 = 0x01,  ///< ain0 and ain3 pins
    ads1115_channel_ain1_ain3 = 0x02,  ///< ain1 and ain3 pins
    ads1115_channel_ain2_ain3 = 0x03,  ///< ain2 and ain3 pins
    ads1115_channel_ain0_gnd = 0x04,   ///< ain0 and gnd pins
    ads1115_channel_ain1_gnd = 0x05,   ///< ain1 and gnd pins
    ads1115_channel_ain2_gnd = 0x06,   ///< ain2 and gnd pins
    ads1115_channel_ain3_gnd = 0x07,   ///< ain3 and gnd pins
} ads1115_channel_t;

/**
 * @brief polarity of the ALERT/RDY pin
 */
typedef enum {
    ads1115_pin_polarity_active_low = 0x00,   ///< active low
    ads1115_pin_polarity_active_high = 0x01,  ///< active low
} ads1115_pin_polarity_t;

/**
 * @brief latching comparator
 */
typedef enum {
    ads1115_comparator_latch_disable = 0x00,  ///< nonlatching ALERT/RDY pin
    ads1115_comparator_latch_enable = 0x01,   ///< latching ALERT/RDY pin
} ads1115_comparator_latch_t;

/**
 * @brief comparator mode
 */
typedef enum {
    ads1115_comparator_mode_traditional = 0x00,  ///< traditional comparator
    ads1115_comparator_mode_window = 0x01,       ///< window comparator
} ads1115_comparator_mode_t;

/**
 * @brief comparator queue
 */
typedef enum {
    ads1115_comparator_queue_1 = 0x00,        ///< assert after 1 conversion
    ads1115_comparator_queue_2 = 0x01,        ///< assert after 2 conversion
    ads1115_comparator_queue_4 = 0x02,        ///< assert after 4 conversion
    ads1115_comparator_queue_disable = 0x03,  ///< disable comparator and
                                              ///< set pin to high impedance
} ads1115_comparator_queue_t;

/**
 * @brief sensor settings
 */
typedef struct {
    ads1115_mode_t mode;        ///< sensor mode
    ads1115_gain_t gain;        ///< the FSR of programmable gain amplifier
    ads1115_rate_t rate;        ///< data rate
    ads1115_channel_t channel;  ///< the channel of input multiplexer
    ads1115_pin_polarity_t pin_polarity;  ///< polarity of the ALERT/RDY pin
    ads1115_comparator_latch_t comparator_latch;  ///< the ALERT/RDY pin latch
    ads1115_comparator_mode_t comparator_mode;    ///< comparator mode
    ads1115_comparator_queue_t comparator_queue;  ///< comparator queue
} ads1115_settings_t;

/**
 * @brief sensor threshold settings
 */
typedef struct {
    int16_t low;   ///< low threshold value
    int16_t high;  ///< high threshold value
} ads1115_threshold_settings_t;

/**
 * @brief start conversion when in power down state
 * @param[in] hndl handle for the i2c device
 * @return the status code returned by pigpio library
 */
int ads1115_start_conversion(unsigned int hndl);

/**
 * @brief get the status of sensor conversion
 * @param[in] hndl handle for the i2c device
 * @param[out] status device conversion state
 * @return the status code returned by pigpio library or 1 if the parameter
 * pointers are NULL
 */
int ads1115_get_status(unsigned int hndl, bool* status);

/**
 * @brief set the sensor settings
 * @param[in] hndl handle for the i2c device
 * @param[in] settings sensor settings
 * @return the status code returned by pigpio library or 1 if the parameter
 * pointers are NULL
 */
int ads1115_set_settings(unsigned int hndl, const ads1115_settings_t* settings);

/**
 * @brief get the sensor settings
 * @param[in] hndl handle for the i2c device
 * @param[out] settings sensor settings
 * @return the status code returned by pigpio library or 1 if the parameter
 * pointers are NULL
 */
int ads1115_get_settings(unsigned int hndl, ads1115_settings_t* settings);

/**
 * @brief set the sensor threshold settings
 * @param[in] hndl handle for the i2c device
 * @param[in] settings sensor threshold settings
 * @return the status code returned by pigpio library or 1 if the parameter
 * pointers are NULL
 */
int ads1115_set_threshold_settings(
    unsigned int hndl, const ads1115_threshold_settings_t* settings);

/**
 * @brief get the sensor threshold settings
 * @param[in] hndl handle for the i2c device
 * @param[out] settings sensor threshold settings
 * @return the status code returned by pigpio library or 1 if the parameter
 * pointers are NULL
 * @note To activate the alert pin, set a positive value for low threshold and a
 * negative value for high threshold.
 */
int ads1115_get_threshold_settings(unsigned int hndl,
                                   ads1115_threshold_settings_t* settings);

/**
 * @brief get the raw data from the sensor
 * @param[in] hndl handle for the i2c device
 * @param[out] data the raw data
 * @return the status code returned by pigpio library or 1 if the parameter
 * pointers are NULL
 */
int ads1115_get_data(unsigned int hndl, int16_t* data);

/**
 * @brief compensate the data from the raw data
 * @param[in] data the raw data
 * @param[in] settings sensor settings
 * @param[out] voltage compensated micro voltage data
 * @return the status code 1 if the parameter pointers are NULL
 */
int ads1115_compensate_data(int16_t data, const ads1115_settings_t* settings,
                            double* voltage);

#ifdef __cplusplus
}
#endif

#endif
