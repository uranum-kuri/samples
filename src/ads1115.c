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
 * @file ads1115.c
 * @brief ads1115 driver source file
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pigpio.h"

#include "ads1115.h"

/// @name register address
/// @{

#define CONVERSION_ADDRESS UINT8_C(0x00)  ///< conversion register address
#define CONFIG_ADDRESS UINT8_C(0x01)      ///< config register address
#define LO_THRESH_ADDRESS UINT8_C(0x02)   ///< lo_thresh register address
#define HI_THRESH_ADDRESS UINT8_C(0x03)   ///< hi_thresh register address

/// @}

/// @name bit mask for config register
/// @{

#define OS_MASK UINT16_C(0x8000)         ///< bit mask for OS field
#define MUX_MASK UINT16_C(0x7000)        ///< bit mask for MUX field
#define PGA_MASK UINT16_C(0x0E00)        ///< bit mask for PGA field
#define MODE_MASK UINT16_C(0x0100)       ///< bit mask for MODE field
#define DR_MASK UINT16_C(0x00E0)         ///< bit mask for DR field
#define COMP_MODE_MASK UINT16_C(0x0010)  ///< bit mask for COMP_MODE field
#define COMP_POL_MASK UINT16_C(0x0008)   ///< bit mask for COMP_POL field
#define COMP_LAT_MASK UINT16_C(0x0004)   ///< bit mask for COMP_LAT field
#define COMP_QUE_MASK UINT16_C(0x0003)   ///< bit mask for COMP_QUE field

/// @}

/// @name bit offset for config register
/// @{

#define OS_OFFSET (15)        ///< bit offset of OS field
#define MUX_OFFSET (12)       ///< bit offset of MUX field
#define PGA_OFFSET (9)        ///< bit offset of PGA field
#define MODE_OFFSET (8)       ///< bit offset of MODE field
#define DR_OFFSET (5)         ///< bit offset of DR field
#define COMP_MODE_OFFSET (4)  ///< bit offset of COMP_MODE field
#define COMP_POL_OFFSET (3)   ///< bit offset of COMP_POL field
#define COMP_LAT_OFFSET (2)   ///< bit offset of COMP_LAT field
#define COMP_QUE_OFFSET (0)   ///< bit offset of COMP_QUE field

/// @}

/// @name LSB size compatible with FSR
/// @{

#define GAIN_6P144_LSB (187.5)   ///< 6.144v FSR
#define GAIN_4P096_LSB (125.0)   ///< 4.096v FSR
#define GAIN_2P048_LSB (62.5)    ///< 2.048v FSR
#define GAIN_1P024_LSB (31.25)   ///< 1.024v FSR
#define GAIN_0P512_LSB (15.625)  ///< 0.512v FSR
#define GAIN_0P256_LSB (7.8125)  ///< 0.256v FSR

/// @}

/// @name private function
/// @{

/**
 * @brief writes the given data to the register address of the sensor
 * @param[in] hndl handle for the i2c device
 * @param[in] addr the register address to write
 * @param[in] data the data to write to the register
 * @return the status code returned by pigpio library
 */
static int ads1115_set_reg(unsigned int hndl, uint8_t addr, uint16_t data);

/**
 * @brief reads the data from the register address of the sensor
 * @param[in] hndl handle for the i2c device
 * @param[in] addr the register address to read
 * @param[out] data the data from the register
 * @return the status code returned by pigpio library
 */
static int ads1115_get_reg(unsigned int hndl, uint8_t addr, uint16_t* data);

/// @}

int ads1115_start_conversion(unsigned int hndl) {
    int ret = 0;
    uint16_t buf = 0;
    ret = ads1115_get_reg(hndl, CONFIG_ADDRESS, &buf);
    if (ret < 0) {
        return ret;
    }
    ret = ads1115_set_reg(hndl, CONFIG_ADDRESS, buf | OS_MASK);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

int ads1115_get_status(unsigned int hndl, bool* status) {
    if (status == NULL) {
        return 1;
    }
    int ret = 0;
    uint16_t buf = 0;
    ret = ads1115_get_reg(hndl, CONFIG_ADDRESS, &buf);
    if (ret < 0) {
        return ret;
    }
    *status = buf & OS_MASK;
    return 0;
}

int ads1115_set_settings(unsigned int hndl,
                         const ads1115_settings_t* settings) {
    if (settings == NULL) {
        return 1;
    }
    int ret = 0;
    uint16_t buf = 0;
    buf |= settings->mode << MODE_OFFSET;
    buf |= settings->gain << PGA_OFFSET;
    buf |= settings->rate << DR_OFFSET;
    buf |= settings->channel << MUX_OFFSET;
    buf |= settings->pin_polarity << COMP_POL_OFFSET;
    buf |= settings->comparator_latch << COMP_LAT_OFFSET;
    buf |= settings->comparator_mode << COMP_MODE_OFFSET;
    buf |= settings->comparator_queue << COMP_QUE_OFFSET;
    ads1115_set_reg(hndl, CONFIG_ADDRESS, buf);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

int ads1115_get_settings(unsigned int hndl, ads1115_settings_t* settings) {
    if (settings == NULL) {
        return 1;
    }
    int ret = 0;
    uint16_t buf = 0;
    // settings->osr_hum = (buf[2] & OSR_HUM_MASK) >> OSR_HUM_OFFSET;
    ret = ads1115_get_reg(hndl, CONFIG_ADDRESS, &buf);
    if (ret < 0) {
        return ret;
    }
    settings->mode = (buf & MODE_MASK) >> MODE_OFFSET;
    settings->gain = (buf & PGA_MASK) >> PGA_OFFSET;
    settings->rate = (buf & DR_MASK) >> DR_OFFSET;
    settings->channel = (buf & MUX_MASK) >> MUX_OFFSET;
    settings->pin_polarity = (buf & COMP_POL_MASK) >> COMP_POL_OFFSET;
    settings->comparator_latch = (buf & COMP_LAT_MASK) >> COMP_LAT_OFFSET;
    settings->comparator_mode = (buf & COMP_MODE_MASK) >> COMP_MODE_OFFSET;
    settings->comparator_queue = (buf & COMP_QUE_MASK) >> COMP_QUE_OFFSET;
    return 0;
}

int ads1115_set_threshold_settings(
    unsigned int hndl, const ads1115_threshold_settings_t* settings) {
    if (settings == NULL) {
        return 1;
    }
    int ret = 0;
    ret = ads1115_set_reg(hndl, LO_THRESH_ADDRESS, settings->low);
    if (ret < 0) {
        return ret;
    }
    ret = ads1115_set_reg(hndl, HI_THRESH_ADDRESS, settings->high);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

int ads1115_get_threshold_settings(unsigned int hndl,
                                   ads1115_threshold_settings_t* settings) {
    if (settings == NULL) {
        return 1;
    }
    int ret = 0;
    uint16_t buf_low = 0;
    uint16_t buf_high = 0;
    ret = ads1115_get_reg(hndl, LO_THRESH_ADDRESS, &buf_low);
    if (ret < 0) {
        return ret;
    }
    ret = ads1115_get_reg(hndl, HI_THRESH_ADDRESS, &buf_high);
    if (ret < 0) {
        return ret;
    }
    settings->low = (int16_t)buf_low;
    settings->high = (int16_t)buf_high;
    return 0;
}

int ads1115_get_data(unsigned int hndl, int16_t* data) {
    if (data == NULL) {
        return 1;
    }
    int ret = 0;
    uint16_t buf = 0;
    ret = ads1115_get_reg(hndl, CONVERSION_ADDRESS, &buf);
    if (ret < 0) {
        return ret;
    }
    *data = (int16_t)buf;
    return 0;
}

int ads1115_compensate_data(int16_t data, const ads1115_settings_t* settings,
                            double* voltage) {
    if (settings == NULL || voltage == NULL) {
        return 1;
    }
    switch (settings->gain) {
        case ads1115_gain_6p144v:
            *voltage = data * GAIN_6P144_LSB;
            break;
        case ads1115_gain_4p096v:
            *voltage = data * GAIN_4P096_LSB;
            break;
        case ads1115_gain_2p048v:
            *voltage = data * GAIN_2P048_LSB;
            break;
        case ads1115_gain_1p024v:
            *voltage = data * GAIN_1P024_LSB;
            break;
        case ads1115_gain_0p512v:
            *voltage = data * GAIN_0P512_LSB;
            break;
        case ads1115_gain_0p256v:
            *voltage = data * GAIN_0P256_LSB;
            break;
    }
    return 0;
}

static int ads1115_set_reg(unsigned int hndl, uint8_t addr, uint16_t data) {
    int ret = 0;
    char buf[2] = {data >> 8, data & 0xFF};
    ret = i2cWriteI2CBlockData(hndl, addr, buf, 2);
    return ret;
}

static int ads1115_get_reg(unsigned int hndl, uint8_t addr, uint16_t* data) {
    int ret = 0;
    char buf[2];
    memset(buf, 0x00, sizeof(char) * 2);
    ret = i2cReadI2CBlockData(hndl, addr, buf, 2);
    *data = buf[0] << 8 | buf[1];
    return ret;
}
