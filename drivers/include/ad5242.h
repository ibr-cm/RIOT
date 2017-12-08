/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ad5242 AD5242 driver
 * @ingroup     drivers_actuators
 * @brief       Driver for ad5242.
 * @{
 *
 * @file
 * @brief       Driver for ad5242.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#ifndef AD5242_H
#define AD5242_H

#include <stdint.h>
#include "periph/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Options for ad5242
 * @brief Combine with OR to form the options byte
 * @{
 */
#define AD5242_CHN_A                0           /**< Use RDAC Register 1 */
#define AD5242_CHN_B                (1 << 7)    /**< Use RDAC Register 2 */
#define AD5242_MID_RESET            (1 << 6)    /**< Midscale reset */
#define AD5242_SHDN                 (1 << 5)    /**< Shutdown */
#define AD5242_OUT_1_HIGH           (1 << 4)    /**< Set output pin 1 to HIGH (default: LOW) */
#define AD5242_OUT_2_HIGH           (1 << 3)    /**< Set output pin 2 to HIGH (default: LOW) */
/** @} */

/**
 * @brief ad5242 device
 */
typedef struct {
	i2c_t i2c;       /**< i2c device */
	uint8_t addr;    /**< i2c address of the ad5242 (7-bit) */
	uint8_t opt;     /**< options (instruction byte sent with every write) */
} ad5242_t;

/**
 * @brief ad5242 configuration parameters
 */
typedef struct {
	i2c_t i2c_dev;        /**< i2c device */
	i2c_speed_t i2c_spd;  /**< i2c speed */
	uint8_t addr;         /**< i2c address of the ad5242 */
	uint8_t opt;          /**< options (instruction byte sent with every write) */
} ad5242_params_t;

/**
 * Initialize ad5242
 *
 * This will immediately write the params->opt byte to the ad5242.
 *
 * @param[out] dev device descriptor
 * @param[in] params configuration parameters
 *
 * @return 0 on success
 */
int ad5242_init(ad5242_t *dev, const ad5242_params_t *params);

/**
 * Set the options byte and write it to the ad5242
 *
 * @param[in] dev device descriptor
 * @param[in] opt options to write
 *
 * @return 0 on success
 */
int ad5242_set_opt(ad5242_t *dev, uint8_t opt);

/**
 * Get the value of the register selected in the options
 *
 * @param[in] dev device descriptor
 *
 * @return value read from the selected register
 */
uint8_t ad5242_get_reg(ad5242_t *dev);

/**
 * Set the value of the register selected in the options
 *
 * @param[in] dev device descriptor
 * @param[in] data value to write into the selected register
 */
void ad5242_set_reg(ad5242_t *dev, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* AD5242_H */
