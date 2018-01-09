/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_max541x MAX541x driver
 * @ingroup     drivers_actuators
 * @brief       Driver for max5414x.
 * @{
 *
 * @file
 * @brief       Driver for max541x.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#ifndef MAX541X_H
#define MAX541X_H

#include <stdint.h>
#include "periph/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Control bytes for max541x
 * @{
 */
#define MAX541X_VREG        0x11
#define MAX541X_NVREG       0x21
#define MAX541X_NVREGXVREG  0x61
#define MAX541X_VREGXNVREG  0x51
/** @} */

/**
 * @brief ad5242 device
 */
typedef struct {
	i2c_t i2c;       /**< i2c device */
	uint8_t addr;    /**< i2c address of the ad5242 (7-bit) */
} max541x_t;

/**
 * @brief ad5242 configuration parameters
 */
typedef struct {
	i2c_t i2c_dev;        /**< i2c device */
	i2c_speed_t i2c_spd;  /**< i2c speed */
	uint8_t addr;         /**< i2c address of the ad5242 */
} max541x_params_t;

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
int max541x_init(max541x_t *dev, const max541x_params_t *params);

/**
 * Set the value of the register selected in the options
 *
 * @param[in] dev device descriptor
 * @param[in] data value to write into the selected register
 */
int max541x_set_reg(max541x_t *dev, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* MAX541X_H */
