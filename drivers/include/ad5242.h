/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ad5242
 * @ingroup     drivers_ad5242
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

#define AD5242_CHN_A                0
#define AD5242_CHN_B                (1 << 7)
#define AD5242_MID_RESET            (1 << 6)
#define AD5242_SHDN                 (1 << 5)
#define AD5242_OUT_1_HIGH           (1 << 4)
#define AD5242_OUT_2_HIGH           (1 << 3)

typedef struct {
	i2c_t i2c;
	uint8_t addr;
	uint8_t opt;
} ad5242_t;

typedef struct {
	i2c_t i2c_dev;
	i2c_speed_t i2c_spd;
	uint8_t addr;
	uint8_t opt;
} ad5242_params_t;

int ad5242_init(ad5242_t *dev, const ad5242_params_t *params);
int ad5242_set_opt(ad5242_t *dev, uint8_t opt);
uint8_t ad5242_get_reg(ad5242_t *dev);
void ad5242_set_reg(ad5242_t *dev, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* AD5242_H */
