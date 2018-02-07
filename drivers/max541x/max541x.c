/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_max541x
 * @{
 *
 * @file
 * @brief       Driver for max541x.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include "max541x.h"
#include <assert.h>
#include <stdio.h>

int max541x_init(max541x_t *dev, const max541x_params_t *params)
{
	uint8_t res;
	dev->i2c = params->i2c_dev;
	dev->addr = params->addr;
	assert((dev->addr & 0x78) == 0x28);
	i2c_acquire(dev->i2c);
	res = i2c_init_master(dev->i2c, params->i2c_spd);
	i2c_release(dev->i2c);
	return res > 0;
}

int max541x_set_reg(max541x_t *dev, uint8_t value)
{
	uint8_t res;
	i2c_acquire(dev->i2c);
	res = i2c_write_reg(dev->i2c, dev->addr, MAX541X_VREG, value);
	i2c_release(dev->i2c);
	return res - 1;
}
