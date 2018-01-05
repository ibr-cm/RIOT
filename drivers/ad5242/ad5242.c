/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ad5242
 * @{
 *
 * @file
 * @brief       Driver for ad5242.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include "ad5242.h"
#include <assert.h>

int ad5242_init(ad5242_t *dev, const ad5242_params_t *params)
{
	dev->i2c = params->i2c_dev;
	dev->addr = params->addr;
	assert((dev->addr & 0x7c) == 0x2c);
	dev->opt = params->opt;
	if(i2c_init_master(dev->i2c, params->i2c_spd) != 0)
		return -1;
	i2c_acquire(dev->i2c);
	if(i2c_write_byte(dev->i2c, dev->addr, dev->opt) != 1) {
		i2c_release(dev->i2c);
		return -1;
	}
	i2c_release(dev->i2c);
	return 0;
}

int ad5242_set_opt(ad5242_t *dev, uint8_t opt)
{
	dev->opt = opt;
	i2c_acquire(dev->i2c);
	if(i2c_write_byte(dev->i2c, dev->addr, dev->opt) != 1) {
		i2c_release(dev->i2c);
		return -1;
	}
	i2c_release(dev->i2c);
	return 0;
}

uint8_t ad5242_get_reg(ad5242_t *dev)
{
	uint8_t data;
	i2c_acquire(dev->i2c);
	i2c_read_byte(dev->i2c, dev->addr, &data);
	i2c_release(dev->i2c);
	return data;
}

void ad5242_set_reg(ad5242_t *dev, uint8_t value)
{
	i2c_acquire(dev->i2c);
	i2c_write_reg(dev->i2c, dev->addr, dev->opt, value);
	i2c_release(dev->i2c);
}
