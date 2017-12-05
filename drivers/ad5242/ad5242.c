#include "ad5242.h"
#include <assert.h>

int ad5242_init(ad5242_t *dev, i2c_t i2c, i2c_speed_t speed, uint8_t addr, uint8_t opt)
{
	dev->i2c = i2c;
	dev->addr = addr;
	assert((addr & 0x7c) == 0x2c);
	dev->opt = opt;
	i2c_acquire(dev->i2c);
	if(i2c_init_master(dev->i2c, speed) != 0 || i2c_write_byte(dev->i2c, dev->addr, dev->opt) != 1) {
		i2c_release(dev->i2c);
		return -1;
	}
	i2c_release(dev->i2c);
	return 0;
}

void ad5242_set_opt(ad5242_t *dev, uint8_t opt)
{
	dev->opt = opt;
}

uint8_t ad5242_get_reg(ad5242_t *dev)
{
	uint8_t data;
	i2c_acquire(dev->i2c);
	i2c_read_reg(dev->i2c, dev->addr, dev->opt, &data);
	i2c_release(dev->i2c);
	return data;
}

void ad5242_set_reg(ad5242_t *dev, uint8_t value)
{
	i2c_acquire(dev->i2c);
	i2c_write_reg(dev->i2c, dev->addr, dev->opt, value);
	i2c_release(dev->i2c);
}
