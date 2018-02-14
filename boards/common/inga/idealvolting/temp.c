#include <stdint.h>
#include "idealvolting_config.h"
#include "periph/i2c.h"
#include "temp.h"

#ifdef BOARD_REAPER
static struct {
	uint16_t t1;
	uint16_t t2;
	uint8_t t3;
} calibration;
#endif

void setup_temp(void)
{
#ifdef BOARD_REAPER
	/* Read calibration registers as defined in
	 * https://github.com/BoschSensortec/BME680_driver/blob/494b3bb26b026dcc26059e512bb71b60eb364376/bme680_defs.h */
	uint8_t cal_buf[5];

	i2c_acquire(IV_I2C_DEV);
	i2c_read_regs(IV_I2C_DEV, TMP_ADDR, 0xe9, cal_buf, 2);
	i2c_read_regs(IV_I2C_DEV, TMP_ADDR, 0x8a, cal_buf + 2, 3);
	i2c_write_reg(IV_I2C_DEV, TMP_ADDR, 0x74, 0b00100001);
	i2c_release(IV_I2C_DEV);

	calibration.t1 = ((uint16_t) cal_buf[1] << 8) + ((uint16_t) cal_buf[0]);
	calibration.t2 = ((uint16_t) cal_buf[3] << 8) + ((uint16_t) cal_buf[2]);
	calibration.t3 = cal_buf[4];
#endif
}

int8_t get_temp(void)
{
#if defined BOARD_INGA_BLUE
	uint8_t result;
	i2c_acquire(IV_I2C_DEV);
	i2c_read_reg(IV_I2C_DEV, TMP_ADDR, TMP_REG, &result);
	i2c_release(IV_I2C_DEV);
	return result;
#elif defined BOARD_REAPER
	/* Read first 2 bytes of the temperature adc registers
	 * and calculate the temperature as shown in
	 * https://github.com/BoschSensortec/BME680_driver/blob/494b3bb26b026dcc26059e512bb71b60eb364376/bme680.c#L824 */
	uint8_t data[2];
	uint16_t adc;
	int16_t result;
	int32_t var1, var2, var3; //int64_t in Bosch reference
	int32_t t_fine;

	i2c_acquire(IV_I2C_DEV);
	i2c_read_regs(IV_I2C_DEV, TMP_ADDR, TMP_REG, data, 2);
	i2c_write_reg(IV_I2C_DEV, TMP_ADDR, 0x74, 0b00100001);
	i2c_release(IV_I2C_DEV);

	adc = (data[0] << 8) | data[1];
	var1 = ((int32_t) adc << 1) - ((int32_t) calibration.t1 << 1);
	var2 = (var1 * (int32_t) calibration.t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = (var3 * ((int32_t) calibration.t3 << 4)) >> 14;
	t_fine = (int32_t) (var2 + var3);
	result = (int16_t) (((t_fine * 5) + 128) >> 8);
	return result / 100;
#endif
}
