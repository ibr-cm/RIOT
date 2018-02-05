#include "temp.h"
#include "../../include/idealvolting_config.h"
#include "usi_i2c_master.h"

#ifdef BOARD_REAPER
static uint8_t bme680_mtmp[2] = {0x74, 0b00100001};

static struct {
	uint16_t t1;
	uint16_t t2;
	uint8_t t3;
} calibration;
#endif

uint8_t setup_temp(void)
{
	uint8_t res = 0;
#ifdef BOARD_REAPER
	/* Read calibration registers as defined in
	 * https://github.com/BoschSensortec/BME680_driver/blob/494b3bb26b026dcc26059e512bb71b60eb364376/bme680_defs.h */
	uint8_t cal_buf[5];

	res |= i2c_read_regs(TMP_ADDR, 0xe9, cal_buf, 2);
	res |= i2c_read_regs(TMP_ADDR, 0x8a, cal_buf + 2, 3);
	res |= i2c_write_bytes(TMP_ADDR, bme680_mtmp, 2);

	calibration.t1 = ((uint16_t) cal_buf[1] << 8) + ((uint16_t) cal_buf[0]);
	calibration.t2 = ((uint16_t) cal_buf[3] << 8) + ((uint16_t) cal_buf[2]);
	calibration.t3 = cal_buf[4];
#endif
	return res;
}

uint8_t get_temp(int8_t *value)
{
	uint8_t res = 0;
#if defined BOARD_INGA_BLUE
	res |= i2c_read_regs(TMP_ADDR, TMP_REG, (uint8_t *) value, 1);
#elif defined BOARD_REAPER
	/* Read first 2 bytes of the temperature adc registers
	 * and calculate the temperature as shown in
	 * https://github.com/BoschSensortec/BME680_driver/blob/494b3bb26b026dcc26059e512bb71b60eb364376/bme680.c#L824 */
	uint8_t data[2];
	uint16_t adc;
	int16_t result;
	int64_t var1, var2, var3;
	int32_t t_fine;

	res |= i2c_read_regs(TMP_ADDR, TMP_REG, data, 2);
	res |= i2c_write_bytes(TMP_ADDR, bme680_mtmp, 2);

	adc = (data[0] << 8) | data[1];
	var1 = ((int32_t) adc << 1) - ((int32_t) calibration.t1 << 1);
	var2 = (var1 * (int32_t) calibration.t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = (var3 * ((int32_t) calibration.t3 << 4)) >> 14;
	t_fine = (int32_t) (var2 + var3);
	result = (int16_t) (((t_fine * 5) + 128) >> 8);
	*value = result / 100;
#endif
	return res;
}
