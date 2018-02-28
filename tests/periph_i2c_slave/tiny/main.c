#include <avr/io.h>
#include <util/delay.h>
#include "drv/sw_uart.h"
#include "drv/usi_i2c_master.h"

#define MEGA_ADDR 0x2d
#define AD5242_ADDR 0x2c
#define TMP_ADDR 0x48

int main(void)
{
	uint8_t txbuffer[3] = {'a', 'b', 'c'};
	uint8_t rxbuffer[8];
	usi_twi_result_t error;
#ifdef BOARD_REAPER
	DDRA &=~(1<<PA2);   // VBAT_OK input test
	DDRA |= (1<<PA1);   // Enable i2c level shifter
	DDRA |= (1<<PA3);   // VOUT_EN buck enable test
	PORTA |= (1<<PA1);  // Enable i2c level shifter
	PORTA |= (1<<PA3);  // Enable buck
#endif
	sw_uart_init();
	i2c_init_master();
	while (1) {
		_delay_ms(1000);
		error = i2c_write_bytes(MEGA_ADDR, txbuffer, sizeof(txbuffer));
		if (error == USI_TWI_SUCCESS)
			printf("Data sent: %c\n", txbuffer[0]);
		else
			printf("Sending data failed: %u\n", error);
		for (uint8_t i = 0; i < sizeof(txbuffer); ++i) {
			if (++txbuffer[i] > 'z')
				txbuffer[i] = 'a';
		}
		/*
		_delay_ms(1000);
		error = i2c_read_bytes(MEGA_ADDR, rxbuffer, 3);
		if (error == USI_TWI_SUCCESS)
			printf("Data read: %s\n", rxbuffer);
		else
			printf("Reading data failed: %u\n", error);
		*/
		/*
		_delay_ms(1000);
		int8_t tmp;
		error = i2c_read_regs(TMP_ADDR, 0x00, (uint8_t *) &tmp, 1);
		if (error == USI_TWI_SUCCESS)
			printf("Temperature: %d\n", tmp);
		else
			printf("Reading temperature failed: %u\n", error);
		*/
	}
}
