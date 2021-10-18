#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

#include "sram_undervolting.h"
#include "board.h"
#include "periph/gpio.h"

#include "bmx280_params.h"
#include "bmx280.h"
#include "fmt.h"
#include "stdio_uart.h"
#include "cpu.h"

void undervolting_reload_routine(void) {
	cpu_init();
	stdio_init();

	printf("Reload done!\n");
    return;
}

int main(void)
{
	bmx280_t dev;
    //int result;
	int16_t temperature;

	char str_temp[8];
	size_t len;

    puts("+------------Initializing------------+");
    switch (bmx280_init(&dev, &bmx280_params[0])) {
        case BMX280_ERR_BUS:
            puts("[Error] Something went wrong when using the I2C bus");
            return 1;
        case BMX280_ERR_NODEV:
            puts("[Error] Unable to communicate with any BMX280 device");
            return 1;
        default:
            /* all good -> do nothing */
            break;
    }


	while(1) {

		LED1_ON;
		LED2_OFF;

		/* Get temperature in deci degrees celsius */
		temperature = bmx280_read_temperature(&dev);
		/* format values for printing */
        len = fmt_s16_dfp(str_temp, temperature, -2);
        str_temp[len] = '\0';
		printf("Temperature [°C]: %s\n", str_temp);
		
		undervolting_sleep();

		LED1_OFF;
		LED2_ON;

		/* Get temperature in deci degrees celsius */
		temperature = bmx280_read_temperature(&dev);
		/* format values for printing */
        len = fmt_s16_dfp(str_temp, temperature, -2);
        str_temp[len] = '\0';
		printf("Temperature [°C]: %s\n", str_temp);

		undervolting_sleep();
	
	}
}
