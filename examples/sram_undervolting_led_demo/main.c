#include "sram_undervolting.h"
#include "board.h"
#include "periph/gpio.h"

int main(void)
{
	while(1) {

		__var_7 = 0x08;
		LED1_ON;
		LED2_OFF;
		
		undervolting_sleep();

		LED1_OFF;
		LED2_ON;

		undervolting_sleep();
	
	}
}
