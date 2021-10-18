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

#include "thread.h"
#include "msg.h"

char temp_thread_stack[THREAD_STACKSIZE_MAIN];

void undervolting_reload_routine(void) {
	cpu_init();
	stdio_init();

	printf("Reload done!\n");
    return;
}

void *temp_thread(void *arg)
{
    (void) arg;

	printf("2nd thread started, pid: %" PRIkernel_pid "\n", thread_getpid());

	bmx280_t dev;
    //int result;
	int16_t temperature;

	char str_temp[8];
	size_t len;

	puts("+------------Initializing------------+");
    switch (bmx280_init(&dev, &bmx280_params[0])) {
        case BMX280_ERR_BUS:
            puts("[Error] Something went wrong when using the I2C bus");
            return NULL;
        case BMX280_ERR_NODEV:
            puts("[Error] Unable to communicate with any BMX280 device");
            return NULL;
        default:
            /* all good -> do nothing */
            break;
    }

    msg_t m;

    while (1) {
        msg_receive(&m);
        printf("2nd: Got msg from %" PRIkernel_pid "\n", m.sender_pid);
		
		/* Get temperature in deci degrees celsius */
		temperature = bmx280_read_temperature(&dev);
		/* format values for printing */
        len = fmt_s16_dfp(str_temp, temperature, -2);
        str_temp[len] = '\0';
		printf("Temperature [°C]: %s\n", str_temp);
        m.content.value = temperature;
        
		msg_reply(&m, &m);
    }

    return NULL;
}

int main(void)
{

	msg_t m;

    kernel_pid_t pid = thread_create(temp_thread_stack, sizeof(temp_thread_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            temp_thread, NULL, "temp");

    m.content.value = 1;


	while(1) {

		LED1_ON;
		LED2_OFF;

		msg_send_receive(&m, &m, pid);
        printf("1st: Got msg with content %u\n", (unsigned int)m.content.value);
		
		undervolting_sleep();

		LED1_OFF;
		LED2_ON;

		msg_send_receive(&m, &m, pid);
        printf("1st: Got msg with content %u\n", (unsigned int)m.content.value);

		undervolting_sleep();
	
	}
}
