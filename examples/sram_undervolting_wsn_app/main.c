#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

#include "sram_undervolting.h"
#include "uv_periph.h"
#include "board.h"
#include "periph/gpio.h"

#include "bmx280_params.h"
#include "bmx280.h"
#include "fmt.h"
#include "stdio_uart.h"
#include "cpu.h"

#include "thread.h"
#include "msg.h"


#include "net/gnrc.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/udp.h"

#include "at86rf2xx.h"

char temp_thread_stack[THREAD_STACKSIZE_MAIN];

void test_irq(void *irq_arg) {
    (void) irq_arg;
    printf("ISR was called!\n");
}

void undervolting_reload_routine(void) {
	cpu_init();
	stdio_init();

    //Reload radio driver by resetting state machine
    at86rf2xx_t *at86Dev = (at86rf2xx_t*)gnrc_netif_iter(NULL)->dev;
    at86rf2xx_reset(at86Dev);

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

    register_uv_isr(IIF_SIG, GPIO_IN_PU, GPIO_FALLING, &test_irq, NULL);

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

static void send( int16_t data) {
    netif_t *iface;
    uint8_t addr[GNRC_NETIF_L2ADDR_MAXLEN];
    size_t addr_len;
    gnrc_pktsnip_t *pkt, *hdr;
    gnrc_netif_hdr_t *nethdr;
    uint8_t flags = GNRC_NETIF_HDR_FLAGS_BROADCAST;

    iface = &(gnrc_netif_iter(NULL)->netif);
    if (!iface) {
        puts("error: invalid interface given");
        return;
    }

    /* parse address */
    addr_len = gnrc_netif_addr_from_str("bcast", addr);

    /* put packet together */
    pkt = gnrc_pktbuf_add(NULL, &data, sizeof(data), GNRC_NETTYPE_UNDEF);
    if (pkt == NULL) {
        puts("error: packet buffer full");
        return;
    }
    hdr = gnrc_netif_hdr_build(NULL, 0, addr, addr_len);
    if (hdr == NULL) {
        puts("error: packet buffer full");
        gnrc_pktbuf_release(pkt);
        return;
    }
    pkt = gnrc_pkt_prepend(pkt, hdr);
    nethdr = (gnrc_netif_hdr_t *)hdr->data;
    nethdr->flags = flags;
    /* and send it */
    if (gnrc_netif_send((gnrc_netif_t *)iface, pkt) < 1) {
        puts("error: unable to send");
        gnrc_pktbuf_release(pkt);
        return;
    }

    return;
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

        send((int16_t)m.content.value);
		
		undervolting_sleep();

		LED1_OFF;
		LED2_ON;

		msg_send_receive(&m, &m, pid);
        printf("1st: Got msg with content %u\n", (unsigned int)m.content.value);

        send((int16_t)m.content.value);

		undervolting_sleep();
	
	}
}
