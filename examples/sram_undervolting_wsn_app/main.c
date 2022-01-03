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
#include "periph/adc.h"

#include "xtimer.h"

char temp_thread_stack[THREAD_STACKSIZE_MAIN];

typedef struct main
{
    int16_t temperature;
    uint16_t adc;
    uint32_t seq_number;
} wsn_data_t;


void test_irq(void *irq_arg) {
    (void) irq_arg;
    printf("ISR was called!\n");
}

void undervolting_reload_routine(void) {

	cpu_init();
	stdio_init();
    xtimer_init();

    printf("UART and CPU done!\n");

    //Reload radio driver by resetting state machine
    at86rf2xx_t *at86Dev = (at86rf2xx_t*)(gnrc_netif_iter(NULL)->dev);
    if (!at86Dev) {
        puts("error: invalid device given");
        return;
    }
    printf("Got device, trying reset!\n");
    at86Dev->netdev.netdev.driver->init((netdev_t*)at86Dev);
    //at86rf2xx_reset(at86Dev);

    printf("Radio done!\n");

    adc_init(ADC_LINE(3));

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

static void send( int16_t temp_data, uint16_t adc_data, uint32_t seq ) {
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

    wsn_data_t paket_content;
    paket_content.temperature = temp_data;
    paket_content.adc         = adc_data;
    paket_content.seq_number  = seq;

    /* put packet together */
    pkt = gnrc_pktbuf_add(NULL, &paket_content, sizeof(paket_content), GNRC_NETTYPE_UNDEF);
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
    gpio_set(LED1_PIN);
    gpio_set(LED2_PIN);
    
	msg_t m;

    kernel_pid_t pid = thread_create(temp_thread_stack, sizeof(temp_thread_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            temp_thread, NULL, "temp");

    m.content.value = 1;

    int sample = 0;
    adc_init(ADC_LINE(3));
    uint32_t counter = 0;
	while(1) {

		//LED1_ON;
		//LED2_OFF;

		msg_send_receive(&m, &m, pid);
        printf("1st: Got msg with content %u\n", (unsigned int)m.content.value);

        sample = adc_sample(ADC_LINE(3), ADC_RES_10BIT);
        sample = adc_sample(ADC_LINE(3), ADC_RES_10BIT);
        printf("ADC Value: %i\n", sample);

        send((int16_t)m.content.value, (uint16_t) sample, counter);
		
		undervolting_sleep();
        counter++;

		//LED1_OFF;
		//LED2_ON;

		msg_send_receive(&m, &m, pid);
        printf("1st: Got msg with content %u\n", (unsigned int)m.content.value);

        sample = adc_sample(ADC_LINE(3), ADC_RES_10BIT);
        sample = adc_sample(ADC_LINE(3), ADC_RES_10BIT);
        printf("ADC Value: %i\n", sample);

        send((int16_t)m.content.value, (uint16_t) sample, counter);

		undervolting_sleep();
        counter++;
	
	}
}
