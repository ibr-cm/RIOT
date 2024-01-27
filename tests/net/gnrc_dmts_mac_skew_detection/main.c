/*
 * Copyright (C) 2024 TU Braunschweig
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Example to test dmts mac
 *
 * @author      Lennart Lutz <lutz@ibr.cs.tu-bs.de>
 *
 * @}
 */

#include <stdio.h>

#include "net/gnrc.h"
#include "net/gnrc/netif.h"
#include "periph/rtc.h"
#include "periph/gpio.h"
#include "ztimer.h"

// Pin D7 of a nucelo-f411re board
#define PIN GPIO_PIN(PORT_A, 8)

typedef struct {
    uint8_t arr[106]; // Dummy
} message_t;
// 9 MACH + 10 DMTS + 106 Payload + 2 MACF = 127 Byte

kernel_pid_t print_thread_pid = KERNEL_PID_UNDEF;
static char print_stack[THREAD_STACKSIZE_SMALL];

#ifdef MODULE_GNRC_DMTS_MAC_SKEW_DETECTION_MASTER

static int l2_send(netif_t *iface, char *str_addr, uint8_t *data, size_t len)
{
    size_t addr_len = 2; /* Only short addr */
    gnrc_pktsnip_t *pkt, *hdr;
    gnrc_netif_hdr_t *nethdr;
    uint8_t flags = 0x00;

    uint8_t addr[addr_len];
    l2util_addr_from_str(str_addr, addr);

    /* Put packet together */
    pkt = gnrc_pktbuf_add(NULL, data, len, GNRC_NETTYPE_UNDEF);
    if (pkt == NULL)
    {
        printf("error: packet buffer full\n");
        return 1;
    }
    hdr = gnrc_netif_hdr_build(NULL, 0, addr, addr_len);
    if (hdr == NULL)
    {
        printf("error: packet buffer full\n");
        gnrc_pktbuf_release(pkt);
        return 1;
    }

    pkt = gnrc_pkt_prepend(pkt, hdr);
    nethdr = (gnrc_netif_hdr_t *)hdr->data;
    nethdr->flags = flags;

    /* Send */

    if (gnrc_netif_send(container_of(iface, gnrc_netif_t, netif), pkt) < 1)
    {
        printf("error: unable to send\n");
        gnrc_pktbuf_release(pkt);
        return 1;
    }

    return 0;
}

#endif

static void *_print_thread(void *arg)
{
    netif_t *iface = (netif_t *) arg;

    msg_t msg;
    uint64_t timestamp;

    while (1)
    {
        msg_receive(&msg);
        /*
         * Take the timestamp and add the clock offset.
         * The clock offset can be obtained from the DMTS MAC layer using the
         * "NETOPT_DMTS_CLOCK_OFFSET" option.
        */
        rtc_get_timestamp_micros(&timestamp);

        int64_t clock_offset = 0;
        netif_get_opt(iface, NETOPT_DMTS_CLOCK_OFFSET, 0, (void *) &clock_offset,
                      sizeof(clock_offset));

        timestamp += clock_offset;

        rtc_print_timestamp(&timestamp);
    }
    
    return NULL;
}

static void print_cb(void *arg)
{
    (void) arg; // Not needed

    msg_t msg;
    msg_send(&msg, print_thread_pid);
}

int main(void)
{
    printf("DMTS MAC test application with skew detection feature\n");

    /* Get the network interface descriptor */
    netif_t *iface = netif_iter(NULL);

    /* Setup RTC */
    rtc_poweron();
    rtc_poweroff();
    rtc_poweron();
    rtc_init();

    /* Create print thread */
    print_thread_pid = thread_create(print_stack, sizeof(print_stack),
                                    THREAD_PRIORITY_MAIN - 1,
                                    THREAD_CREATE_STACKTEST,
                                    _print_thread, (void *) iface, "print_thread");

    /* Initialize a pin and callback */
    gpio_init_int(PIN, GPIO_IN_PU, GPIO_FALLING, print_cb, NULL);

#ifdef MODULE_GNRC_DMTS_MAC_SKEW_DETECTION_MASTER

    /* Reset calibration */
    rtc_smooth_cal(RTC_SmoothCalibPeriod_32sec, RTC_SmoothCalibPlusPulses_Reset, 0);

    /* Start sending with some interval */
#ifdef MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_1S
    /*
     * If the sync interval is one second we have to get 32 beacons to calculate the clock drift
     * one time.
     * So this calibrates the rtc five times.
    */
    uint8_t tx_ctr = 6*32;
#elif MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_8S
    /*
     * If the sync interval is 8 seconds we have to get 4 beacons to calculate the clock drift
     * one time.
     * So this calibrates the rtc five times.
    */
    uint8_t tx_ctr = 6*4;
#elif MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_16S
    /*
     * If the sync interval is 16 seconds we have to get 2 beacons to calculate the clock drift
     * one time.
     * So this calibrates the rtc five times.
    */
    uint8_t tx_ctr = 6*2;
#else
    /*
     * If the sync interval is 8, 16, 32 or 64 seconds, we need two beacons to calibrate the rtc.
     * After the first calibration is performed, next received beacon causes the rtc to
     * calibrate. So this calibrates the rtc five times.
    */
    uint8_t tx_ctr = 6;
#endif
    while (tx_ctr != 0)
    {
        message_t beacon;
        l2_send(iface, "FF:FF", (uint8_t *) &beacon, sizeof(message_t));
#ifdef MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_1S
        ztimer_sleep(ZTIMER_MSEC, 1000);
#elif MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_8S
        ztimer_sleep(ZTIMER_MSEC, 8000);
#elif MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_16S
        ztimer_sleep(ZTIMER_MSEC, 16000);
#elif MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_64S
        ztimer_sleep(ZTIMER_MSEC, 64000);
#else
        // Case for 32 second sync interval
        ztimer_sleep(ZTIMER_MSEC, 32000);
#endif
        tx_ctr--;
    }
#else
    /* Reset calibration */
    rtc_smooth_cal(RTC_SmoothCalibPeriod_32sec, RTC_SmoothCalibPlusPulses_Reset, 256);
#endif
    thread_sleep(); //Put thread to sleep
    return 0;
}
