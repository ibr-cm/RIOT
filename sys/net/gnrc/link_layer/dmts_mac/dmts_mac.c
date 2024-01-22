/*
 * Copyright (C) 2023 TU Braunschweig
 */

/**
 * @ingroup     net_gnrc_dmts_mac
 * @{
 *
 * @file
 * @brief       Implementation of the DMTS MAC protocol.
 *              For further information on how this protcol works, refer to
 *              <Ping, Su. "Delay measurement time synchronization for wireless
 *              sensor networks." Intel Research Berkeley Lab 6 (2003): 1-10.>
 *
 * @note        Currently this implementation works only for the at86rf215
 *              radio driver with the legacy 0-QPSK modulation with default settings.
 *
 * @author      Lennart Lutz <lutz@ibr.cs.tu-bs.de>
 */

#include "net/gnrc.h"
#include "net/gnrc/netif/ieee802154.h"
#include "net/netdev/ieee802154.h"

#include "at86rf215_netdev.h" // Needed to call a getter function

#ifdef MODULE_GNRC_IPV6
#include "net/ipv6/hdr.h"
#endif

#define ENABLE_DEBUG 0
#include "debug.h"

#include "od.h"
#include "fmt.h" // Needed if Debugging is enabled
#include "xtimer.h"
#include "random.h"
#include "periph/rtc.h"

/*
 * Deterministic and measured delays for the at86rf215 radio:
 * Warning: The measured delays are only valid if a 100 MHz MCU is used and the modulation is 
 *          0-QPSK with default settings (250kbit/s).
 *          These delays change, if the code of the radio module, the modulation or the
 *          RTC functions change.
 *
 * Taking the RTC timestamp in the _send function: ~16us (measured)
 *
 * _send <-> cmd_rf_txprep: Depends on packet size!
 * -> 127 (125, MAC Footer isnt loaded into the radio) Byte ~400us (measured)
 *
 * cmd_rf_txprep <-> cmd_rf_tx: ~78us (measured)
 *
 * cmd_rf_tx <-> tx_started:
 * - tx_bb_delay: 3us
 * - tx_proc_delay: 2us
 * - tx_start_delay: 4us
 * - tx_pa_ramp: 4us
 * <- Datasheet AT86RF215 page 46ff
 *
 * tx_started <-> isr_call (receiver):
 * - 250 kbit/s <-> 1 bit / 4us (one bit each 4 us)
 * - The rx_started callback is being called after the radio receives a valid PHR
 * - A look into IEEE 802.15.4 on page 43f (Chapter 6.3) for legacy 0-QPSK modulation reveals:
 * -> Preamble, SFD and PHR: (32 + 8 + 8) * 4 = 192us
 *
 * isr_call <-> rx_started:
 * - Is handled dynamically by taking a timestamp in the ISR routine, which is
 *   combined with another timestamp directly in the rx_started callback.
 *   The difference between the two timestamps is then subtracted from
 *   the rx_started timestamp.
 * 
 * Taking the RTC timestamp in the rx_started callback function: ~16us (measured)
 *
 */
uint16_t at86rf215_deterministic_offset = 16 + 400 + 78 + 13 + 192 + 16;

typedef struct
{
    uint64_t rx_timestamp; // Received timestamp from remote node
    uint64_t rx_started_timestamp;
    int64_t clock_offset;
    int64_t last_clock_offset;
    int64_t clock_drift;
} dmts_mac_timestamp_t;

dmts_mac_timestamp_t timestamps;

#define DMTS_ID (14826) // Unique DMTS ID

uint64_t start_timestamp = 0;
uint64_t tx_timestamp;
uint16_t dmts_id; // Holds the DMTS ID

/*
 * Indicates whether the received packet contains dmts information or not
 * (if not dont calculate the clock offset)
 */
uint8_t dmts_pkt_valid = 0;
/*
 * Indicates whether dmts calulates the clock offset of a remote clock
 * automatically
 */
uint8_t auto_dmts = 1;

/*
 * Period duration of the clock that is used to calculate the necessary pulses
 * to mask or add.
 */
uint16_t period_duration = 305; // 30,5 us -> 32768 Hz crystal

/* CSMA/CA parameter */

uint8_t min_be = 3;
uint8_t max_be = 4;
uint16_t max_backoffs = 4;
uint32_t backoff_period = 160; // 160 us

static gnrc_pktsnip_t *_dmts_mac_recv(gnrc_netif_t *netif);
static int _dmts_mac_send(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt);
static int _dmts_mac_init(gnrc_netif_t *netif);
static void _dmts_mac_event_cb(netdev_t *dev, netdev_event_t event);
static int _dmts_mac_get_options(gnrc_netif_t *netif, gnrc_netapi_opt_t *opt);
static int _dmts_mac_set_options(gnrc_netif_t *netif, const gnrc_netapi_opt_t *opt);

static const gnrc_netif_ops_t dmts_ops = {
    .init = _dmts_mac_init,
    .send = _dmts_mac_send,
    .recv = _dmts_mac_recv,
    .get = _dmts_mac_get_options,
    .set = _dmts_mac_set_options,
};

int gnrc_netif_dmts_create(gnrc_netif_t *netif, char *stack, int stacksize,
                           char priority, const char *name, netdev_t *dev)
{
    return gnrc_netif_create(netif, stack, stacksize, priority, name, dev,
                             &dmts_ops);
}

static gnrc_pktsnip_t *_make_netif_hdr(uint8_t *mhr)
{
    gnrc_netif_hdr_t *hdr;
    gnrc_pktsnip_t *snip;
    uint8_t src[IEEE802154_LONG_ADDRESS_LEN], dst[IEEE802154_LONG_ADDRESS_LEN];
    int src_len, dst_len;
    le_uint16_t _pan_tmp; /* TODO: hand-up PAN IDs to GNRC? */

    dst_len = ieee802154_get_dst(mhr, dst, &_pan_tmp);
    src_len = ieee802154_get_src(mhr, src, &_pan_tmp);
    if ((dst_len < 0) || (src_len < 0))
    {
        DEBUG("_make_netif_hdr_dmts_mac: Unable to get addresses\n");
        return NULL;
    }
    /* allocate space for header */
    snip = gnrc_netif_hdr_build(src, (size_t)src_len, dst, (size_t)dst_len);
    if (snip == NULL)
    {
        DEBUG("_make_netif_hdr_dmts_mac: No space left in packet buffer\n");
        return NULL;
    }
    hdr = snip->data;
    /* set broadcast flag for broadcast destination */
    if ((dst_len == 2) && (dst[0] == 0xff) && (dst[1] == 0xff))
    {
        hdr->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
    }
    /* set flags for pending frames */
    if (mhr[0] & IEEE802154_FCF_FRAME_PEND)
    {
        hdr->flags |= GNRC_NETIF_HDR_FLAGS_MORE_DATA;
    }
    return snip;
}

static gnrc_pktsnip_t *_dmts_mac_recv(gnrc_netif_t *netif)
{
    netdev_t *dev = netif->dev;
    netdev_ieee802154_rx_info_t rx_info;
    gnrc_pktsnip_t *pkt = NULL;
    int bytes_expected = dev->driver->recv(dev, NULL, 0, NULL);

    if (bytes_expected >= (int)IEEE802154_MIN_FRAME_LEN)
    {
        int nread;
        // For debugging purposes
        char src_str[GNRC_NETIF_HDR_L2ADDR_PRINT_LEN];
        size_t mhr_len = 0;

        pkt = gnrc_pktbuf_add(NULL, NULL, bytes_expected, GNRC_NETTYPE_UNDEF);
        if (pkt == NULL)
        {
            DEBUG("_recv_dmts_mac: Cannot allocate pktsnip.\n");
            /* Discard packet on netdev device */
            dev->driver->recv(dev, NULL, bytes_expected, NULL);
            return NULL;
        }
        nread = dev->driver->recv(dev, pkt->data, bytes_expected, &rx_info);
        if (nread <= 0)
        {
            gnrc_pktbuf_release(pkt);
            return NULL;
        }

        if (netif->flags & GNRC_NETIF_FLAGS_RAWMODE)
        {
            /* Raw mode, skip packet processing, but provide rx_info via
             * GNRC_NETTYPE_NETIF */
            gnrc_pktsnip_t *netif_snip = gnrc_netif_hdr_build(NULL, 0, NULL, 0);
            if (netif_snip == NULL)
            {
                DEBUG("_recv_dmts_mac: No space left in packet buffer\n");
                gnrc_pktbuf_release(pkt);
                return NULL;
            }
            gnrc_netif_hdr_t *hdr = netif_snip->data;
            hdr->lqi = rx_info.lqi;
            hdr->rssi = rx_info.rssi;
            gnrc_netif_hdr_set_netif(hdr, netif);
            pkt = gnrc_pkt_append(pkt, netif_snip);
        }
        else
        {
            /* Normal mode, try to parse the frame according to IEEE 802.15.4 */
            gnrc_pktsnip_t *ieee802154_hdr, *netif_hdr;
            gnrc_netif_hdr_t *hdr;
            mhr_len = ieee802154_get_frame_hdr_len(pkt->data);

            /* nread was checked for <= 0 before so we can safely cast it to
             * unsigned */
            if ((mhr_len == 0) || ((size_t)nread < mhr_len))
            {
                DEBUG("_recv_dmts_mac: Illegally formatted frame received\n");
                gnrc_pktbuf_release(pkt);
                return NULL;
            }
            nread -= mhr_len;
            /* mark IEEE 802.15.4 header */
            ieee802154_hdr = gnrc_pktbuf_mark(pkt, mhr_len, GNRC_NETTYPE_UNDEF);
            if (ieee802154_hdr == NULL)
            {
                DEBUG("_recv_dmts_mac: No space left in packet buffer\n");
                gnrc_pktbuf_release(pkt);
                return NULL;
            }
            netif_hdr = _make_netif_hdr(ieee802154_hdr->data);
            if (netif_hdr == NULL)
            {
                DEBUG("_recv_dmts_mac: No space left in packet buffer\n");
                gnrc_pktbuf_release(pkt);
                return NULL;
            }

            hdr = netif_hdr->data;
            hdr->lqi = rx_info.lqi;
            hdr->rssi = rx_info.rssi;
            gnrc_netif_hdr_set_netif(hdr, netif);
            dev->driver->get(dev, NETOPT_PROTO, &pkt->type, sizeof(pkt->type));

            /* Get src address for debugging */
            if (IS_ACTIVE(ENABLE_DEBUG))
            {
                gnrc_netif_addr_to_str(gnrc_netif_hdr_get_src_addr(hdr), hdr->src_l2addr_len, src_str);
            }

            gnrc_pktbuf_remove_snip(pkt, ieee802154_hdr);
            pkt = gnrc_pkt_append(pkt, netif_hdr);
        }

        /* Get dmts_id and timestamp from the message */
        gnrc_pktsnip_t *id_pkt;
        id_pkt = gnrc_pktbuf_mark(pkt, sizeof(uint16_t), GNRC_NETTYPE_UNDEF);
        dmts_id = *(uint16_t *)id_pkt->data;

        /* Check dmts_id */
        if (dmts_id == DMTS_ID)
        {
            DEBUG("_recv_dmts_mac: Message does contain DMTS information.\n");
            gnrc_pktbuf_remove_snip(pkt, id_pkt); // Remove id

            gnrc_pktsnip_t *timestamp_pkt;
            timestamp_pkt = gnrc_pktbuf_mark(pkt, sizeof(uint64_t), GNRC_NETTYPE_UNDEF);
            tx_timestamp = *(uint64_t *)timestamp_pkt->data;
            memcpy(&timestamps.rx_timestamp, &tx_timestamp, sizeof(uint64_t)); // Get timestamp
            gnrc_pktbuf_remove_snip(pkt, timestamp_pkt);                       // Remove timestamp

            dmts_pkt_valid = 1;
            nread -= 10; // uint64_t + uint16_t
            gnrc_pktbuf_realloc_data(pkt, nread);
        }
        else
        {
            DEBUG("_recv_dmts_mac: Message does not contain DMTS information, dropping packet.\n");
            gnrc_pktbuf_release(pkt);
            return NULL;
        }

        if (IS_ACTIVE(ENABLE_DEBUG))
        {
            DEBUG("_recv_dmts_mac: Received packet from %s,\n", src_str);
            DEBUG("with length: MAC %d + DMTS 10 + Payload %d Byte\n", mhr_len, nread);

            if (IS_USED(MODULE_OD))
            {
                od_hex_dump(pkt->data, nread, OD_WIDTH_DEFAULT);
            }
        }
    }
    else if (bytes_expected > 0)
    {
        DEBUG("_recv_dmts_mac: Received frame is too short\n");
        dev->driver->recv(dev, NULL, bytes_expected, NULL);
    }

    return pkt;
}

static int _dmts_mac_csma(netdev_t *dev)
{
    random_init(_xtimer_now());

    int nb = 0, be = min_be;
    while (nb <= max_be)
    {
        /* Delay for an adequate random backoff period */
        if (be < min_be)
        {
            be = min_be;
        }
        if (be > max_be)
        {
            be = max_be;
        }
        uint32_t max_backoff = ((1 << be) - 1) * backoff_period;

        uint32_t period = random_uint32() % max_backoff;
        if (period < backoff_period)
        {
            period = backoff_period;
        }
        xtimer_usleep(period);

        /* Check if channel is clear (CCA) */

        netopt_enable_t is_clear;
        int res = dev->driver->get(dev,
                                   NETOPT_IS_CHANNEL_CLR,
                                   (void *)&is_clear,
                                   sizeof(netopt_enable_t));
        if (res < 0)
        {
            DEBUG("_csma_dmts_mac: Device driver failure... transmission aborted\n");
            return -ECANCELED;
        }

        /* If medium is clear, return */
        if (is_clear == NETOPT_ENABLE)
        {
            return 1;
        }

        /* Medium is busy: increment CSMA counters */
        DEBUG("_csma_dmts_mac: Radio medium busy.\n");
        be++;
        if (be > max_be)
        {
            be = max_be;
        }
        nb++;

        /* ... and try again if we have no exceeded the retry limit */
    }

    /* if we arrive here, medium was never available for transmission */
    DEBUG("_csma_dmts_mac: CSMA failure: medium never available.\n");
    return -EBUSY;
}

static int _dmts_mac_send(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt)
{
    netdev_t *dev = netif->dev;
    netdev_ieee802154_t *state = (netdev_ieee802154_t *)netif->dev;
    gnrc_netif_hdr_t *netif_hdr;
    const uint8_t *src, *dst = NULL;
    int res = 0;
    size_t src_len, dst_len;
    uint8_t mhr[IEEE802154_MAX_HDR_LEN];
    uint8_t flags = (uint8_t)(state->flags & NETDEV_IEEE802154_SEND_MASK);
    le_uint16_t dev_pan = byteorder_btols(byteorder_htons(state->pan));

    flags |= IEEE802154_FCF_TYPE_DATA;
    if (pkt == NULL)
    {
        DEBUG("_send_dmts_mac: PKT was NULL\n");
        return -EINVAL;
    }
    if (pkt->type != GNRC_NETTYPE_NETIF)
    {
        DEBUG("_send_dmts_mac: First header is not generic netif header\n");
        return -EBADMSG;
    }

    netif_hdr = pkt->data;
    if (netif_hdr->flags & GNRC_NETIF_HDR_FLAGS_MORE_DATA)
    {
        /* Set frame pending field */
        flags |= IEEE802154_FCF_FRAME_PEND;
    }

    /* Prepare destination address */
    if (netif_hdr->flags & /* If any of these flags is set assume broadcast */
        (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST))
    {
        dst = ieee802154_addr_bcast;
        dst_len = IEEE802154_ADDR_BCAST_LEN;
    }
    else
    {
        dst = gnrc_netif_hdr_get_dst_addr(netif_hdr);
        dst_len = netif_hdr->dst_l2addr_len;
    }
    src_len = netif_hdr->src_l2addr_len;

    if (src_len > 0)
    {
        src = gnrc_netif_hdr_get_src_addr(netif_hdr);
    }
    else // If src adress not set -> get device l2 adress
    {
        src_len = netif->l2addr_len;
        src = netif->l2addr;
    }

    /* Fill MAC header, seq should be set by device */
    if ((res = ieee802154_set_frame_hdr(mhr, src, src_len,
                                        dst, dst_len, dev_pan,
                                        dev_pan, flags, state->seq++)) == 0)
    {
        DEBUG("_send_dmts_mac: Error preperaring frame\n");
        return -EINVAL;
    }

    DEBUG("_send_dmts_mac: Performing CSMA/CA\n");

    /* Perform software CSMA/CA */
    int csma = _dmts_mac_csma(dev);
    if (csma < 0)
    {
        gnrc_pktbuf_release(pkt);
        return -EBUSY;
    }

    /*
     * Build and add dmts payload (ID and timestamp)
     * MAC -> ID -> Timestamp -> (IPv6, UDP/TCP) -> Data
     */
    pkt = gnrc_pktbuf_remove_snip(pkt, pkt); // Release the netif (mac) header

    gnrc_pktsnip_t *timestamp_pkt;
    gnrc_pktsnip_t *id_pkt;
    timestamp_pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(uint64_t), GNRC_NETTYPE_UNDEF);
    dmts_id = DMTS_ID;
    id_pkt = gnrc_pktbuf_add(timestamp_pkt, &dmts_id, sizeof(uint16_t), GNRC_NETTYPE_UNDEF);

    /* Prepare iolist for Radio and send the frame */
    iolist_t iolist = {
        .iol_next = (iolist_t *)id_pkt, // Payload packet (ID->Timestamp->(IPv6, UDP/TCP)->Data)
        .iol_base = mhr,                // MAC Header
        .iol_len = (size_t)res};

    /* Get timestamp and copy it into the timestamp_pkt */
    rtc_get_timestamp_micros(&tx_timestamp);
    tx_timestamp += timestamps.clock_offset; // Forward offset of another remote clock
    memcpy(timestamp_pkt->data, &tx_timestamp, sizeof(uint64_t));

    res = dev->driver->send(dev, &iolist);

    DEBUG("_send_dmts_mac: Packet sent\n");

    /* Release packet */
    gnrc_pktbuf_release(id_pkt);

    return res;
}

static int _dmts_mac_init(gnrc_netif_t *netif)
{
    netdev_t *dev = netif->dev;

    gnrc_netif_default_init(netif);
    dev->event_callback = _dmts_mac_event_cb;

    /* Configure radio */

    netopt_enable_t option = NETOPT_DISABLE;
    // Turn off Hardware CSMA/CA
    dev->driver->set(dev, NETOPT_CSMA, &option, sizeof(option));
    // Turn off auto CSMA before sending
    dev->driver->set(dev, NETOPT_AUTOCCA, &option, sizeof(option));
    // Turn off ACKs
    dev->driver->set(dev, NETOPT_AUTOACK, &option, sizeof(option));
    // 0 Retransmissions
    uint8_t retrans = 0;
    dev->driver->set(dev, NETOPT_RETRANS, &retrans, sizeof(retrans));

    /* Init timestamp struct */
    timestamps.clock_offset = 0;
    timestamps.last_clock_offset = 0;
    timestamps.rx_started_timestamp = 0;
    timestamps.rx_timestamp = 0;

    DEBUG("_init_dmts_mac: Successfully initiated dmts_mac!\n");

    return 0;
}

#ifdef MODULE_GNRC_DMTS_MAC_SKEW_DETECTION

/*
 * Variable holds the current calibration value.
 */
int32_t current_cal_val = 0;
/*
 * Delay the calibration routine by some time.
 * The minimum is 2, because we need at least two beacons to calculate the clock drift.
 */
uint8_t cal_delay = 2;

static void _dmts_mac_cal_rtc(void)
{
    cal_delay--;
    if (cal_delay == 0)
    {
        /*
         * After the initial delay, calibrate the rtc every time a beacon has been received.
         */
        cal_delay = 1;
        
        DEBUG("_cal_rtc_dmts_mac: Clock Drift: %ld -> ", (uint32_t) timestamps.clock_drift);
        
        /*
         * 32767 HZ ~= 30,5 us
         * 32000 HZ ~= 31,25 us
         */
        int32_t cal_val = (timestamps.clock_drift * 10) / period_duration;

        DEBUG("CALV: %ld, CCALV: %ld -> ", cal_val, current_cal_val);

        int32_t tmp_cal_val = current_cal_val + cal_val;
        if (tmp_cal_val > 255)
        {
            cal_val = 255;
        }
        else if (tmp_cal_val < -255)
        {
            cal_val = -255;
        }
        else
        {
            cal_val = tmp_cal_val;
        }

        current_cal_val = cal_val;

        /*
        * Since the asynchronous prescaler is less than 3, 256 equals to 0, because the
        * synchronous prescaler is accelerated by 8.
        * (See reference manual of STM32 MCUs, section "Smooth Calibration")
        */
        if (cal_val > 0)
        {
            cal_val = 256 + cal_val;
            rtc_smooth_cal(RTC_SmoothCalibPeriod_32sec,
                           RTC_SmoothCalibPlusPulses_Reset, cal_val);
        }
        else if (cal_val < 0)
        {
            cal_val = 256 + cal_val;
            rtc_smooth_cal(RTC_SmoothCalibPeriod_32sec,
                           RTC_SmoothCalibPlusPulses_Set, cal_val);
        }

        DEBUG("CALR set: %ld\n", cal_val);
    }
}

#endif

static void _dmts_mac_calc_offset(void)
{
    /*
     * If rx_started_timestamp > rx_timestamp + d_offset
     *
     * -> our clock is advanced
     * -> clock_offset is negative
     */

    if (dmts_pkt_valid)
    {
        DEBUG("_calc_offset_dmts_mac: Calculating clock offset\n");

        timestamps.clock_offset = (timestamps.rx_timestamp + at86rf215_deterministic_offset)
                                - timestamps.rx_started_timestamp;

        /**
         * Advance or delay the RTC according to the clock drift
         * 
         * DMTS MAC supports multiple sync intervals when the skew detection feature is used.
         * 1 second, 8 second, 16 second, 32 second (default) and 64 second sync interval.
         * The interval defines the time between two consecutive beacons.
         * Since a smooth calibration is performed, we need the clock drift of an
         * 32 second interval. See reference manual of STM32 MCUs, section "Smooth Calibration"
         * for more information.
         *
         * For a sync interval of 1, 8 and 16 second, we have to multiply the obtained clock drift
         * by 32, 4 and 2 respectively.
         * For a sync interval of 32 seconds, we do not need to modify the clock drift. (default)
         * For a sync interval of 64 seconds, we have to divide the clock drift by 2.
         */

#ifdef MODULE_GNRC_DMTS_MAC_SKEW_DETECTION

#if !defined(MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_32S) && !defined(MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_64S)
    static uint8_t cal_ctr = 0;
    static bool first_offset = true;

#ifdef MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_1S
#define CAL_MSG_DELAY 33
#elif MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_8S
#define CAL_MSG_DELAY 5
#elif MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_16S
#define CAL_MSG_DELAY 3
#endif

    if (cal_ctr == 0 && first_offset)
    {
        /* Take the first clock offset */
        timestamps.last_clock_offset = timestamps.clock_offset;
        first_offset = false;
    }
    cal_ctr++;
    if (cal_ctr == CAL_MSG_DELAY)
    {
        timestamps.clock_drift = timestamps.last_clock_offset - timestamps.clock_offset;
        _dmts_mac_cal_rtc();
        timestamps.last_clock_offset = timestamps.clock_offset;
        cal_ctr = 0; // Reset
    }

        timestamps.last_clock_offset = timestamps.clock_offset;

#else
    if (timestamps.last_clock_offset != 0)
    {
        timestamps.clock_drift = timestamps.last_clock_offset - timestamps.clock_offset;
#ifdef MODULE_GNRC_DMTS_MAC_SYNC_INTERVAL_64S
        timestamps.clock_drift /= 2;
#endif
        _dmts_mac_cal_rtc();
    }
    timestamps.last_clock_offset = timestamps.clock_offset;
#endif
        
#endif

        dmts_pkt_valid = 0;

        if (IS_ACTIVE(ENABLE_DEBUG))
        {
            char int64_str[64];
            fmt_s64_dec(int64_str, timestamps.clock_offset);
            DEBUG("_calc_offset_dmts_mac: Clock Offset: %s\n", int64_str);
        }
    }
}

static void _dmts_mac_event_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netif_t *netif = (gnrc_netif_t *)dev->context;

    if (event == NETDEV_EVENT_ISR)
    {
        event_post(&netif->evq[GNRC_NETIF_EVQ_INDEX_PRIO_LOW], &netif->event_isr);
    }
    else
    {
        DEBUG("_event_dmts_mac: Event triggered -> %i\n", event);

        gnrc_pktsnip_t *pkt = NULL;
        switch (event)
        {
        case NETDEV_EVENT_RX_STARTED:
            start_timestamp = xtimer_now_usec64();
            // Timestamp on receive
            rtc_get_timestamp_micros(&timestamps.rx_started_timestamp);
            timestamps.rx_started_timestamp = timestamps.rx_started_timestamp -
                                              (start_timestamp - get_isr_timestamp());
            break;
        case NETDEV_EVENT_RX_COMPLETE:
            pkt = netif->ops->recv(netif);

            if (pkt != NULL)
            {
                /* Throw away packet if no one is interested */
                if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt))
                {
                    /* Throw away packet if no one is interested */
                    if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt))
                    {
                        DEBUG("_event_dmts_mac: No one is interested on packet type: %i\n",
                              pkt->type);
                        gnrc_pktbuf_release(pkt);
                    }
                }

            timestamps.rx_started_timestamp = timestamps.rx_started_timestamp -
                                              (start_timestamp - get_isr_timestamp());
            break;
        case NETDEV_EVENT_RX_COMPLETE:
            pkt = netif->ops->recv(netif);

            if (pkt != NULL)
            {
                /* Throw away packet if no one is interested */
                if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt))
                {
                    DEBUG("_event_dmts_mac: No one is interested on packet type: %i\n",
                          pkt->type);
                    gnrc_pktbuf_release(pkt);
                }
            }

            /* Calculate clock offset, only if it is enabled */
            if (auto_dmts == 1)
            {
                _dmts_mac_calc_offset();
            }

            break;
        default:
            DEBUG("_event_dmts_mac: Unhandled event %u.\n", event);
        }
    }
}

static int _dmts_mac_get_options(gnrc_netif_t *netif, gnrc_netapi_opt_t *opt)
{
    int res = -ENOTSUP;

    /* DMTS Values */
    switch (opt->opt)
    {
    case NETOPT_DMTS_CLOCK_OFFSET:
        *(int64_t *)opt->data = timestamps.clock_offset;
        res = sizeof(int64_t);
        break;
    case NETOPT_DMTS_CALC_CLOCK_OFFSET:
        _dmts_mac_calc_offset();
        res = 1;
        break;
    default:
        /* If option is not used for DMTS, forward option */
        res = gnrc_netif_get_from_netdev(netif, opt);
        break;
    }

    return res;
}

static int _dmts_mac_set_options(gnrc_netif_t *netif, const gnrc_netapi_opt_t *opt)
{
    int res = -ENOTSUP;

    switch (opt->opt)
    {
    case NETOPT_DMTS_AUTO_CLOCK_OFFSET:
        assert(opt->data_len == sizeof(uint8_t));
        auto_dmts = *(uint8_t *)opt->data;
        res = sizeof(uint8_t);
        break;
    case NETOPT_DMTS_RESET_CAL_RTC:
#ifdef MODULE_GNRC_DMTS_MAC_SKEW_DETECTION
        current_cal_val = 0;
        cal_delay = 3;
        rtc_smooth_cal(RTC_SmoothCalibPeriod_32sec, RTC_SmoothCalibPlusPulses_Reset, 256);
        timestamps.clock_drift = 0;
        timestamps.last_clock_offset = 0;
#endif
        res = sizeof(uint8_t);
        break;
    case NETOPT_DMTS_PERIOD:
        assert(opt->data_len == sizeof(uint16_t));
        period_duration = *(uint16_t *) opt->data;
        res = sizeof(uint16_t);
        break;
    default:
        /* If option is not used for DMTS, forward option */
        res = gnrc_netif_set_from_netdev(netif, opt);
        break;
    }

    return res;
}

/** @} */
