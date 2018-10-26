/*
 * Copyright (C) 2018 Freie Universität Berlin
 *               2018 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_nrf52_802154
 * @{
 *
 * @file
 * @brief       Implementation of the radio driver for nRF52 radios
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Dimitri Nahm <dimitri.nahm@haw-hamburg.de>
 * @author      Semjon Kerner <semjon.kerner@fu-berlin.de>
 *
 * @}
 */

#include <string.h>
#include <errno.h>

#include "cpu.h"
#include "luid.h"
#include "mutex.h"

#include "net/ieee802154.h"
#include "net/netdev/ieee802154.h"
#include "nrf802154.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

static const netdev_driver_t nrf802154_netdev_driver;

netdev_ieee802154_t nrf802154_dev = {
    {
        .driver = &nrf802154_netdev_driver,
        .event_callback = NULL,
        .context = NULL,
#ifdef MODULE_NETSTATS_L2
        .stats = { 0 }
#endif
    },
#ifdef MODULE_GNRC
#ifdef MODULE_GNRC_SIXLOWPAN
    .proto = GNRC_NETTYPE_SIXLOWPAN,
#else
    .proto = GNRC_NETTYPE_UNDEF,
#endif
#endif
    .pan = IEEE802154_DEFAULT_PANID,
    .short_addr = { 0, 0 },
    .long_addr = { 0, 0, 0, 0, 0, 0, 0, 0 },
    .chan = IEEE802154_DEFAULT_CHANNEL,
    .flags = 0
};

static uint8_t rxbuf[IEEE802154_FRAME_LEN_MAX + 3]; /* len PHR + PSDU + LQI */
static uint8_t txbuf[IEEE802154_FRAME_LEN_MAX + 3]; /* len PHR + PSDU + LQI */

#define RX_AVAIL (1)
static volatile uint8_t _state;
static mutex_t txlock;

/**
 * @brief   Set radio into DISABLED state
 */
static void _disable(void)
{
    /* set device into DISABLED state */
    if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while (!(NRF_RADIO->EVENTS_DISABLED)) {};
        DEBUG("Device state: DISABLED\n");
    }
}

/**
 * @brief   Set radio into RXIDLE state
 */
static void _enable_rx(void)
{
    DEBUG("Set device state to RXIDLE\n");
    NRF_RADIO->PACKETPTR = (uint32_t)rxbuf;
    /* set device into RXIDLE state */
    if (NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        _disable();
    }
    NRF_RADIO->EVENTS_RXREADY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (!(NRF_RADIO->EVENTS_RXREADY)) {};
    DEBUG("Device state: RXIDLE\n");
}

/**
 * @brief   Set radio into TXIDLE state
 */
static void _enable_tx(void)
{
    DEBUG("Set device state to TXIDLE\n");
    NRF_RADIO->PACKETPTR = (uint32_t)txbuf;
    /* set device into TXIDLE state */
    if (NRF_RADIO->STATE != RADIO_STATE_STATE_TxIdle) {
        _disable();
    }
    NRF_RADIO->EVENTS_TXREADY = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    while (!(NRF_RADIO->EVENTS_TXREADY)) {};
    DEBUG("Device state: TXIDLE\n");
}

/**
 * @brief   Reset the RXIDLE state
 */
 static void _reset_rx(void)
 {
    if (NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) {
        return;
    }

    /* reset RX state and listen for new packets */
    _state &= ~(1 << NETDEV_EVENT_RX_COMPLETE);
    NRF_RADIO->TASKS_START = 1;
 }

static void _set_chan(uint16_t chan)
{
    assert((chan >= IEEE802154_CHANNEL_MIN) && (chan <= IEEE802154_CHANNEL_MAX));
    /* Channel map between 2400 MHZ ... 2500 MHz
     * -> Frequency = 2400 + FREQUENCY (MHz) */
    NRF_RADIO->FREQUENCY = (chan - 10) * 5;
    nrf802154_dev.chan = chan;
}

static int16_t _get_txpower(void)
{
    int8_t txpower = (int8_t)NRF_RADIO->TXPOWER;
    if (txpower < 0) {
        return (int16_t)(0xff00 | txpower);
    }
    return (int16_t)txpower;
}

static void _set_txpower(int16_t txpower)
{
    if (txpower > 9) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos9dBm;
    }
    if (txpower > 1) {
        NRF_RADIO->TXPOWER = (uint32_t)txpower;
    }
    else if (txpower > -1) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_0dBm;
    }
    else if (txpower > -5) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg4dBm;
    }
    else if (txpower > -9) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg8dBm;
    }
    else if (txpower > -13) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg12dBm;
    }
    else if (txpower > -17) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg16dBm;
    }
    else if (txpower > -21) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg20dBm;
    }
    else {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg40dBm;
    }
}

static int _init(netdev_t *dev)
{
    assert(dev);

    /* initialize local variables */
    mutex_init(&txlock);

    /* reset buffer */
    rxbuf[0] = 0;
    txbuf[0] = 0;
    _state = 0;

    /* power on peripheral */
    NRF_RADIO->POWER = 1;

    /* make sure the radio is disabled/stopped */
    _disable();

    /* we configure it to run in IEEE802.15.4 mode */
    NRF_RADIO->MODE = RADIO_MODE_MODE_Ieee802154_250Kbit;
    /* and set some fitting configuration */
    NRF_RADIO->PCNF0 = ((8 << RADIO_PCNF0_LFLEN_Pos) |
                        (RADIO_PCNF0_PLEN_32bitZero << RADIO_PCNF0_PLEN_Pos) |
                        (RADIO_PCNF0_CRCINC_Include << RADIO_PCNF0_CRCINC_Pos));
    NRF_RADIO->PCNF1 = IEEE802154_FRAME_LEN_MAX;
    /* set start frame delimiter */
    NRF_RADIO->SFD = IEEE802154_SFD;
    /* set MHR filters */
    NRF_RADIO->MHRMATCHCONF = 0;              /* Search Pattern Configuration */
    NRF_RADIO->MHRMATCHMAS = 0xff0007ff;      /* Pattern mask */
    /* configure CRC conform to IEEE802154 */
    NRF_RADIO->CRCCNF = ((RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
                         (RADIO_CRCCNF_SKIPADDR_Ieee802154 << RADIO_CRCCNF_SKIPADDR_Pos));
    NRF_RADIO->CRCPOLY = 0x011021;
    NRF_RADIO->CRCINIT = 0;

    /* assign default addresses */
    luid_get(nrf802154_dev.short_addr, IEEE802154_SHORT_ADDRESS_LEN);
    luid_get(nrf802154_dev.long_addr, IEEE802154_LONG_ADDRESS_LEN);

    /* set default channel */
    _set_chan(nrf802154_dev.chan);

    /* configure some shortcuts */
    NRF_RADIO->SHORTS = RADIO_SHORTS_RXREADY_START_Msk | RADIO_SHORTS_TXREADY_START_Msk;

    /* enable interrupts */
    NVIC_EnableIRQ(RADIO_IRQn);
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;

#ifdef MODULE_NETSTATS_L2
    memset(&dev->stats, 0, sizeof(netstats_t));
#endif

    /* switch to RX mode */
    _enable_rx();

    return 0;
}

static int _send(netdev_t *dev,  const iolist_t *iolist)
{
    (void)dev;

    DEBUG("Send a packet\n");

    assert(iolist);

    mutex_lock(&txlock);
    /* copy packet data into the transmit buffer */
    int len = 0;
    for (; iolist; iolist = iolist->iol_next) {
        if ((IEEE802154_FCS_LEN + len + iolist->iol_len) > (IEEE802154_FRAME_LEN_MAX)) {
            DEBUG("[nrf802154] send: unable to do so, packet is too large!\n");
            return -EOVERFLOW;
        }
        memcpy(&txbuf[len + 1], iolist->iol_base, iolist->iol_len);
        len += iolist->iol_len;
    }
#ifdef MODULE_NETSTATS_L2
        dev->stats.tx_bytes += len;
#endif

    /* specify the length of the package. */
    txbuf[0] = len + IEEE802154_FCS_LEN;

    /* trigger the actual transmission */
    _enable_tx();
    DEBUG("[nrf802154] send: putting %i byte into the ether\n", len);

    return len;
}

static int _recv(netdev_t *dev, void *buf, size_t len, void *info)
{
    (void)dev;
    (void)info;

    size_t pktlen = (size_t)rxbuf[0] - IEEE802154_FCS_LEN;

    /* check if packet data is readable */
    if (!(_state & (1 << NETDEV_EVENT_RX_COMPLETE))) {
        DEBUG("recv: no packet data available\n");
        return 0;
    }

    if (buf == NULL) {
        if (len > 0) {
            /* drop packet */
            DEBUG("recv: dropping packet of length %i\n", pktlen);
        }
        else if (len == 0) {
          /* return packet length */
          DEBUG("recv: return packet length: %i\n", pktlen);
          return pktlen;
        }
    }
    else {
        DEBUG("recv: reading packet of length %i\n", pktlen);
        pktlen = (len < pktlen) ? len : pktlen;
        memcpy(buf, &rxbuf[1], pktlen);

#ifdef MODULE_NETSTATS_L2
        dev->stats.rx_count++;
        dev->stats.rx_bytes += pktlen;
#endif
    }

    _reset_rx();

    return (int)pktlen;
}

static void _isr(netdev_t *dev)
{
    if (!nrf802154_dev.netdev.event_callback) {
        return;
    }
    if (_state & (1 << NETDEV_EVENT_RX_COMPLETE)) {
        nrf802154_dev.netdev.event_callback(dev, NETDEV_EVENT_RX_COMPLETE);
    }
    if (_state & (1 << NETDEV_EVENT_TX_COMPLETE)) {
        nrf802154_dev.netdev.event_callback(dev, NETDEV_EVENT_TX_COMPLETE);
        _state &= ~(1 << NETDEV_EVENT_TX_COMPLETE);
    }
}

static int _get(netdev_t *dev, netopt_t opt, void *value, size_t max_len)
{
    assert(dev);

    DEBUG("get: %s\n", netopt2str(opt));

    switch (opt) {
        case NETOPT_CHANNEL:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t *)value) = nrf802154_dev.chan;
            return sizeof(uint16_t);
        case NETOPT_TX_POWER:
            assert(max_len >= sizeof(int16_t));
            *((int16_t *)value) = _get_txpower();
            return sizeof(int16_t);

        default:
            return netdev_ieee802154_get((netdev_ieee802154_t *)dev,
                                          opt, value, max_len);
    }
}

static int _set(netdev_t *dev, netopt_t opt,
                const void *value, size_t value_len)
{
    assert(dev);

    DEBUG("set: %s\n", netopt2str(opt));

    switch (opt) {
        case NETOPT_CHANNEL:
            assert(value_len == sizeof(uint16_t));
            _set_chan(*((uint16_t *)value));
            return sizeof(uint16_t);
        case NETOPT_TX_POWER:
            assert(value_len == sizeof(int16_t));
            _set_txpower(*((int16_t *)value));
            return sizeof(int16_t);

        default:
            return netdev_ieee802154_set((netdev_ieee802154_t *)dev,
                                          opt, value, value_len);
    }
}

void isr_radio(void)
{
    NRF_RADIO->EVENTS_END = 0;

    /* did we just send or receive something? */
    if (NRF_RADIO->STATE == RADIO_STATE_STATE_RxIdle) {
        /* only process packet if event callback is set and CRC is valid */
        if ((nrf802154_dev.netdev.event_callback) &&
            (NRF_RADIO->CRCSTATUS == 1) &&
            (netdev_ieee802154_dst_filter(&nrf802154_dev, &rxbuf[1]) == 0)) {
            _state |= 1 << NETDEV_EVENT_RX_COMPLETE;
        }
        else {
            _reset_rx();
        }
    }
    else if (NRF_RADIO->STATE == RADIO_STATE_STATE_TxIdle) {
        _state |= 1 << NETDEV_EVENT_TX_COMPLETE;
        mutex_unlock(&txlock);
        _enable_rx();
    }

    if (_state) {
        nrf802154_dev.netdev.event_callback(&nrf802154_dev.netdev, NETDEV_EVENT_ISR);
    }

    cortexm_isr_end();
}

/**
 * @brief   Export of the netdev interface
 */
static const netdev_driver_t nrf802154_netdev_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr  = _isr,
    .get  = _get,
    .set  = _set
};
