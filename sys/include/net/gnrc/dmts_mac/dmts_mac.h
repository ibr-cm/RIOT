/*
 * Copyright (C) 2024 TU Braunschweig
 */

/**
 *
 * @defgroup    net_gnrc_dmts_mac gnrc_dmts_mac
 * @ingroup     net_gnrc
 * @{
 *
 * @brief   MAC layer implementation with Delay Measurement Time Synchronization functionality.
 *
 * The Delay Measured Time Synchronization (DMTS) protocol is a simple time synchroization
 * protocol, that can synchronize a whole network with a single message.
 * For further information on how this protocol works, refer to <Ping, Su. "Delay measurement time
 * synchronization for wireless sensor networks." Intel Research Berkeley Lab 6 (2003): 1-10.>
 *
 * ### Features
 *
 * The DMTS protocol enables the synchronization of two nodes with microseconds accuracy. This is
 * achieved by transmitting a RTC timestamp over the network interface, which will be received from
 * another node. Based on this timestamp, the receiving node calculates a clock offset that takes
 * the network and propagation delay into account. The clock offset can then be added to the clock
 * of the receiver node in order to be in sync with the other node. DMTS facilitates simultaneous
 * synchronization of all receiving nodes when combined with broadcast.
 * Furthermore, a skew detection mechanism has been integrated, leveraging the RTC to
 * adjust its frequency and eliminate any skew relative to a master clock.
 *
 * To utilize the skew detection functionality, multiple pseudo modules must be used. Enabling
 * the clock detection mechanism necessitates the use of the pseudo module
 * "gnrc_dmts_mac_skew_detection."
 * The skew detection mechanism calculates the clock drift out of two consecutive clock offsets
 * from a master node. In order to do that, one of the nodes have to be elected as a master node.
 * This node needs to use the pseudo module "gnrc_dmts_mac_skew_detection_master".
 * Since the clock detection feature relies on smooth calibration, which necessitates a constant
 * calibration interval of 32 seconds, receiving a beacon with DMTS information must occur every
 * 32 seconds. This ensures that the skew detection is accurately updated to minimize the node's
 * skew. To adjust this interval, there are multiple pseudo modules available:
 * "module_gnrc_dmts_mac_sync_interval_1s" sets the interval to 1 second,
 * "module_gnrc_dmts_mac_sync_interval_8s" sets the interval to 8 seconds
 * "module_gnrc_dmts_mac_sync_interval_16s" sets the interval to 16 seconds
 * "module_gnrc_dmts_mac_sync_interval_32s" sets the interval to 32 seconds and
 * "module_gnrc_dmts_mac_sync_interval_64s" sets it to 64 seconds. This means that if the sync
 * interval is set to one second, the receiver must receive a message as precisely as possible
 * every second so that the clock skew can be minimised. The same applies to 32 and 64 second
 * intervals. The default interval is 32 seconds.
 *
 * ### Limitations
 *
 * Presently, this implementation is exclusively compatible with the AT86RF215 radio driver and
 * STM32 MCUs which support a "Smooth Calibration". Additionally, the measured delays in dmts_mac.c
 * are only valid when using a 100MHz STM32 MCU. If you plan to use a different
 * MCU, with a different clock speed, you will need to measure these delays yourself.
 * Packets sent by the DMTS must always be 127 bytes in size. Therefore, the playoad must be
 * 106 bytes.
 *
 * ### Warning
 *
 * Changing code or any parameters such as modulation or data rate on the AT86RF215 radio will
 * cause the timings to mismatch, resulting in reduced synchronisation accuracy.
 * It should also be noted that interrupts that occur during the initiation of the transmission can
 * also impair the synchronization accuracy.
 *
 * @author  Lennart Lutz <lutz@ibr.cs.tu-bs.de>
 */

#ifndef NET_GNRC_DMTS_MAC_DMTS_MAC_H
#define NET_GNRC_DMTS_MAC_DMTS_MAC_H

#include "net/gnrc/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Creates an IEEE 802.15.4 DMTS MAC network interface
 *
 * @param[out] netif    The interface. May not be `NULL`.
 * @param[in] stack     The stack for the network interface's thread.
 * @param[in] stacksize Size of @p stack.
 * @param[in] priority  Priority for the network interface's thread.
 * @param[in] name      Name for the network interface. May be NULL.
 * @param[in] dev       Device for the interface
 *
 * @see @ref gnrc_netif_create()
 *
 * @return  0 on success
 * @return  negative number on error
 */
int gnrc_netif_dmts_create(gnrc_netif_t *netif, char *stack, int stacksize,
                                 char priority, const char *name, netdev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_DMTS_MAC_DMTS_MAC_H */
/** @} */
