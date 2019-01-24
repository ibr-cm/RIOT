/*
 * Copyright (C) 2008, 2009, 2010 Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
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
 * @brief       Default application that shows a lot of functionality of RIOT
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "msg.h"
#include "thread.h"
#include "shell.h"
#include "shell_commands.h"
#include "xtimer.h"
#include "net/gnrc.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/netif/ieee802154.h"
#include "periph/i2c.h"

#define SEND_INTERVAL (1)
#define RCV_QUEUE_SIZE (8)
#define MAX_PAYLOAD_LENGTH (32)
#define MAGIC_STRING "IBREVAL\0"

#define SHELL_BUFFER_SIZE (256)

#ifndef NODE_ID
#error NODE_ID undefined
#endif

/// 512 required for samr21-xpro, 256 sufficient for INGA and telosb
#ifdef BOARD_SAMR21_XPRO
char dump_thread_stack[1024+256];
char send_thread_stack[1024+256];
#else
char dump_thread_stack[512+256];
char send_thread_stack[512+256];
#endif
//static msg_t dump_thread_msg_queue[RCV_QUEUE_SIZE];
//static msg_t send_thread_msg_queue[RCV_QUEUE_SIZE];

char line_buf[SHELL_BUFFER_SIZE];

typedef struct /*__attribute__((packed))*/ {
    char magic_string[8];
    uint8_t node_id;
    uint8_t seq_nr;
    uint8_t temp;
    uint8_t payload_length;
    uint8_t payload[MAX_PAYLOAD_LENGTH];
} eval_message_t;

static eval_message_t eval_message = { .node_id = NODE_ID, .payload_length = 5, .payload = "Hello\0", .magic_string = MAGIC_STRING, .seq_nr = 0 };

kernel_pid_t send_thread_pid = 0;

/**
 * Sends a packet via the network interface
 * 
 * tx [count [delay]]
 * - count: Number of packets to send
 * - delay: Delay in µS between the packets
 */
int shell_tx(int argc, char** argv) {
    (void)argc;
    (void)argv;
    uint8_t count = 1;
    uint32_t delay = 100000; // 100000us = 100ms
    if(argc > 1) {
        count = atoi(argv[1]);
    }
    if(argc > 2) {
        delay = (uint32_t)atol(argv[2]);
    }
    msg_t msg;
    msg.type = 1337;
    msg.content.value = 0;
    //printf("Sending %u packets every %luus\n", count, delay);
    while(count-- > 0) {
        msg_send(&msg, send_thread_pid);
        xtimer_usleep( delay );
    }
    return 0;
}

int shell_payload(int argc, char** argv) {
    /// argv[0] is the function name
    if(argc == 1) {
        printf("Payload is: '%s'\n", eval_message.payload);
    } else {
        strcpy((char*)eval_message.payload, argv[1]);
        eval_message.payload_length = strlen(argv[1]);
        //eval_message.payload_length = strncpy( (char*)eval_message.payload, argv[1], MAX_PAYLOAD_LENGTH);
    }
    return 0;
}

int shell_memdump(int argc, char** argv) {
    (void) argc;
    (void) argv;
    //irq_disable();
    /// TODO(rh): dump memory
    const uint8_t *addr = (const uint8_t*)0x1800;
    while(addr < (const uint8_t*)0x2600) {
        uint8_t i = 0;
        printf("%04X: ",(unsigned int)addr);
        while(i<0xF) {
            printf("%02X ", (unsigned int)(*(addr+i)));
            i++;
        }
        i = 0;
        while(i<0xF) {
            char c = *(addr+i);
            printf("%c", (c >= ' ' && c <= '~') ? c : '.');
            i++;
        }
        addr += 0xF;
        puts("");
    }
    //irq_enable();
    return 0;
}

#ifdef BOARD_INGA_BLUE
int shell_temp(int argc, char** argv) {
    (void) argc;
    (void) argv;

    // uint8_t result;
    uint8_t result[2];

	i2c_acquire(0);
    //i2c_read_reg(0, 0x48, 0x00, &result);
	i2c_read_regs(0, 0x48, 0x00, result, 2, 0);
	i2c_release(0);
    //uint16_t temp = (result[0] << 8) | (result[1] >> 4);
	//printf("temperature is %d %d\n", result[0], result[1]);
    //printf("temperature=%u\n", temp);
    printf("temperature=%u.%d\n", result[0], (result[1]>>4) );

    return 0;
}
#endif

shell_command_t eval_shell_commands[] = {
    {"tx", "Sends packets", shell_tx},
    {"payload", "Sets or prints the payload", shell_payload},
    {"memdump", "Prints the memory", shell_memdump},
#ifdef BOARD_INGA_BLUE
    {"temp", "Read temperatur", shell_temp},
#endif
    {NULL, NULL, NULL}
};

/*
static void _dump(gnrc_pktsnip_t *pkt) {
    gnrc_pktsnip_t *snip = pkt;
    gnrc_pktsnip_t *payload = NULL;
    uint8_t *data;
    eval_message_t *packet;
    gnrc_netif_hdr_t *hdr;
    uint8_t len;
    char serialbuffer[256];

    printf("serialbuffer: from %p to %p\n", serialbuffer, serialbuffer+256);
    
    // printf("_dump: %d %d\n", gnrc_netreg_num(GNRC_NETTYPE_UNDEF, 0), gnrc_netreg_num(GNRC_NETTYPE_NETIF, 0));
    while(snip != NULL) {
        switch(snip->type)  {
            case GNRC_NETTYPE_UNDEF :
                payload = snip;
            break;
            case GNRC_NETTYPE_NETIF :
                hdr = snip->data;
                //printf("HDR: %d %d %d\n", hdr->rssi, hdr->lqi, hdr->crc_valid);
                //puts("OK");
                
                packet = (eval_message_t*)payload->data;
                
                if(hdr->crc_valid) {
                    if(strcmp(packet->magic_string, MAGIC_STRING) == 0) {
                        puts("<magic:found");
                        int length = sprintf(serialbuffer, "%u|%u|%d|%u", packet->node_id, packet->seq_nr, hdr->rssi, hdr->lqi);
                        if(length > 0) {
                            printf("<magic:ok=%u/%u\n", length, strlen( serialbuffer ));
                            uint8_t sum = 0;
                            for(uint8_t i=0; i<length; i++) {
                                sum += serialbuffer[i];
                            }
                            printf("<%s=%02X\n", serialbuffer, sum);
                        }
                        else {
                            printf("<magic:length_error:%d\n", length);
                        }
                    } else {
                        printf("<valid\n");
                        /// This is another packet, ignore it!
                    }
                } else {
                    /// Make sure a corrupt packet does not exceed payload size
                    len = payload->size;
                    if(len > sizeof(eval_message_t)) {
                        len = sizeof(eval_message_t);
                    }
                    /// Pointer to data
                    data = (uint8_t*)payload->data;
                    uint8_t i,
                            sum = 0,
                            length = 0;
                    int tmp;
                    tmp = snprintf(&serialbuffer[length], sizeof(serialbuffer) - length, "%d|%u|%u|", hdr->rssi, hdr->lqi, len);
                    if(tmp > 0) {
                        length += tmp;
                        /// Print each single byte
                        for(i=0; i<len; i++) {
                            /// Even though " %02X" is only three characters long, we need to pass 4 because of the null terminator for the string!
                            tmp = snprintf(&serialbuffer[length], sizeof(serialbuffer) - length, " %02X", data[i]);
                            if(tmp > 0) {
                                length += tmp;
                            } else {
                                puts("!length error 2");
                                break;
                            }
                        }

                        for(i=0; i<length; i++) {
                            sum += serialbuffer[i];
                        }
                        printf("<%s=%02X\n", serialbuffer, sum);
                    } else {
                        puts("!length error 1");
                    }
                }
            break;
            default :
                printf("snip of type %d\n", snip->type);
            break;
        }
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
}
*/

static char serialbuffer[256];

void *dump_thread(void *arg)
{
    (void) arg;
    msg_t dump_thread_msg_queue[RCV_QUEUE_SIZE];
    msg_init_queue(dump_thread_msg_queue, RCV_QUEUE_SIZE);

    gnrc_netreg_entry_t me_reg = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL, sched_active_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &me_reg);

    msg_t msg;
    gnrc_pktsnip_t *snip = NULL;
    gnrc_pktsnip_t *payload = NULL;
    uint8_t *data;
    eval_message_t *packet;
    gnrc_netif_hdr_t *hdr;
    uint8_t len;
    memset(serialbuffer, sizeof(serialbuffer), 0x00);

    while(1) {
        if(msg_receive(&msg) != 1) {
            puts("Unable to receive message");
            continue;
        }
        //printf("[dump_thread] message received: %d\n", msg.type);
        switch(msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV :
                //irq_disable();
                //_dump( msg.content.ptr );

                snip = msg.content.ptr;
                payload = NULL;
                
                // printf("_dump: %d %d\n", gnrc_netreg_num(GNRC_NETTYPE_UNDEF, 0), gnrc_netreg_num(GNRC_NETTYPE_NETIF, 0));
                while(snip != NULL) {
                    switch(snip->type)  {
                        case GNRC_NETTYPE_UNDEF :
                            payload = snip;
                        break;
                        case GNRC_NETTYPE_NETIF :
                            hdr = snip->data;
                            //printf("HDR: %d %d %d\n", hdr->rssi, hdr->lqi, hdr->crc_valid);
                            //puts("OK");
                            if(payload == NULL) {
                                puts("!whoops? NULL pointer");
                            }
                            packet = (eval_message_t*)payload->data;
                            
                            if(hdr->crc_valid) {
                                if(strcmp(packet->magic_string, MAGIC_STRING) == 0) {
                                    puts("<magic:found");
                                    int length = sprintf(serialbuffer, "%u|%u|%d|%u", packet->node_id, packet->seq_nr, hdr->rssi, hdr->lqi);
                                    if(length > 0) {
                                        printf("<magic:ok=%u/%u\n", length, strlen( serialbuffer ));
                                        uint8_t sum = 0;
                                        for(uint8_t i=0; i<length; i++) {
                                            sum += serialbuffer[i];
                                        }
                                        printf("<%s=%02X\n", serialbuffer, sum);
                                    }
                                    else {
                                        printf("<magic:length_error:%d\n", length);
                                    }
                                } else {
                                    printf("<valid\n");
                                    /// This is another packet, ignore it!
                                }
                            } else {
                                /* Make sure a corrupt packet does not exceed payload size */
                                len = payload->size;
                                if(len > sizeof(eval_message_t)) {
                                    len = sizeof(eval_message_t);
                                }
                                /* Pointer to data */
                                data = (uint8_t*)payload->data;
                                uint8_t i,
                                        sum = 0,
                                        length = 0;
                                int tmp;
                                tmp = snprintf(&serialbuffer[length], sizeof(serialbuffer) - length, "%d|%u|%u|", hdr->rssi, hdr->lqi, len);
                                if(tmp > 0) {
                                    length += tmp;
                                    /* Print each single byte */
                                    for(i=0; i<len; i++) {
                                        /// Even though " %02X" is only three characters long, we need to pass 4 because of the null terminator for the string!
                                        tmp = snprintf(&serialbuffer[length], sizeof(serialbuffer) - length, " %02X", data[i]);
                                        if(tmp > 0) {
                                            length += tmp;
                                        } else {
                                            puts("!length error 2");
                                            break;
                                        }
                                    }

                                    for(i=0; i<length; i++) {
                                        sum += serialbuffer[i];
                                    }
                                    printf("<%s=%02X\n", serialbuffer, sum);
                                } else {
                                    puts("!length error 1");
                                }
                            }
                        break;
                        default :
                            printf("snip of type %d\n", snip->type);
                        break;
                    }
                    snip = snip->next;
                }
                gnrc_pktbuf_release(msg.content.ptr);

                //irq_enable();
            break;
            default :
                puts("ERROR: Unknown message type???");
                //gnrc_pktbuf_release( msg.content.ptr );
            break;
        }
    }
    puts("END OF dump_thread");
    return NULL;
}

static char serialbuffer2[32];

void *send_thread(void *arg)
{
    gnrc_netif_t *ieee802154_netif = arg;

    /*
    struct iovec vector[2];
    uint8_t hdr[IEEE802154_MAX_HDR_LEN];
    uint8_t src[2] = {1,1};
    uint8_t dst[2] = {0xFF, 0xFF};
    //uint8_t pan[2] = {0x12, 0x34};
    le_uint16_t pan = {.u16 = 0x1234};
    uint8_t seq = 0;

    int res = ieee802154_set_frame_hdr(hdr, src, 2, dst, 2, pan, pan, NETDEV_IEEE802154_RAW | IEEE802154_FCF_TYPE_DATA, seq);

    vector[0].iov_base = hdr;
    vector[0].iov_len = (size_t)res;

    vector[1].iov_base = data;
    vector[1].iov_len = size;
    */

    size_t addr_len = 0;
    uint8_t addr[GNRC_NETIF_L2ADDR_MAXLEN];
    gnrc_pktsnip_t *pkt, *hdr;
    gnrc_netif_hdr_t *nethdr;

    netopt_enable_t enable = true;
    netopt_enable_t disable = false;
    int ret;
    ret = gnrc_netapi_set(ieee802154_netif->pid, NETOPT_PROMISCUOUSMODE, 0, &enable, sizeof(netopt_enable_t));
    if(ret < 0) {
        puts("Unable to set promiscous mode");
    }
    ret = gnrc_netapi_set(ieee802154_netif->pid, NETOPT_AUTOACK, 0, &disable, sizeof(netopt_enable_t));
    if(ret < 0) {
        puts("Unable to disable auto ack");
    }
    ret = gnrc_netapi_set(ieee802154_netif->pid, NETOPT_CSMA, 0, &disable, sizeof(netopt_enable_t));
    if(ret < 0) {
        puts("Unable to disable CSMA");
    }

    uint8_t retrans = 0;
    ret = gnrc_netapi_set(ieee802154_netif->pid, NETOPT_RETRANS, 0, &retrans, sizeof(retrans));
    if(ret < 0) {
        puts("Unable to set retrans = 0");
    }

    int8_t retries = 7;
    ret = gnrc_netapi_set(ieee802154_netif->pid, NETOPT_CSMA_RETRIES, 0, &retries, sizeof(retries));
    if(ret < 0) {
        puts("Unable to set CSMA retries = 0");
    }

    uint8_t flags = 0 | GNRC_NETIF_HDR_FLAGS_BROADCAST;
    eval_message.payload_length = snprintf((char*)eval_message.payload, MAX_PAYLOAD_LENGTH, "Hello from %2d", NODE_ID);

    msg_t send_thread_msg_queue[RCV_QUEUE_SIZE];
    msg_init_queue(send_thread_msg_queue, RCV_QUEUE_SIZE);
    msg_t msg;
    
    memset(serialbuffer, sizeof(serialbuffer2), 0x00);
    
    while(1) {
        msg_receive(&msg);
        (void)msg;

        /*
        //printf("[dump_thread] message received: %d\n", msg.type);
        switch(msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV :
                _dump( msg.content.ptr );
            break;
            default :
                puts("ERROR: Unknown message type???");
                /// gnrc_pktbuf_release( msg.content.ptr );
            break;
        }

        xtimer_sleep(SEND_INTERVAL);	
        */

        //printf("send... %d", sizeof(eval_message) - MAX_PAYLOAD_LENGTH + payload_length);
        //printf("send...");
        /* Prepare packet */
        eval_message.seq_nr += 1;
        
        pkt = gnrc_pktbuf_add(NULL, &eval_message, sizeof(eval_message) - MAX_PAYLOAD_LENGTH + eval_message.payload_length, GNRC_NETTYPE_UNDEF);
        if (pkt == NULL) {
            puts("ERROR: packet buffer full");
            return NULL;
        }

        hdr = gnrc_netif_hdr_build(NULL, 0, addr, addr_len);
        if (hdr == NULL) {
            puts("ERROR: packet buffer full");
            gnrc_pktbuf_release(pkt);
            return NULL;
        }
        LL_PREPEND(pkt, hdr);
        nethdr = (gnrc_netif_hdr_t *)hdr->data;
        nethdr->flags = flags;
        ret = gnrc_netapi_send(ieee802154_netif->pid, pkt);
        if (ret < 1) {
            printf("ERROR: unable to send: %d\n", ret);
            gnrc_pktbuf_release(pkt);
        } else {
            //irq_disable();
            uint8_t sum = 0;
            uint8_t length = snprintf(serialbuffer2, sizeof(serialbuffer2), "%u", eval_message.seq_nr);
            for(uint8_t i=0; i<length; i++) {
                sum += serialbuffer2[i];
            }
            printf(">%s=%02X\n", serialbuffer2, sum);
            //irq_enable();
        }
    }
    return NULL;
}

int main(void)
{
    (void) puts("Welcome to RIOT!");
    /// +1 -> INGA working, but TelosB/Sky not
    thread_create(dump_thread_stack, sizeof(dump_thread_stack), THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST, dump_thread, NULL, "dump_thread");

    //int res = i2c_init(I2C_DEV(0));
    i2c_init(I2C_DEV(0));
    //printf("i2c init: %s\n", res == 0 ? "OK\n" : "FAILED\n");
    
    gnrc_netif_t *netif = NULL;
    if((netif = gnrc_netif_iter(netif))) {
        gnrc_netif_t *ieee802154_netif = netif;
        printf("Found gnrc netif: %d %d\n", netif->pid, ieee802154_netif->pid);
        /// +2 -> INGA working, but TelosB/Sky not
        send_thread_pid = thread_create(send_thread_stack, sizeof(send_thread_stack), THREAD_PRIORITY_MAIN + 2, THREAD_CREATE_STACKTEST, send_thread, ieee802154_netif, "send_thread");
    }
    else {
        puts("Unable to find netif");
    }
    
    (void) puts("Welcome to RIOT!");
    printf("This is node %d.\n", NODE_ID);

    printf("line_buf from %p to %p\n", line_buf, line_buf + sizeof(line_buf));
    printf("serialbuffer: from %p to %p\n", serialbuffer, serialbuffer+sizeof(serialbuffer));
    printf("serialbuffer2: from %p to %p\n", serialbuffer2, serialbuffer2+sizeof(serialbuffer2));

    shell_run(eval_shell_commands, line_buf, SHELL_BUFFER_SIZE);

    return 0;
}
