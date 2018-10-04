/*
 * Copyright (C) 2018 Torben Petersen <petersen@ibr.cs.tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       This application sends idealvolting to sleep and wakes it up when it recieves a message
 *
 * @author      Torben Petersen <petersen@ibr.cs.tu-bs.de>
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
#include "idealvolting.h"

#define RCV_QUEUE_SIZE (16)

char dump_thread_stack[512+256];
kernel_pid_t main_thread_pid = 0;


static void _dump(gnrc_pktsnip_t *pkt) {
    /* Wake idealvolting up */
    
    /// Iterate though snippets (Linked List)
    gnrc_pktsnip_t *snip = pkt;
    /// Payload and header not known
    gnrc_pktsnip_t *payload = NULL;
    gnrc_netif_hdr_t *hdr = NULL;
    
    while(snip != NULL) {
        switch(snip->type)  {
            case GNRC_NETTYPE_UNDEF :
                /// No specific network type -> payload
                payload = snip;
            break;
            case GNRC_NETTYPE_NETIF :
                /// Interface data -> header
                hdr = snip->data;
                (void) hdr;
                /// Print payload
                printf("Received message: '%s'\n", (char*)payload->data);
            break;
            default : 

            break;
        }
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
}

void *dump_thread(void *arg)
{
    (void) arg;
    msg_t dump_thread_msg_queue[RCV_QUEUE_SIZE];
    msg_init_queue(dump_thread_msg_queue, RCV_QUEUE_SIZE);

    gnrc_netreg_entry_t me_reg = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL, sched_active_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &me_reg);  //alamiert den Thread sobald eine Nachricht von TYP GNRC_NETTYPE_UNDEF übers NW Interface reinkommt.

    msg_t msg; 
    while(1) {
        if(msg_receive(&msg) != 1) {
            puts("Unable to receive message");
            continue;
        }
        printf("[dump_thread] message received: %d\n", msg.type);
        switch(msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV :  //look here: https://riot-os.org/api/group__net__gnrc__netapi.html#ga57b7e8cf32c12beecc9b84ca2cc073b5
                //_dump( msg.content.ptr );
                msg_send(&msg, main_thread_pid);
            break;
        }
    }
    puts("END OF dump_thread");
    return NULL;
}

int main(void)
{
    thread_create(dump_thread_stack, sizeof(dump_thread_stack), THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST, dump_thread, NULL, "dump_thread");
    puts("created dump thread");
    main_thread_pid = thread_getpid();
    (void) puts("Welcome to RIOT!");
	while(1)
	{
		idealvolting_print_status();
        idealvolting_sleep(10); //wie bekomme ich die ampfangene Nachricht wieder aus dem IV sleep raus?
        if(wokenUpByMessage == 1)
        {
            puts("IV was woken up!");
            _dump(lastRecievedMessage.content.ptr);
        } 

	}

    return 0;
}