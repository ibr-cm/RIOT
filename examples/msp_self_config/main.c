/*
 * Copyright (C) 2018 Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "msg.h"
#include "thread.h"
#include "shell.h"
#include "shell_commands.h"
#include "xtimer.h"
#include "net/gnrc.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/netif/ieee802154.h"

extern void msp430_init_dco(void);

int main(void)
{
  puts("Welcome to RIOT! Starting Auto Calib");

	unsigned int i;
	msp430_init_dco();

	while(1)
	{
		
	}
    return 0;
}
