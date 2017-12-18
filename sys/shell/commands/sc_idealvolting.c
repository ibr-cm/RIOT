/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell_commands
 * @{
 *
 * @file
 * @brief       Shell commands for the IdealVolting implementation in
 *              boards_inga_common.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include "idealvolting.h"
#include <stdio.h>
#include <string.h>

int _idealvolting_handler(int argc, char **argv)
{
	(void) argc;
	(void) argv;

	if (argc == 1) {
		printf("Usage: %s \n"
				"        status\n"
				"        enable\n"
				"        disable\n"
				"        debug on|off\n",
				argv[0]);
		return 1;
	} else if (strcmp("status", argv[1]) == 0) {
		idealvolting_print_status();
	} else if (strcmp("enable", argv[1]) == 0) {
		idealvolting_enable();
		puts("Idealvolting is enabled");
	} else if (strcmp("disable", argv[1]) == 0) {
		idealvolting_disable();
		puts("IdealVolting is disabled.");
	} else if (strcmp("debug", argv[1]) == 0) {
		if (argc == 3 && strcmp("on", argv[2]) == 0) {
			idealvolting_set_debug(1);
			puts("Debug mode is on");
		} else if (argc == 3 && strcmp("off", argv[2]) == 0) {
			idealvolting_set_debug(0);
			puts("Debug mode is off");
		} else {
			printf("Usage: %s debug on|off", argv[0]);
		}
	} else {
		printf("Unknown command: %s\n", argv[1]);
	}

	return 0;
}
