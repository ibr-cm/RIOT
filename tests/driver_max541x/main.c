/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
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
 * @brief       Test application for max541x driver.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "shell.h"
#include "periph/i2c.h"
#include "max541x.h"
#include "max541x_params.h"

static max541x_t max541x_dev;

int setreg_handler(int argc, char **argv);

static const shell_command_t commands[] = {
	{
		"setreg",
		"Sets the register of the max541x.",
		setreg_handler
	},
	{NULL, NULL, NULL}
};

int setreg_handler(int argc, char **argv)
{
	uint8_t value;
	if (argc != 2) {
		puts("Usage: setreg <value>");
		return -1;
	}
	value = atoi(argv[1]);
	if (max541x_set_reg(&max541x_dev, value) != 0) {
		puts("Cannot set MAX541X register value");
	} else {
		printf("MAX541X register value set to %d\n", value);
	}
	return 0;
}

int main(void)
{
	uint8_t res;
	char line_buf[SHELL_DEFAULT_BUFSIZE];

	res = max541x_init(&max541x_dev, &max541x_params);
	if (res != 0) {
		puts("Cannot initialize max541x.");
		return -1;
	}

	shell_run(commands, line_buf, SHELL_DEFAULT_BUFSIZE);
	return 0;
}
