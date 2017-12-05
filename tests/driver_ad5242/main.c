#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "shell.h"
#include "periph/i2c.h"
#include "ad5242.h"
#include "ad5242_params.h"

static ad5242_t ad5242_dev;

int setreg_handler(int argc, char **argv);
int getreg_handler(int argc, char **argv);

static const shell_command_t commands[] = {
	{
		"setreg",
		"Sets the register of the ad5242.",
		setreg_handler
	},
	{
		"getreg",
		"Gets the current value in the register of the ad5242 and prints it.",
		getreg_handler
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
	ad5242_set_reg(&ad5242_dev, value);
	printf("AD5242 register value set to %d\n", value);
	return 0;
}

int getreg_handler(int argc, char **argv)
{
	uint8_t value;
	(void) argc;
	(void) argv;

	value = ad5242_get_reg(&ad5242_dev);
	printf("AD5242 register value is %d\n", value);
	return 0;
}

int main(void)
{
	uint8_t res;
	char line_buf[SHELL_DEFAULT_BUFSIZE];

	res = ad5242_init(&ad5242_dev, &ad5242_params);
	if (res != 0) {
		puts("Cannot initialize ad5242.");
		return -1;
	}

	shell_run(commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
