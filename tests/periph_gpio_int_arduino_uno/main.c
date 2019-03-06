/*
 * Copyright (C) 2019 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for GPIO interrupts for the Arduino Uno
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>

#include "periph/gpio.h"
#include "arduino_pinmap.h"

static const gpio_t pins[] = {
    ARDUINO_PIN_2,  ARDUINO_PIN_3,  ARDUINO_PIN_4,  ARDUINO_PIN_5,
    ARDUINO_PIN_6,  ARDUINO_PIN_7,  ARDUINO_PIN_8,  ARDUINO_PIN_9,
    ARDUINO_PIN_10, ARDUINO_PIN_11, ARDUINO_PIN_12, ARDUINO_PIN_13,
    ARDUINO_PIN_A0, ARDUINO_PIN_A1, ARDUINO_PIN_A2, ARDUINO_PIN_A3,
    ARDUINO_PIN_A4, ARDUINO_PIN_A5,
};

static const char *pin_names[] = {
    "2",  "3",  "4",  "5",  "6",  "7",  "8",  "9",
    "10", "11", "12", "13", "A0", "A1", "A2", "A3",
    "A4", "A5",
};

static const char *irq_types[] = {
    "INT0",    "INT1",    "PCINT20", "PCINT21",
    "PCINT22", "PCINT23", "PCINT0",  "PCINT1",
    "PCINT2",  "PCINT3",  "PCINT4",  "PCINT5",
    "PCINT8",  "PCINT9",  "PCINT10", "PCINT11",
    "PCINT12", "PCINT13",
};

static const gpio_flank_t flanks[] = {
    GPIO_FALLING, GPIO_RISING, GPIO_BOTH,
};

static const char *flank_names[] = {
    "falling", "rising", "falling and rising",
};

#define PINS_NUMOF      (sizeof(pins) / sizeof(pins[0]))
#define FLANKS_NUMOF    (sizeof(flanks) / sizeof(flanks[0]))

static void callback(void *arg)
{
    unsigned idx = (unsigned)arg;
    const char *pin = pin_names[idx];
    const char *type = irq_types[idx];
    const char *flank = flank_names[idx % FLANKS_NUMOF];

    printf("INT: Pin = %s; IRQ type = %s; flank(s) = %s\n", pin, type, flank);
}

int main(void)
{
    puts("Installing interrupts...");
    for (unsigned i = 0; i < PINS_NUMOF; i++) {
        if (gpio_init_int(pins[i], GPIO_IN_PU, flanks[i % FLANKS_NUMOF],
                          callback,
                          (void *)i)) {
            printf("Failed to install interrupt for pin %s\n", pin_names[i]);
        }
    }

    puts(
        "GPIO interrupt test for Arduino UNO\n"
        "\n"
        "For all pins except RX/TX (2 - 13, A0 - A5) an interrupt service\n"
        "routine will be installed and the internal pull ups will be enabled.\n"
        "The flanks are falling, rising and both (in turns). Pull the pins\n"
        "against ground one by one and check if the console output matches\n"
        "your actions");

    return 0;
}
