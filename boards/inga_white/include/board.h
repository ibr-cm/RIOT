/*
 * Copyright (C) 2016 Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Use the UART 1 for STDIO on this board
 */
#define STDIO_UART_DEV       (UART_DEV(0))
/**
 * Context swap defines
 * Setup to use PA0 which is pin change interrupt 0 (PCINT0)
 * This emulates a software triggered interrupt
 */
#define AVR_CONTEXT_SWAP_INIT do { \
            DDRA |= (1 << DDA0); \
            PCICR |= (1 << PCIE0); \
            PCMSK0 |= (1 << PCINT0); \
} while (0)
#define AVR_CONTEXT_SWAP_INTERRUPT_VECT         PCINT0_vect
#define AVR_CONTEXT_SWAP_INTERRUPT_VECT_NUM     PCINT0_vect_num
#define AVR_CONTEXT_SWAP_TRIGGER                PORTA ^= (1 << PA0)


/**
 * @name    at86rf233 configuration
 * @{
 */
#define AT86RF2XX_PARAMS {.spi = SPI_DEV(0), \
                                .spi_clk = SPI_CLK_5MHZ, \
                                .cs_pin = GPIO_PIN(PORT_B, 4),\
                                .int_pin = GPIO_PIN(PORT_D, 6),\
                                .sleep_pin = GPIO_PIN(PORT_B, 3),\
                                .reset_pin = GPIO_PIN(PORT_B, 1)}
/** @} */


/**
 * @name    STDIO configuration
 *
 * As the CPU is too slow to handle 115200 baud, we set the default
 * baudrate to 9600 for this board
 * @{
 */
#define STDIO_UART_BAUDRATE (9600U)
/** @} */

/**
 * @name    xtimer configuration values
 * @{
 */
#define XTIMER_WIDTH                (16)
#define XTIMER_BACKOFF                    (40)
#define XTIMER_DEV                        (0)
#define XTIMER_CHAN                       (0)
#if CLOCK_CORECLOCK > 4000000UL
#define XTIMER_HZ                   (CLOCK_CORECLOCK / 64)
#else
#define XTIMER_HZ                   (CLOCK_CORECLOCK / 8)
#endif

/** @} */

/**
 * @name    User LED pin definitions and handlers
 */
#define LED1_PIN            GPIO_PIN(PORT_D, 5)
#define LED1_MASK           (1 << 5)
#define LED1_OFF            (PORTD |= LED1_MASK)
#define LED1_ON             (PORTD &= ~LED1_MASK)
#define LED1_TOGGLE         (PORTD ^= LED1_MASK)

#define LED2_PIN            GPIO_PIN(PORT_D, 7)
#define LED2_MASK           (1 << 7)
#define LED2_OFF            (PORTD |= LED2_MASK)
#define LED2_ON             (PORTD &= ~LED2_MASK)
#define LED2_TOGGLE         (PORTD ^= LED2_MASK)
/** @} */

/**
 * @name    User button pin definitions
 */
#define BTN0_PIN            GPIO_PIN(1,2) /* PB2 */
#define BTN0_MODE           GPIO_IN
/** @} */


/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */

