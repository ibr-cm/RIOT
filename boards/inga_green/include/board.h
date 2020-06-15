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
//#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

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

#if CLOCK_CORECLOCK > 4000000UL
#define XTIMER_HZ                   (CLOCK_CORECLOCK / 64)
#else
#define XTIMER_HZ                   (CLOCK_CORECLOCK / 8)
#endif

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
