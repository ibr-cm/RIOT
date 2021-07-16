/*
 * Copyright (C) 2016-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_common_nucleo64 STM32 Nucleo-64
 * @ingroup     boards
 * @brief       Support for STM32 Nucleo-64 boards
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Sebastian Meiling <s@mlng.net>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"
#include "arduino_pinmap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    STDIO configuration
 *
 * Setting it down for our experiments.
 * @{
 */
#define STDIO_UART_BAUDRATE (19200U)

#define XTIMER_BACKOFF              (8)

/**
 * @name    LED pin definitions and handlers
 * @{
 */

#define LED0_PORT           GPIOB
#define LED0_PIN            GPIO_PIN(PORT_B, 12)
#define LED0_MASK           (1 << 12)

#define LED1_PORT           GPIOB
#define LED1_PIN            GPIO_PIN(PORT_B, 13)
#define LED1_MASK           (1 << 13)

#define LED2_PORT           GPIOB
#define LED2_PIN            GPIO_PIN(PORT_B, 14)
#define LED2_MASK           (1 << 14)

#define LED3_PORT           GPIOB
#define LED3_PIN            GPIO_PIN(PORT_B, 15)
#define LED3_MASK           (1 << 15)




#define LED_ON(port,mask)            (port->BSRR = mask)
#define LED_OFF(port,mask)           (port->BSRR = (mask << 16))
#define LED_TOGGLE(port,mask)        (port->ODR  ^= mask)

#define LED0_ON             LED_ON(LED0_PORT, LED0_MASK)
#define LED0_OFF            LED_OFF(LED0_PORT, LED0_MASK)
#define LED0_TOGGLE         LED_TOGGLE(LED0_PORT, LED0_MASK)

#define LED1_ON             LED_ON(LED1_PORT, LED1_MASK)
#define LED1_OFF            LED_OFF(LED1_PORT, LED1_MASK)
#define LED1_TOGGLE         LED_TOGGLE(LED1_PORT, LED1_MASK)

#define LED2_ON             LED_ON(LED2_PORT, LED2_MASK)
#define LED2_OFF            LED_OFF(LED2_PORT, LED2_MASK)
#define LED2_TOGGLE         LED_TOGGLE(LED2_PORT, LED2_MASK)

#define LED3_ON             LED_ON(LED3_PORT, LED3_MASK)
#define LED3_OFF            LED_OFF(LED3_PORT, LED3_MASK)
#define LED3_TOGGLE         LED_TOGGLE(LED3_PORT, LED3_MASK)
/** @} */

/**
 * @name    User button
 * @{
 */
#define BTN0_PIN            GPIO_PIN(PORT_C, 13)
#define BTN0_MODE           GPIO_IN_PU
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
