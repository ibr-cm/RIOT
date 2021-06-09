/*
 * Copyright (C) 2017 Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_inga_common
 * @{
 *
 * @file
 * @brief       Common board definitions for the INGA boards
 *
 * @author      Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * @}
 */

#ifndef UNIVERSAL_NODE_BOARD_H_
#define UNIVERSAL_NODE_BOARD_H_

#include "cpu.h"
#include "periph_conf.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif



#define STDIO_UART_BAUDRATE               (9600)



//#define GPIO_LIST   {GPIO_PIN(PORT_D, 6)}

#define LED1_PIN            GPIO_PIN(PORT_E, 3)
#define LED1_MASK           (1 << 3)
#define LED1_OFF            (PORTE |= LED1_MASK)
#define LED1_ON             (PORTE &= ~LED1_MASK)
#define LED1_TOGGLE         (PORTE ^= LED1_MASK)

#define LED2_PIN            GPIO_PIN(PORT_E, 4)
#define LED2_MASK           (1 << 4)
#define LED2_OFF            (PORTE |= LED2_MASK)
#define LED2_ON             (PORTE &= ~LED2_MASK)
#define LED2_TOGGLE         (PORTE ^= LED2_MASK)
/**
 * @name    xtimer configuration values
 * @{
 */
#define XTIMER_WIDTH                      (16)
#define XTIMER_BACKOFF                    (40)
#define XTIMER_DEV                        (0)
#define XTIMER_CHAN                       (0)
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
#endif /*  INGA_COMMON_BOARD_COMMON_H_ */
