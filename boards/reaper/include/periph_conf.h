/*
 * Copyright (C) 2017 TU Braunschweig, IBR
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_inga_red
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the INGA red board
 *
 * @author      Robert Hartung <hartung@ibr.cs.tu-bs.de>
 */

#ifndef PERIPH_CONF_H_
#define PERIPH_CONF_H_



#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Clock configuration
 * @{
 */
#ifndef CLOCK_CORECLOCK
/* Using 8MHz internal oscillator as default clock source */
#define CLOCK_CORECLOCK     (8000000UL)
#endif

/**
 * The INGA has exactly 1 I2C interface
 * @{
 */
#define I2C_NUMOF           (1U)

#define I2C_0_EN            (1)
#define I2C_0_SCL           GPIO_PIN(PORT_C, 0)
#define I2C_0_SDA           GPIO_PIN(PORT_C, 1)
#define I2C_BUS_SPEED 		I2C_SPEED_NORMAL



/**
 * Pin Change Interrupt configuration
 * @{
 */
#define AVR_USE_PCINT       (1)


#ifdef __cplusplus
}
#endif

#include "periph_conf_atmega_common.h"

#endif /* PERIPH_CONF_H_ */
