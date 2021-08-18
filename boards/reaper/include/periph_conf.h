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

#include "periph_conf_atmega_common.h"

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
 * Pin Change Interrupt configuration
 * @{
 */
#define AVR_USE_PCINT       (1)

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
