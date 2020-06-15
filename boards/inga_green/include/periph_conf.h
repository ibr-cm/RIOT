/*
 * Copyright (C) 2017 TU Braunschweig, IBR
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_inga_green
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the INGA green board
 *
 * @author      Robert Hartung <hartung@ibr.cs.tu-bs.de>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

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
/** @} */

#ifdef __cplusplus
}
#endif

#include "periph_conf_atmega_common.h"

#endif /* PERIPH_CONF_H */
