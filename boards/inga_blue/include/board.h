/*
 * Copyright (C) 2016 Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#ifndef INGA_COMMON_BOARD_H_
#define INGA_COMMON_BOARD_H_

#include "cpu.h"
#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "board_common.h"
#include "ad5242.h"

#ifdef __cplusplus
extern "C" {
#endif

/* the blue inga does not have any LEDs or buttons */

/**
 * @brief Value that is written to the OSCCAL register on boot
 *
 * The ad5242 defaults to a MCU voltage of about 2.28V (max. 3.3V).
 * The OSCCAL has to be adapted to the current voltage.
 */
#define DEFAULT_OSCCAL 0xa6

/**
 * @name    ad5242 configuration
 * @{
 */
#define AD5242_PARAMS_BOARD {.i2c_dev = I2C_DEV(0), \
                             .i2c_spd = I2C_SPEED_NORMAL, \
                             .addr = 0x2c,\
                             .opt = AD5242_CHN_A}
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
#endif /*  INGA_COMMON_BOARD_H_ */
