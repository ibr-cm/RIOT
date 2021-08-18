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

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Use the UART 1 for STDIO on this board
 */
#define STDIO_UART_DEV       (UART_DEV(0))

/**
 * @name    STDIO configuration
 *
 * As the CPU is too slow to handle 115200 baud, we set the default
 * baudrate to 9600 for this board
 * @{
 */
#define STDIO_UART_BAUDRATE (9600U)

/** 
 * @name    User LED pin definitions and handlers
 */

#define LED1_PIN            GPIO_PIN(PORT_D, 5)
#define LED1_MASK           (1 << 5)
#define LED1_ON             (PORTD &= ~LED1_MASK)
#define LED1_OFF            (PORTD |= LED1_MASK)
#define LED1_TOGGLE         (PORTD ^= LED1_MASK)

#define LED2_PIN            GPIO_PIN(PORT_D, 7)
#define LED2_MASK           (1 << 7)
#define LED2_ON             (PORTD &= ~LED2_MASK)
#define LED2_OFF            (PORTD |= LED2_MASK)
#define LED2_TOGGLE         (PORTD ^= LED2_MASK)
/** @} */

/**
 * @brief Value that is written to the OSCCAL register on boot
 *
 * The ad5242 defaults to a MCU voltage of about 2.28V (max. 3.3V).
 * The OSCCAL has to be adapted to the current voltage.
 * 
 * TODO: figure out value for the REAPer
 */
//#define DEFAULT_OSCCAL 0xa6

/**
 * @name    ad5242 configuration
 * @{
 */
#define MAX541X_PARAMS_BOARD {.i2c_dev = I2C_DEV(0), \
                              .i2c_spd = I2C_SPEED_NORMAL, \
                              .addr = 0x2a}


/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
#endif /*  INGA_COMMON_BOARD_H_ */
