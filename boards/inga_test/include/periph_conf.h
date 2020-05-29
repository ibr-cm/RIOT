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
 * INGA ADXL345 configuration
 * @{
 */
#define ADXL345_PARAM_ADDR  ADXL345_ADDR_53 /* (0xA6>>1) */
#define ADXL345_PARAM_I2C   (I2C_DEV(0))
#define ADXL345_PARAMS              { .i2c    = ADXL345_PARAM_I2C,       \
                                      .addr   = ADXL345_PARAM_ADDR,      \
                                      .offset = ADXL345_PARAM_OFFSET,    \
                                      .range  = ADXL345_RANGE_2G,        \
                                      .rate   = ADXL345_RATE_100HZ,      \
                                      .full_res = ADXL345_PARAM_FULL_RES }

/** @} */



/**
 * Pin Change Interrupt configuration
 * @{
 */
#define AVR_USE_PCINT       (1)
/** @} */



/**
 * INGA L3G4200D configuration
 * @{
 */
#define L3G4200D_PARAM_ADDR (0x69) /* 0xD2>>1 */
#define L3G4200D_PARAM_I2C   (I2C_DEV(0))
#define L3G4200D_PARAM_MODE L3G4200D_MODE_100_25



/**
 * INGA BMP085 configuration
 * @{
 */
#define BMP180_PARAM_ADDR (0x77) /* 0xEE>>1 */
#define BMP180_PARAM_I2C   (I2C_DEV(0))
#define BMP180_PARAMS  {.i2c_dev = BMP180_PARAM_I2C, \
						.i2c_addr = BMP180_PARAM_ADDR, \
						.oversampling = BMP180_STANDARD }
/** @} */

#ifdef __cplusplus
}
#endif

#include "periph_conf_atmega_common.h"

#endif /* PERIPH_CONF_H */
