/*
 * Copyright (C) 2016 Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#ifndef INGA_COMMON_PERIPH_CONF_COMMON_H_
#define INGA_COMMON_PERIPH_CONF_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief   Clock configuration
 * @{
 */
#define CLOCK_CORECLOCK     (8000000UL)
/** @} */

/**
 * @brief   Timer configuration
 *
 * The timer driver only supports the four 16-bit timers (Timer1, Timer3,
 * Timer4, Timer5), so those are the only onces we can use here.
 *
 * @{
 */
#define TIMER_NUMOF         (1U)
#define TIMER_CHANNELS      (2)

#define TIMER_0             MEGA_TIMER1
#define TIMER_0_MASK        &TIMSK1
#define TIMER_0_FLAG        &TIFR1
#define TIMER_0_ISRA        TIMER1_COMPA_vect
#define TIMER_0_ISRB        TIMER1_COMPB_vect
/** @} */

/**
 * @brief   UART configuration
 *
 * The UART devices have fixed pin mappings, so all we need to do, is to specify
 * which devices we would like to use and their corresponding RX interrupts. See
 * the reference manual for the fixed pin mapping.
 *
 * @{
 */
#define UART_NUMOF          (1U)

#define UART_0              MEGA_UART0
#define UART_0_ISR          USART0_RX_vect
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
/** @} */

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

/** @} */

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

/**
 * The INGA has exactly 1 SPI interface
 * @{
 */
#define SPI_NUMOF           (1U)
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
#endif /*  INGA_COMMON_PERIPH_CONF_COMMON_H_ */
