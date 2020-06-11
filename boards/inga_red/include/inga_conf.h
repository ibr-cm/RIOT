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
 * @brief       Peripheral MCU configuration for the INGA red board
 *
 * @author      Robert Hartung <hartung@ibr.cs.tu-bs.de>
 */

#ifndef INGA_CONF_H
#define INGA_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Use the UART 1 for STDIO on this board
 */
#define STDIO_UART_DEV       (UART_DEV(0))
/**
 * Context swap defines
 * Setup to use PA0 which is pin change interrupt 0 (PCINT0)
 * This emulates a software triggered interrupt
 */
#define AVR_CONTEXT_SWAP_INIT do { \
            DDRA |= (1 << DDA0); \
            PCICR |= (1 << PCIE0); \
            PCMSK0 |= (1 << PCINT0); \
} while (0)
#define AVR_CONTEXT_SWAP_INTERRUPT_VECT         PCINT0_vect
#define AVR_CONTEXT_SWAP_INTERRUPT_VECT_NUM     PCINT0_vect_num
#define AVR_CONTEXT_SWAP_TRIGGER                PORTA ^= (1 << PA0)


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
 * @name    User LED pin definitions and handlers
 */
#define LED1_PIN            GPIO_PIN(PORT_D, 5)
#define LED1_MASK           (1 << 5)
#define LED1_OFF            (PORTD |= LED1_MASK)
#define LED1_ON             (PORTD &= ~LED1_MASK)
#define LED1_TOGGLE         (PORTD ^= LED1_MASK)

#define LED2_PIN            GPIO_PIN(PORT_D, 7)
#define LED2_MASK           (1 << 7)
#define LED2_OFF            (PORTD |= LED2_MASK)
#define LED2_ON             (PORTD &= ~LED2_MASK)
#define LED2_TOGGLE         (PORTD ^= LED2_MASK)
/** @} */


/**
 * @name    User button pin definitions
 */
#define BTN0_PIN            GPIO_PIN(1,2) /* PB2 */
#define BTN0_MODE           GPIO_IN
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

#endif /* INGA_CONF_H */
