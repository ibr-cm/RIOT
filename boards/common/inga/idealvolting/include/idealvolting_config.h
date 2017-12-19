/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    idealvolting IdealVolting implementation for RIOT
 * @ingroup     boards_inga_common
 * @brief       Idealvolting implementation.
 * @{
 *
 * @file
 * @brief       Potentially board specific configuration for Idealvolting.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#ifndef IDEALVOLTING_CONFIG_H
#define IDEALVOLTING_CONFIG_H

/**
 * @brief On the Tiny, synchronize the frame interval to the Megs's clock
 * on start
 */
#define USE_MEGA_CLOCK 0

/**
 * @name I2C configuration
 * @{
 */
#define IV_I2C_DEV        I2C_DEV(0)       /*< I2C device */
#define SI_I2C_ADDR       0x2d             /*< Address of the Tiny */
#define SI_I2C_SPEED      I2C_SPEED_NORMAL /*< I2C Speed */
#define TMP_ADDR          0x48             /*< Address of the temp sensor */
#define TMP_REG           0x00             /*< Register to read temp byte */
/* @} */

/**
 * @name OSCCAL boundaries
 * @brief The OSCCAL will not be set outside these values
 * @{
 */
#define IV_OSCCAL_MIN     0x9e
#define IV_OSCCAL_MAX     0xbf
/* @} */

/**
 * @brief Initial voltage register value
 */
#define IV_RESET_VREG     128

/**
 * @brief MCU self check configuration
 * @{
 */
#define MCU_CHECK()       alu_check((uint8_t) 1212987413.12)
#define MCU_CHECK_RESULT_TYPE     uint8_t
#define MCU_CHECK_RESULT_CORRECT  19
/* @} */

#endif /* IDEALVOLTING_CONFIG_H */
