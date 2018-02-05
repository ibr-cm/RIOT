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

#include "../alu_check.h"

#if defined BOARD_INGA_BLUE && defined RIOT_BOARD
#include "ad5242.h"
#include "ad5242_params.h"
#define vscale_t              ad5242_t
#define VSCALE_INIT(x)        ad5242_init(x, &ad5242_params)
#define VSCALE_SET_REG(x, y)  ad5242_set_reg(x, y)
#elif defined BOARD_REAPER && defined RIOT_BOARD
#include "max541x.h"
#include "max541x_params.h"
#define vscale_t              max541x_t
#define VSCALE_INIT(x)        max541x_init(x, &max541x_params)
#define VSCALE_SET_REG(x, y)  max541x_set_reg(x, y)
#endif

/**
 * @brief On the Tiny, synchronize the frame interval to the Megs's clock
 * on start
 */
#define USE_MEGA_CLOCK  0

/**
 * @name I2C configuration
 * @{
 */
#define IV_I2C_DEV    I2C_DEV(0)       /*< I2C device */
#define SI_I2C_ADDR   0x2d             /*< Address of the Tiny */
#define SI_I2C_SPEED  I2C_SPEED_NORMAL /*< I2C Speed */
#define MEGA_SL_ADDR_SLEEP 0x2e /*< Address of the Mega when sleeping */
#define MEGA_SL_ADDR_READY 0x2f /*< Address of the Mega when ready to become master*/
#if defined BOARD_INGA_BLUE
#define TMP_ADDR      0x48             /*< Address of the temp sensor */
#define TMP_REG       0x00             /*< Register to read temp byte */
#define VSCALE_ADDR   0x2c
#define VSCALE_REG    0
#elif defined BOARD_REAPER
#define TMP_ADDR      0x76
#define TMP_REG       0x22
#define VSCALE_ADDR   0x2a
#define VSCALE_REG    0x11
#endif
/* @} */

/**
 * @name OSCCAL boundaries
 * @brief The OSCCAL will not be set outside these values
 * @{
 */
#define IV_OSCCAL_MIN  0x9e
#define IV_OSCCAL_MAX  0xbf
/* @} */

/**
 * @brief Initial voltage register value
 */
#define IV_RESET_VREG  128

/**
 * @brief MCU self check configuration
 * @{
 */
#define MCU_CHECK()               alu_check()
#define MCU_CHECK_RESULT_TYPE     alu_check_result_t
#define MCU_CHECK_RESULT_CORRECT  alu_check_result_correct
/* @} */

#endif /* IDEALVOLTING_CONFIG_H */
