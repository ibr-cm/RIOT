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
 * @brief       Idealvolting implementation.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#ifndef IDEALVOLTING_CONFIG_H
#define IDEALVOLTING_CONFIG_H

#define SI_I2C_DEV        I2C_DEV(0)
#define SI_I2C_ADDR       0x2d
#define SI_I2C_SPEED      I2C_SPEED_NORMAL

#define IV_OSCCAL_MIN     0x9e
#define IV_OSCCAL_MAX     0xbf

#define IV_RESET_VREG     128

#endif /* IDEALVOLTING_CONFIG_H */
