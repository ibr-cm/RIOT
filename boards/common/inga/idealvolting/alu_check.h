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

#ifndef IV_ALU_CHECK_H
#define IV_ALU_CHECK_H

#include <stdint.h>

/**
 * Perform matrix calculation to check the ALU
 *
 * If test_data is ((uint8_t) 1212987413.12), the result should be 19.
 *
 * @param[in] test_data Seed for the matrix generation
 * @return Result of the calculation
 */
int alu_check(int test_data);

#endif /* IV_ALU_CHECK_H */
