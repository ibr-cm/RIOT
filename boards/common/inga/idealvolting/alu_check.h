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

#define ALU_CHECK_N 3

typedef uint8_t alu_check_result_t;

static const uint8_t alu_check_result_correct[1] = {19};

typedef struct {
	uint8_t c_f[(ALU_CHECK_N + 1) * (ALU_CHECK_N + 1)];
} alu_check_alt_result_t;

static const uint8_t alu_check_alt_result_correct[(ALU_CHECK_N + 1) * (ALU_CHECK_N + 1)] = {
	42, 45, 48, 135, 150, 162, 174, 230, 2, 23, 44, 69, 194, 230, 10, 178
};

/**
 * Perform matrix calculation to check the ALU
 *
 * @return Result of the calculation
 */
int alu_check(void);

alu_check_alt_result_t alu_check_alt(void);

#endif /* IV_ALU_CHECK_H */
