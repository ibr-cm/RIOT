/*
 * Copyright (C) 2017 Ulf Kulau <>
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
 * @author      Ulf Kulau <>
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include "alu_check.h"
#include "idealvolting_config.h"

#define MATRIX_SIZE 3

int alu_check(int test_data)
{
	static volatile uint8_t i, j, k;

	static volatile double a_c[MATRIX_SIZE][MATRIX_SIZE];
	static volatile double b_c[MATRIX_SIZE][MATRIX_SIZE];
	static volatile double c_c[MATRIX_SIZE][MATRIX_SIZE];
	static volatile double d_d;

	unsigned char *pf = (unsigned char *) &d_d;
	uint16_t tmp = 0;

	for (i = 0; i < MATRIX_SIZE; i++) {
		for (j = 0; j < MATRIX_SIZE; j++) {
			a_c[i][j] = test_data - (double)(i);
			b_c[i][j] = test_data * (double)(i);
		}
	}
	d_d = 0;

	for (i = 0; i < MATRIX_SIZE; i++) {
		for (j = 0; j < MATRIX_SIZE; j++) {
			for (k = 0; k < MATRIX_SIZE; k++) {
				d_d += a_c[i][k] * b_c[k][j];
			}
				c_c[j][i] = d_d;
				d_d = 0;
		}
	}

	for (i = 0; i < MATRIX_SIZE; i++) {
		for (j = 0; j < MATRIX_SIZE; j++) {
			d_d += c_c[i][j];
		}
	}
	d_d = d_d - 9820980.23;

	for(i = 0; i < sizeof(d_d); i++){
	    tmp += pf[i];
	}
	return tmp;
}
