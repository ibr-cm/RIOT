/*
 * Copyright (C) 2014 Ulf Kulau
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
 * @author      Ulf Kulau
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include "alu_check.h"
#include "idealvolting_config.h"

int alu_check(int test_data)
{
	static volatile uint8_t i, j, k;

	static volatile double a_c[ALU_CHECK_N][ALU_CHECK_N];
	static volatile double b_c[ALU_CHECK_N][ALU_CHECK_N];
	static volatile double c_c[ALU_CHECK_N][ALU_CHECK_N];
	static volatile double d_d;

	unsigned char *pf = (unsigned char *) &d_d;
	uint16_t tmp = 0;

	for (i = 0; i < ALU_CHECK_N; i++) {
		for (j = 0; j < ALU_CHECK_N; j++) {
			a_c[i][j] = test_data - (double)(i);
			b_c[i][j] = test_data * (double)(i);
		}
	}
	d_d = 0;

	for (i = 0; i < ALU_CHECK_N; i++) {
		for (j = 0; j < ALU_CHECK_N; j++) {
			for (k = 0; k < ALU_CHECK_N; k++) {
				d_d += a_c[i][k] * b_c[k][j];
			}
			c_c[j][i] = d_d;
			d_d = 0;
		}
	}

	for (i = 0; i < ALU_CHECK_N; i++) {
		for (j = 0; j < ALU_CHECK_N; j++) {
			d_d += c_c[i][j];
		}
	}
	d_d = d_d - 9820980.23;

	for (i = 0; i < sizeof(d_d); i++){
		tmp += pf[i];
	}
	return tmp;
}

uint8_t sum(volatile uint8_t *data, uint8_t n, uint8_t distance)
{
	uint8_t r = 0;
	while (n)
		r += data[(--n * distance)];
	return r;
}

alu_check_alt_result_t alu_check_alt(void)
{
	static volatile uint8_t a_c[(ALU_CHECK_N + 1) * ALU_CHECK_N];
	static volatile uint8_t b_c[ALU_CHECK_N * (ALU_CHECK_N + 1)];
	static volatile uint8_t c_f[(ALU_CHECK_N + 1) * (ALU_CHECK_N + 1)];

	uint8_t r = 0;
	uint8_t i, j;

	// init A
	for (i = 0; i < (ALU_CHECK_N * ALU_CHECK_N); ++i)
		a_c[i] = r++;
	for (i = 0; i < ALU_CHECK_N; ++i)
		a_c[(ALU_CHECK_N * ALU_CHECK_N) + i] = sum((a_c + i), ALU_CHECK_N, ALU_CHECK_N);
	// init B
	for (i = 0; i < (ALU_CHECK_N * ALU_CHECK_N); ++i)
		b_c[i + (i / (ALU_CHECK_N))] = r++;
	for (i = 0; i < ALU_CHECK_N; ++i)
		b_c[i * (ALU_CHECK_N + 1) + ALU_CHECK_N] = sum((b_c + (i * (ALU_CHECK_N + 1))), ALU_CHECK_N, 1);
	// calculate C
	for (i = 0; i < ((ALU_CHECK_N + 1) * (ALU_CHECK_N + 1)); ++i) {
		c_f[i] = 0;
		for (j = 0; j < ALU_CHECK_N; ++j)
			c_f[i] += a_c[((i / (ALU_CHECK_N + 1)) * ALU_CHECK_N) + j] * b_c[(i % (ALU_CHECK_N + 1)) + (j * (ALU_CHECK_N + 1))];
	}
	return *((alu_check_alt_result_t *) c_f);
}
