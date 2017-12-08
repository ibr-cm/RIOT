/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ad5242
 * @{
 *
 * @file
 * @brief       Board parameters for ad5242.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#ifndef AD5242_PARAMS_H
#define AD5242_PARAMS_H

#include "board.h"

#ifndef AD5242_PARAMS
#ifdef AD5242_PARAMS_BOARD
#define AD5242_PARAMS AD5242_PARAMS_BOARD
#else
#error "No AD5242_PARAMS defined and no AD5242_PARAMS_BOARD defined in board config."
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

static const ad5242_params_t ad5242_params = AD5242_PARAMS;

#ifdef __cplusplus
}
#endif

#endif /* AD5242_PARAMS_H */
