/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_max541x
 * @{
 *
 * @file
 * @brief       Board parameters for max541x.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#ifndef MAX541X_PARAMS_H
#define MAX541X_PARAMS_H

#include "board.h"

#ifndef MAX541X_PARAMS
#ifdef MAX541X_PARAMS_BOARD
#define MAX541X_PARAMS MAX541X_PARAMS_BOARD
#else
#error "No MAX541X_PARAMS defined and no MAX541X_PARAMS_BOARD defined in board config."
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

static const max541x_params_t max541x_params = MAX541X_PARAMS;

#ifdef __cplusplus
}
#endif

#endif /* MAX541X_PARAMS_H */
