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

#ifndef IDEALVOLTING_H
#define IDEALVOLTING_H

/**
 * Initialize IdealVolting
 *
 * This will start the idealvolting thread with priority 0
 * IdealVolting will be enabled immediately
 */
void idealvolting_init(void);

/**
 * Enable Idealvolting
 *
 * If it was disabled, IdealVolting will be enabled and return to
 * the minimal voltage.
 */
void idealvolting_enable(void);

/**
 * Disable Idealvolting
 *
 * This should raise the voltage to the initial value and disable
 * resets on the Tiny.
 * Currently this will only pause the idealvolting thread, and the
 * Tiny will reset the node.
 */
void idealvolting_disable(void);

/**
 * Print information about the current state of IdealVolting
 */
void idealvolting_print_status(void);

#endif /* IDEALVOLTING_H */
