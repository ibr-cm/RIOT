/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     idealvolting
 * @{
 *
 * @file
 * @brief       Network frame definitions for Idealvolting implementation.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#ifndef IV_FRAME_H
#define IV_FRAME_H

#include <stdint.h>
#include "idealvolting_config.h"

#define REQUEST_FRAME_SIZE sizeof(iv_req_t)
#define REPLY_FRAME_SIZE (sizeof(iv_res_t) + 1)

/**
 * @name Virtual register addresses when reading from the Tiny.
 * @{
 */
#define SI_REG_LOCK       0x00 /*< Lock state */
#define SI_REG_REPLY      0x01 /*< Reply frame */
/* @} */

/**
 * @name Virtual register addresses when writing to the Tiny.
 * @{
 */
#define SI_REG_REQUEST    0x00 /*< Request frame */
/** @} */

/**
 * @name SI lock states.
 * @{
 */
enum {
	SI_BUSY = 0,
	SI_READY = 1,
	SI_BOOTING = 2
};
/* @} */

/**
 * @brief Data type of the MCU check result
 */
typedef MCU_CHECK_RESULT_TYPE mcu_check_result_t;

/**
 * @brief Request frame sent to the Tiny
 */
typedef struct iv_req {
	mcu_check_result_t checksum;    /**< MCU self test result */
	int8_t temperature;  /**< Current temperature in °C */
	uint8_t osccal;      /**< Current OSCCAL value */
	uint8_t rst_flags;   /**< Hardware/Software resets */
	uint8_t alt_byte;    /**< Alternating byte */
	uint8_t rst_disable; /**< Disable resets */
} iv_req_t;

/**
 * @brief Response frame read from the Tiny
 */
typedef struct iv_res {
	uint8_t osccal;  /**< Suggested OSCCAL value */
	uint8_t voltage; /**< Suggested voltage register value */
	uint16_t dt;    /**< Ticks between last requests */
	uint8_t debug;   /**< Debug byte */
} iv_res_t;

#endif /* IV_FRAME_H */
