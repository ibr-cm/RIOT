/*
 * Copyright (C) 2014 Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    drivers_periph_rtc RTC
 * @ingroup     drivers_periph
 * @brief       Low-level RTC (Real Time Clock) peripheral driver
 *
 * @note
 * The values used for setting and getting the time/alarm should
 * conform to the `struct tm` specification.
 * Compare: http://pubs.opengroup.org/onlinepubs/7908799/xsh/time.h.html
 *
 * # (Low-) Power Implications
 *
 * After the RTC has been initialized (i.e. after calling rtc_init()), the RTC
 * should be powered on and running. The RTC can then be powered off manually
 * at a later point in time by calling the rtc_poweroff() function. When the RTC
 * is powered back on using the rtc_poweron() function, it **should**
 * transparently continue its previously configured operation.
 *
 * On many CPUs, certain power states might need to be blocked in rtc_init(), so
 * that it is ensured that the RTC will function properly while it is enabled.
 *
 * @{
 * @file
 * @brief       Low-level RTC peripheral driver interface definitions
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 */

#ifndef PERIPH_RTC_H
#define PERIPH_RTC_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "rtc_utils.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(RIOT_EPOCH) || DOXYGEN
/**
 * @brief Earliest year of the RTC
 *
 * 01.01.$RIOT_EPOCH will be the reset value of the RTC if supported.
 *
 * Internal RTC helper functions such as @ref rtc_mktime and @ref rtc_localtime
 * will not work on dates earlier than that.
 */
#define RIOT_EPOCH (2020)
#endif

/**
 * @brief Signature for alarm Callback
 *
 * @param[in] arg           optional argument to put the callback in the right context
 */
typedef void(*rtc_alarm_cb_t)(void *arg);

/**
 * @brief Initialize RTC module
 */
void rtc_init(void);

/**
 * @brief Set RTC to given time.
 *
 * @param[in] time          Pointer to the struct holding the time to set.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_set_time(struct tm *time);

/**
 * @brief Get current RTC time.
 *
 * @param[out] time         Pointer to the struct to write the time to.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_time(struct tm *time);

/**
 * @brief Get current RTC time with millis or micros resolution.
 *
 * @param[out] time         Pointer to the struct to write the time to.
 * @param[out] millis       Pointer to the int16_t to write the milli seconds to.
 * @param[out] micros       Pointer to the int16_t to write the micro seconds to.
 * 
 * @note If the millis pointer is NULL, millis are not stored (same for micros)
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_time_micros(struct tm *time, int16_t *millis, int16_t *micros);

/**
 * @brief Get current RTC time as uint64_t timestamp.
 *
 * @param[out] timestamp         Pointer to the uint64_t to write the 
 *                               timestamp to.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_timestamp(uint64_t *timestamp);

/**
 * @brief Get current RTC time with milli second resolution
 *        as uint64_t timestamp.
 *
 * @param[out] timestamp         Pointer to the uint64_t to write the 
 *                               timestamp to.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_timestamp_millis(uint64_t *timestamp);

/**
 * @brief Get current RTC time with micro second resolution
 *        as uint64_t timestamp.
 *
 * @param[out] timestamp         Pointer to the uint64_t to write the 
 *                               timestamp to.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_timestamp_micros(uint64_t *timestamp);

/**
 * @brief Get current RTC time with sub-second component.
 *        Requires the `periph_rtc_ms` feature.
 *
 * @param[out] time         Pointer to the struct to write the time to.
 * @param[out] ms           Pointer to a variable to hold the microsecond
 *                          component of the current RTC time.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_time_ms(struct tm *time, uint16_t *ms);

/**
 * @brief Convert uint64_t timestamp to struct tm.
 *
 * @param[in] timestamp          Pointer to the uint64_t to write the timestamp to.
 * @param[out] time              Pointer to the struct tm to store the converted timestamp.
 * @param[out] millis            Pointer to the location where to store the
 *                               milli seconds of the timestamp.
 * @param[out] micros            Pointer to the location where to store the
 *                               micro seconds of the timestamp.
 * 
 * @note If millis is NULL, milli seconds are not stored. (same for micros)
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_timestamp_to_time(uint64_t *timestamp, struct tm *time, int16_t *millis, int16_t *micros);

/**
 * @brief Convert struct tm to uint64_t timestamp.
 *
 * @param[in] time                Pointer to the struct tm.
 * @param[in] millis              Pointer to the milli seconds.
 * @param[in] micros              Pointer to the micro seconds.
 * @param[out] timestamp          Pointer to the location where to store the timestamp.
 * 
 * @note If millis is NULL, milli seconds are not stored. If micros is NULL, micro seconds are nto stored. 
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_time_to_timestamp(struct tm *time, int16_t *millis, int16_t *micros, uint64_t *timestamp);

/**
 * @brief Set an alarm for RTC to the specified value.
 *
 * @note Any already set alarm will be overwritten.
 *
 * @param[in] time          The value to trigger an alarm when hit.
 * @param[in] cb            Callback executed when alarm is hit.
 * @param[in] arg           Argument passed to callback when alarm is hit.
 *
 * @note    The driver must be prepared to work with denormalized time values
 *          (e.g. seconds > 60). The driver may normalize the value, or just
 *          keep it denormalized. In either case, the timeout should occur at
 *          the equivalent normalized time.
 *
 * @retval  0           success
 * @return  -EINVAL     @p time was invalid (e.g. in the past, out of range)
 * @return  <0          other error (negative errno code to indicate cause)
 */
int rtc_set_alarm(struct tm *time, rtc_alarm_cb_t cb, void *arg);

/**
 * @brief Set an alarm for RTC to the specified value with milli or
 *        micro second resolution.
 *
 * @note Any already set alarm will be overwritten.
 *
 * @param[in] time          The value to trigger an alarm when hit.
 * @param[in] cb            Callback executed when alarm is hit.
 * @param[in] arg           Argument passed to callback when alarm is hit.
 * @param[in] millis        Milli seconds to trigger an alarm.
 * @param[in] micros        Micro seconds to trigger an alarm.
 *
 * @note    The driver must be prepared to work with denormalized time values
 *          (e.g. seconds > 60). The driver may normalize the value, or just
 *          keep it denormalized. In either case, the timeout should occur at
 *          the equivalent normalized time.
 *
 * @retval  0           success
 * @return  -EINVAL     @p time was invalid (e.g. in the past, out of range)
 * @return  <0          other error (negative errno code to indicate cause)
 */
int rtc_set_alarm_micros(struct tm *time, rtc_alarm_cb_t cb, void *arg, int16_t millis, int16_t micros);

/**
 * @brief Gets the current alarm setting
 *
 * @param[out]  time        Pointer to structure to receive alarm time
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_alarm(struct tm *time);

/**
 * @brief Clear any set alarm, do nothing if nothing set
 */
void rtc_clear_alarm(void);

/**
 * @brief Turns the RTC hardware module on
 */
void rtc_poweron(void);

/**
 * @brief Turns the RTC hardware module off
 */
void rtc_poweroff(void);

#define RTC_SmoothCalibPeriod_32sec   ((uint32_t)0x00000000) /*!<  if RTCCLK = 32768 Hz, Smooth calibation
                                                             period is 32s,  else 2^20 RTCCLK seconds */
#define RTC_SmoothCalibPeriod_16sec   ((uint32_t)0x00002000) /*!<  if RTCCLK = 32768 Hz, Smooth calibation 
                                                             period is 16s, else 2^19 RTCCLK seconds */
#define RTC_SmoothCalibPeriod_8sec    ((uint32_t)0x00004000) /*!<  if RTCCLK = 32768 Hz, Smooth calibation 
                                                             period is 8s, else 2^18 RTCCLK seconds */

#define RTC_SmoothCalibPlusPulses_Set    ((uint32_t)0x00008000) /*!<  The number of RTCCLK pulses added  
                                                                during a X -second window = Y - CALM[8:0]. 
                                                                 with Y = 512, 256, 128 when X = 32, 16, 8 */
#define RTC_SmoothCalibPlusPulses_Reset  ((uint32_t)0x00000000) /*!<  The number of RTCCLK pulses subbstited
                                                                 during a 32-second window =   CALM[8:0]. */
/**
 * @brief Performs a smooth calibration according to the datasheet.
 *
 * @param[in]  calib_period         Calibration period
 * @param[in]  plus_pulses          Adds 512, 256 or 128 pulses to the RTCCLK signal within the conf. window of 32, 16 or 8 seconds.
 * @param[in]  minus_pulses_value   Subs 512, 256 or 128 pulses from the RTCCLK signal ...
 * 
 * @note minus_pulses_value cant be greater than 512.
 * 
 */
void rtc_smooth_cal(uint32_t calib_period, uint32_t plus_pulses, uint32_t minus_pulses_value);

/**
 * @brief Prints the time stored in the struct tm.
 *
 * @param[in]  time        Pointer to the struct tm, holding the time
 * @param[in]  millis      Pointer to the milli seconds
 * @param[in]  micros      Pointer to the micro seconds
 * @param[in]  date        If 1 the date gets printed, otherwise not
 * 
 * @note If millis is NULL, the milli seconds are not printed. (Same for micros)
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_print_time(struct tm *time, int16_t *millis, int16_t *micros, uint8_t date);

/**
 * @brief Prints the time stored in the uint64_t timestamp.
 *
 * @param[in]  timestamp        Pointer to the uint64_t timestamp.

 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_print_timestamp(uint64_t *timestamp);

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_RTC_H */
/** @} */
