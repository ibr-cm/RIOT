#ifndef __PCF85063_H__
#define __PCF85063_H__

#include <stdint.h>
#include "usi_i2c_master.h"

#define PCF85063_ADDR                   (0b1010001)

#define PCF85063_REG_CONTROL_1          (0x00)
#define PCF85063_REG_CONTROL_2          (0x01)
#define PCF85063_REG_OFFSET             (0x02)
#define PCF85063_REG_RAM_BYTE           (0x03)
#define PCF85063_REG_SECOND             (0x04)
#define PCF85063_REG_MINUTE             (0x05)
#define PCF85063_REG_HOURS              (0x06)
#define PCF85063_REG_DAYS               (0x07)
#define PCF85063_REG_WEEKDAYS           (0x08)
#define PCF85063_REG_MONTHS             (0x09)
#define PCF85063_REG_YEARS              (0x0A)
#define PCF85063_REG_SECOND_ALARM       (0x0B)
#define PCF85063_REG_MINUTE_ALARM       (0x0C)
#define PCF85063_REG_HOUR_ALARM         (0x0D)
#define PCF85063_REG_DAY_ALARM          (0x0E)
#define PCF85063_REG_WEEKDAY_ALARM      (0x0F)

#define PCF85063_REG_TIMER_VALUE        (0x10)
#define PCF85063_REG_TIMER_MODE         (0x11)

typedef struct __attribute__((packed)) {
    unsigned int unused : 3;
    unsigned int tcf : 2;
    unsigned int te : 1;
    unsigned int tie : 1;
    unsigned int ti_tp : 1;
} pcf85063_reg_timer_mode_t;

usi_twi_result_t pcf85063_init(void);
usi_twi_result_t pcf85063_set_countdown(uint8_t val);
usi_twi_result_t pcf85063_reset_flags(void);

usi_twi_result_t pcf85063_set_stop(void);
usi_twi_result_t pcf85063_clear_stop(void);

#endif