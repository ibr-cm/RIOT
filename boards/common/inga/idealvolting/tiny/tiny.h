#include <stdlib.h>
#include <avr/io.h>
#include "drv/usi_i2c_master.h"
#include "drv/i2c-slave.h"
#include "../include/idealvolting_config.h"
#include "../include/idealvolting_frame.h"


/* Definitions */
struct table_entry {
	uint8_t voltage;
	uint8_t osccal;
	uint8_t info;
};

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#define SI_INIT                             0
#define SI_TRANSIENT                        1
#define SI_MAIN                             2
#define SI_IDLE                             3
#define SI_RESET                            4
#define SI_DEBUG                            5
#define SI_MASTER                           6

#define SI_STARTUP_DELAY                    7
#define SI_VOLT_REG_OFFSET                  130//128///200// für 4mhz200
#define SI_VOLT_REG_RESET                   140
#define SI_DEFAULT_VOLT_OFFSET              4
#define MATRIX_SIZE                         4
#define SI_ADAPTION_INTERVAL                5
#define SI_DELTA_T_MARGIN                   25
#define SI_TEMP_OFFSET                      25

#define SI_PREDICTION_THRESHOLD             7
#define SI_TABLE_SIZE                       51
#define SI_TABLE_VALUE_IS_EMPTY             0xFF
#define SI_TABLE_VALUE_IS_MEASURED          0x01
#define SI_TABLE_VALUE_IS_PREDICTED         0x02
#define SI_TABLE_PREDICTION                 0x03

#define EEPROM_ADDR_AVAIL                   ((void *) 0x00)
#define EEPROM_ADDR_VOFF                    ((void *) 0x01)
#define EEPROM_ADDR_TABLE                   ((void *) 0x02)

#define SI_PWR_MONITOR_ICC_ADC              ADC_CHANNEL_1
#define SI_PWR_MONITOR_VCC_ADC              ADC_CHANNEL_0

#define SI_LOCK()                           *lock_buffer = 0
#define SI_UNLOCK()                         *lock_buffer = 1
#define SI_BOOTING()                        *lock_buffer = 2

#define SI_REPLY_DEBUG_RESET()              res_buffer->debug = 0x00
#define SI_REPLY_DEBUG_STATE(current_state) res_buffer->debug |= current_state
#define SI_REPLY_DEBUG_TABLE_ENTRY()        res_buffer->debug |= (1 << 3)
#define SI_REPLY_DEBUG_RESET_HW()           res_buffer->debug |= (1 << 4)
#define SI_REPLY_DEBUG_RESET_SW()           res_buffer->debug |= (1 << 5)
#define SI_REPLY_DEBUG_TABLE_USED(info)     res_buffer->debug |= (info << 6)

#define SI_STAY_IN_DEBUG                    1

#define SI_INIT_RESET_LINE()                DDRA |= (1<<PA7)    //0b00000100 set PB2 to Output
#define SI_PULL_RESET_LINE()                PORTA |= (1<<PA7)   //0b00000100 set PB2 to 1
#define SI_RELEASE_RESET_LINE()             PORTA &= ~(1<<PA7)  // set PB2 to 0
#define MCU_HW_RESET                        0x01
#define MCU_SW_RESET                        0x02

///Deadlock Reset
#define DEADLOCK_THRESHOLD                  3 //overflows corresponds to a deadlock situation

///Digital Potentiometer Reset Software i2c
#if defined BOARD_INGA_BLUE
#define VREG_DEV_ADDR_W                     0x58
#define VREG_OP                             0x00
#elif defined BOARD_REAPER
#define VREG_DEV_ADDR_W                     0x54
#define VREG_OP                             0x11
#else
#error Define either BOARD_INGA_BLUE or BOARD_REAPER
#endif

#define SI_STEP                             10

#if USE_MEGA_CLOCK
#define DT_TARGET_SRC TCNT1;
#else
#define DT_TARGET_SRC 16000;
#endif

#define REPORT_HELLO                        "h"
#define REPORT_PERIODIC                     "p"
#define REPORT_ERROR                        "e"
#define REPORT_DEBUG                        "d"
#define REPORT_MASTER                       "m"

#define ERROR_CHECKSUM                      "s"
#define ERROR_TIMEOUT                       "t"
#define ERROR_TEMP                          "T"
#define ERROR_RESET                         "r"


void init_table(void);
void reset_voltage_level(void);
void prediction(void);
void erase_eeprom(void);
