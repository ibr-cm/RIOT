#include <stdint.h>
#include "pattern_vars.h"
#include "periph/gpio.h"
#include "board.h"
#include <avr/sleep.h>

#ifndef SRAM_UV_H
#define SRAM_UV_H

void undervolting_sleep(void);
void undervolting_restore(void);
void undervolting_reload_routine(void);


void check_uv_irqs(void);
void gpio_restore(void);
void gpio_update_restore_table(void);
void register_uv_isr(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank, gpio_cb_t cb, void *arg);

#define UV_IRQ_PIN_COUNT    (1)

//All Pins which are connected to the IIF. Is set in sram_undervolting.c
extern const gpio_t uv_irq_pins[UV_IRQ_PIN_COUNT];

extern gpio_isr_ctx_t uv_cb_config[UV_IRQ_PIN_COUNT];

//store all reg info here for restore after UV
extern uint8_t pin_config[4][3];

//state of all pcints
extern uint8_t *pcint_state_pointer;

#endif
