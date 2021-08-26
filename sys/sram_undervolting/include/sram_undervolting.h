#include <stdint.h>
#include "pattern_vars.h"
#include "periph/gpio.h"
#include "board.h"

#ifndef SRAM_UV_H
#define SRAM_UV_H

void undervolting_sleep(void);
void undervolting_restore(void);


void check_uv_irqs(void);

#define UV_IRQ_PIN_COUNT    (1)

//All Pins which are connected to the IIF. Is set in sram_undervolting.c
extern const gpio_t uv_irq_pins[UV_IRQ_PIN_COUNT];

extern gpio_isr_ctx_t uv_cb_config[UV_IRQ_PIN_COUNT];

#endif
