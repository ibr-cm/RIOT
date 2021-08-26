#include "periph.h"
#include "pattern_vars.h"

#ifndef SRAM_UV_H
#define SRAM_UV_H

void undervolting_sleep(void);
void undervolting_restore(void);

#define UV_IRQ_PIN_COUNT    (1)

//All Pins which are connected to the IIF. Is set in sram_undervolting.c
extern const pin_t uv_irq_pins[UV_IRQ_PIN_COUNT];

#endif