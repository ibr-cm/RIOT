#include "periph_cpu.h"

#ifndef PERIPH_H
#define PERIPH_H

#define IRQ_PINS_NUMOF (7)

#define IRQ_PIN_0 (GPIO_PIN(PORT_A, 1)) //PA1
#define IRQ_PIN_1 (GPIO_PIN(PORT_A, 0)) //PA0
#define IRQ_PIN_2 (GPIO_PIN(PORT_B, 0)) //PB0
#define IRQ_PIN_3 (GPIO_PIN(PORT_B, 1)) //PB1
#define IRQ_PIN_4 (GPIO_PIN(PORT_B, 2)) //PB2
#define IRQ_PIN_5 (GPIO_PIN(PORT_B, 3)) //PB3
#define IRQ_PIN_6 (GPIO_PIN(PORT_B, 4)) //PB4

/** Used in Testbed v2 (with Undervolting HW) **/
#define POWER_MIKROC    (GPIO_PIN(PORT_C, 2)) //PC2
#define IIF_SIG         (GPIO_PIN(PORT_C, 5)) //PC5
#define IIF_RESET       (GPIO_PIN(PORT_C, 4)) //PC4
#define IIF_ENABLE      (GPIO_PIN(PORT_C, 3)) //PC3

/** Defines for better Code readability in startup and UV section.**/
#define POWER_MIKROC_SETUP  (DDRC  |=  (1 << 2))
#define POWER_MIKROC_ON     (PORTC |=  (1 << 2))
#define POWER_MIKROC_OFF    (PORTC &= ~(1 << 2))

#define IIF_RESET_SETUP     (DDRC  |=  (1 << 4))
#define IIF_RESET_ON        (PORTC |=  (1 << 4))
#define IIF_RESET_OFF       (PORTC &= ~(1 << 4))

/** Important: High and Low does not refer to the signal on the Pin, 
*   but the Signal that we are trying to read!
*   The Signals are decoupled from the mikroController using Switches,
*   So the Chip wont source itself via the Logic Signals.
*   The Pins are connected to ground via a Normal-Open Switch, that is controlled by the
*   Signal we are trying to red (IIF_ENABLE and IIF_SIG)
*   The Pullups are activated on Startup. If the Signal is HIGH, the Switch is closed and
*   the pin reads Low.
*   If the Signal is Low, the Switch is open and the Pin reads High, due to its Pullup.
**/
#define IIF_SIG_SETUP       DDRC  &= ~(1 << 5); PORTC |= (1 << 5)
#define IIF_SIG_LOW         ((PINC & 0x20) == 0x20)
#define IIF_SIG_HIGH        (!IIF_SIG_LOW)

#define IIF_ENABLE_SETUP    DDRC  &= ~(1 << 3); PORTC |= (1 << 3)
#define IIF_ENABLE_LOW      ((PINC & 0x08) == 0x08)
#define IIF_ENABLE_HIGH     (!IIF_ENABLE_LOW)

/** Define for debugpins, which are used to signal certain states of the code **/
/**
#define SIG_INIT_SETUP                (DDRA   |=  (1 << 5))
#define SIG_INIT_HIGH                 (PORTA  |=  (1 << 5))
#define SIG_INIT_LOW                  (PORTA  &= ~(1 << 5))

#define SIG_BLOCK_ONE_SETUP           (DDRA   |=  (1 << 6))
#define SIG_BLOCK_ONE_HIGH            (PORTA  |=  (1 << 6))
#define SIG_BLOCK_ONE_LOW             (PORTA  &= ~(1 << 6))

#define SIG_BLOCK_TWO_SETUP      (DDRA   |=  (1 << 7))
#define SIG_BLOCK_TWO_HIGH       (PORTA  |=  (1 << 7))
#define SIG_BLOCK_TWO_LOW        (PORTA  &= ~(1 << 7))
**/
#endif
