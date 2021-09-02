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
#define POWER_MIKROC    (IRQ_PIN_3) //PB1
#define IIF_SIG         (IRQ_PIN_5) //PB5
#define IIF_RESET       (IRQ_PIN_1) //PA0
#define IIF_ENABLE      (IRQ_PIN_2) //PB0

/** Defines for better Code readability in startup and UV section.**/
#define POWER_MIKROC_SETUP  (DDRB  |=  (1 << 1))
#define POWER_MIKROC_ON     (PORTB |=  (1 << 1))
#define POWER_MIKROC_OFF    (PORTB &= ~(1 << 1))

#define IIF_RESET_SETUP     (DDRA  |=  (1 << 0))
#define IIF_RESET_ON        (PORTA |=  (1 << 0))
#define IIF_RESET_OFF       (PORTA &= ~(1 << 0))

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
#define IIF_SIG_SETUP       DDRB  &= ~(1 << 3); PORTB |= (1 << 3)
#define IIF_SIG_LOW         ((PINB & 0x08) == 0x08)
#define IIF_SIG_HIGH        (!IIF_SIG_LOW)

#define IIF_ENABLE_SETUP    DDRB  &= ~(1 << 0); PORTB |= (1 << 0)
#define IIF_ENABLE_LOW      ((PINB & 0x01) == 0x01)
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
