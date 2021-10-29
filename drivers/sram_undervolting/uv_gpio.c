#include "sram_undervolting.h"
#include <avr/io.h>
#include "periph/gpio.h"
#include "periph_cpu.h"
#include "uv_periph.h"
#include "atmega_gpio.h"

#define F_CPU CLOCK_CORECLOCK
#include <util/delay.h>

uint8_t pin_config[4][3];

gpio_isr_ctx_t uv_cb_config[UV_IRQ_PIN_COUNT];

const gpio_t uv_irq_pins[] = {
    IIF_SIG,
};

//this is the wrapper for UV INT Pins
//just call the handler and reset the IIF

void uv_isr_wrapper(void *arg) {

    //pointer to the stored cb info is the arg!
    gpio_isr_ctx_t *cb_info = (gpio_isr_ctx_t*)(arg);

    //call the cb
    cb_info->cb(cb_info->arg);
    
    IIF_RESET_ON; //IRQ detected, reset IIF.
    IIF_RESET_OFF;

}


int get_Port(gpio_t pin) {
    return (pin >> 4);
}

int get_Pin(gpio_t pin) {
    return (pin & 0x0F);
}

void gpio_update_restore_table(void) {
    pin_config[0][0] = DDRA;
    pin_config[0][1] = PORTA;
    pin_config[0][2] = PCMSK0;

    pin_config[1][0] = DDRB;
    pin_config[1][1] = PORTB;
    pin_config[1][2] = PCMSK1;

    pin_config[2][0] = DDRC;
    pin_config[2][1] = PORTC;
    pin_config[2][2] = PCMSK2;

    pin_config[3][0] = DDRD;
    pin_config[3][1] = PORTD;
    pin_config[3][2] = PCMSK3;
    return;
}

void gpio_restore(void) {
    //use |= or the restore will take longer. Reason unknown...
    DDRA  |= pin_config[0][0];
    PORTA |= pin_config[0][1];
    
    DDRB  |= pin_config[1][0];
    PORTB |= pin_config[1][1]; 
    
    DDRC  |= pin_config[2][0];
    PORTC |= pin_config[2][1]; 

    DDRD  |= pin_config[3][0];
    PORTD |= pin_config[3][1]; 
    
    //restore IRQS
    if(pin_config[0][2] != 0x00) {
        PCICR |= 1 << PCIE0;
        PCMSK0 |= pin_config[0][2];
        pcint_state_pointer[0] = PINA;
    }
    if(pin_config[1][2] != 0x00) {
        PCICR |= 1 << PCIE1;
        PCMSK1 |= pin_config[1][2];
        pcint_state_pointer[1] = PINB;
    }
    if(pin_config[2][2] != 0x00) {
        PCICR |= 1 << PCIE2;
        PCMSK2 |= pin_config[2][2];
        pcint_state_pointer[2] = PINC;
    }
    if(pin_config[3][2] != 0x00) {
        PCICR |= 1 << PCIE3;
        PCMSK3 |= pin_config[3][2];
        pcint_state_pointer[3] = PIND;
    }
    //sei() ist nicht notwendig, da das SREG vom undervolting_sleep auf den stack gepusht wurde!
}

void check_uv_irqs(void) {
    //check which IRQ woke uC up
    //call ISR, if ISR is registered
    //look in table for all UV wakeup pins
    for(uint8_t i = 0; i < UV_IRQ_PIN_COUNT; i++) {
        if (gpio_read(uv_irq_pins[i]) == 0) {
            uint8_t port_num = get_Port(uv_irq_pins[i]);
            uint8_t pin_num  = get_Pin(uv_irq_pins[i]);

            //check if ISR on Pin is registered
            volatile uint8_t *irq_reg = 0x00;

            switch(port_num) 
            {
                case PORT_A:
                    irq_reg = &PCMSK0;
                    break;

                case PORT_B:
                    irq_reg = &PCMSK1;
                    break;

                case PORT_C:
                    irq_reg = &PCMSK2;
                    break;

                case PORT_D:
                    irq_reg = &PCMSK3;
                    break;

            }

            //check if handler is registered.
            if( (*irq_reg) & (1 << pin_num)) {
                //call the CB!
                uv_cb_config[i].cb(uv_cb_config[i].arg);
            }
        }
    }
    IIF_RESET_ON; //IRQ detected, reset IIF.
    //_delay_ms(delay); //If Pulse is too short, the IIF wont reset!
    IIF_RESET_OFF;
    
    return;
}

void register_uv_isr(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank, gpio_cb_t cb, void *arg) {
    //check if gpio is UV IRQ Pin
    for(int i = 0; i < UV_IRQ_PIN_COUNT; i++) {
        if(pin == uv_irq_pins[i]) {
            //found a match, register IRQ!
            uv_cb_config[i].cb = cb;
            uv_cb_config[i].arg = arg;

            //register normally, just in case Interrupts happen during active phase!
            gpio_init_int(pin, mode, flank, &uv_isr_wrapper, &(uv_cb_config[i]));

            //Done, break the loop
            break;
        }
    }
}