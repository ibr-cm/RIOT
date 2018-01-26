/*
 * Adapted from Atmel Application Note AVR310 - Using the USI module as a TWI Master
 */

#include <avr/io.h>
#include <stdint.h>

// Defines controlling timing limits
#define TWI_FAST_MODE

#define SYS_CLK 1000.0  // [kHz]

#ifdef TWI_FAST_MODE  // TWI FAST mode timing limits. SCL = 100-400kHz
	#define T2_TWI    ((SYS_CLK *1300) / 1000000) + 1  // >1,3us
	#define T4_TWI    ((SYS_CLK * 600) / 1000000) + 1  // >0,6us
#else  // TWI STANDARD mode timing limits. SCL <= 100kHz
	#define T2_TWI    ((SYS_CLK *4700) / 1000000) + 1  // >4,7us
	#define T4_TWI    ((SYS_CLK *4000) / 1000000) + 1  // >4,0us
#endif

enum USI_TWI_RESULT {
	USI_TWI_SUCCESS = 0x00,            /* Transmission buffer is empty */
	USI_TWI_NO_ACK_ON_DATA = 0x05,     /* The slave did not acknowledge all data */
	USI_TWI_NO_ACK_ON_ADDRESS = 0x06,  /* The slave did not acknowledge the address */
	USI_TWI_MISSING_START_CON = 0x07,  /* Generated Start Condition not detected on bus */
	USI_TWI_MISSING_STOP_CON = 0x08    /* Generated Stop Condition not detected on bus */
};

#if defined(__AVR_AT90Mega169__) | defined(__AVR_ATmega169PA__) | \
	defined(__AVR_AT90Mega165__) | defined(__AVR_ATmega165__) | \
	defined(__AVR_ATmega325__) | defined(__AVR_ATmega3250__) | \
	defined(__AVR_ATmega645__) | defined(__AVR_ATmega6450__) | \
	defined(__AVR_ATmega329__) | defined(__AVR_ATmega3290__) | \
	defined(__AVR_ATmega649__) | defined(__AVR_ATmega6490__)
	#define DDR_USI             DDRE
	#define PORT_USI            PORTE
	#define PIN_USI             PINE
	#define PORT_USI_SDA        PORTE5
	#define PORT_USI_SCL        PORTE4
	#define PIN_USI_SDA         PINE5
	#define PIN_USI_SCL         PINE4
#endif

#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__) | \
    defined(__AVR_AT90Tiny26__) | defined(__AVR_ATtiny26__)
	#define DDR_USI             DDRB
	#define PORT_USI            PORTB
	#define PIN_USI             PINB
	#define PORT_USI_SDA        PORTB0
	#define PORT_USI_SCL        PORTB2
	#define PIN_USI_SDA         PINB0
	#define PIN_USI_SCL         PINB2
#endif

#if defined(__AVR_AT90Tiny2313__) | defined(__AVR_ATtiny2313__)
	#define DDR_USI             DDRB
	#define PORT_USI            PORTB
	#define PIN_USI             PINB
	#define PORT_USI_SDA        PORTB5
	#define PORT_USI_SCL        PORTB7
	#define PIN_USI_SDA         PINB5
	#define PIN_USI_SCL         PINB7
#endif

#if defined(__AVR_ATtiny84__)
	#define DDR_USI             DDRA
	#define PORT_USI            PORTA
	#define PIN_USI             PINA
	#define PORT_USI_SDA        PA6
	#define PORT_USI_SCL        PA4
	#define PIN_USI_SDA         PINA6
	#define PIN_USI_SCL         PINA4
#endif

void i2c_init_master(void);
uint8_t i2c_write_bytes(uint8_t addr, uint8_t *data, uint8_t len);
uint8_t i2c_read_bytes(uint8_t addr, uint8_t *data, uint8_t len);
uint8_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
