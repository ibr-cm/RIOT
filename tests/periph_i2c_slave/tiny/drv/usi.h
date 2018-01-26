#ifndef USI_H
#define USI_H

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

#endif /* USI_H */
