#ifndef _USI_TWI
#define _USI_TWI

//ATTINY 84 Register Definitions
		#define DDR_USI             DDRA
		#define PORT_USI            PORTA
		#define PIN_USI             PINA
		#define PORT_USI_SDA        PA6
		#define PORT_USI_SCL        PA4
		#define PIN_USI_SDA         PINA6
		#define PIN_USI_SCL         PINA4
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_STR_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect  
#endif
