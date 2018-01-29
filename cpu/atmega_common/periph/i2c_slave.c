/**
 * modified https://github.com/g4lvanix/I2C-slave-lib
 */

#include "periph/i2c_slave.h"
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "cpu.h"

#ifdef MODULE_PM_LAYERED
#include "pm_layered.h"
#endif

uint8_t buffer_address, n_tx;
uint8_t buffer[I2C_SLAVE_MAX_FRAME_SIZE];
i2c_slave_rcb_t _rcb;  // Callback on data receive
i2c_slave_tcb_t _tcb;  // Callback on data transmit

void i2c_init_slave(uint8_t address, i2c_slave_rcb_t rcb, i2c_slave_tcb_t tcb)
{
	power_twi_enable();
	_rcb = rcb;
	_tcb = tcb;
	// load address into TWI address register
	TWAR = (address << 1);
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
}

void i2c_stop_slave(void)
{
	// clear acknowledge and enable bits
	TWCR &= ~((1<<TWEA) | (1<<TWEN));
	power_twi_disable();
}

ISR(TWI_vect)
{
	// temporary stores the received data
	uint8_t data;
	// own address has been acknowledged
	switch (TWSR & 0xF8) {
	case TW_SR_SLA_ACK:
		buffer_address = 0;
		// clear TWI interrupt flag, prepare to receive next byte and acknowledge
		TWCR |= (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
		break;
	case TW_SR_DATA_ACK: // data has been received in slave receiver mode
		// save the received byte inside data
		data = TWDR;
		// store the data at the current address
		buffer[buffer_address] = data;
		// increment the buffer address
		buffer_address++;
		// if there is still enough space inside the buffer
		if (buffer_address < I2C_SLAVE_MAX_FRAME_SIZE) {
			// clear TWI interrupt flag, prepare to receive next byte and acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		} else {
			// Don't acknowledge
			TWCR &= ~(1<<TWEA);
			// clear TWI interrupt flag, prepare to receive last byte.
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN);
		}
		break;
	case TW_ST_SLA_ACK:
		buffer_address = 0;
		n_tx = _tcb(buffer);
		__attribute__ ((fallthrough));
	case TW_ST_DATA_ACK: // device has been addressed to be a transmitter
		// copy the specified buffer address into the TWDR register for transmission
		TWDR = buffer[buffer_address];
		// increment buffer read address
		++buffer_address;
		// if there is another buffer address that can be sent
		if (buffer_address < n_tx) {
			// clear TWI interrupt flag, prepare to send next byte and receive acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		} else {
			// Don't acknowledge
			TWCR &= ~(1<<TWEA);
			// clear TWI interrupt flag
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN);
		}
		break;
	case TW_SR_STOP: // finished receiving data
		_rcb(buffer_address, buffer);
		// prepare TWI to be addressed again
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
		break;
	default:
		// if none of the above apply prepare TWI to be addressed again
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	}
}
