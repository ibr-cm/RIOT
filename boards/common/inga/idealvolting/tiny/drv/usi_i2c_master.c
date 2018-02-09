#include "usi_i2c_master.h"
#include <avr/io.h>
#include <util/delay.h>
#include "usi.h"

#define TWI_SLOW_MODE
#define SYS_CLK 1000.0  // [kHz]

#ifdef TWI_FAST_MODE  // TWI FAST mode timing limits. SCL = 100-400kHz
	#define T2_TWI    ((SYS_CLK * 1300) / 1000000) + 1  // >1.3us
	#define T4_TWI    ((SYS_CLK * 600) / 1000000) + 1  // >0.6us
#elif defined TWI_SLOW_MODE // Non-standard SLOW mode. SCL >= 10kHz
	#define T2_TWI    ((SYS_CLK * 47000) / 1000000) + 1  // >47.0us
	#define T4_TWI    ((SYS_CLK * 40000) / 1000000) + 1  // >40.0us
#else  // TWI STANDARD mode timing limits. SCL <= 100kHz
	#define T2_TWI    ((SYS_CLK * 4700) / 1000000) + 1  // >4.7us
	#define T4_TWI    ((SYS_CLK * 4000) / 1000000) + 1  // >4.0us
#endif

uint8_t _i2c_write_byte(uint8_t data);
void _i2c_write_bit(uint8_t data);
uint8_t _i2c_read_byte(uint8_t ack);
uint8_t _i2c_read_bit(void);
uint8_t _i2c_transfer(void);
void _i2c_start(void);
void _i2c_stop(void);

/*
 * Initialise the usi as i2c master
 */
void i2c_init_master(void)
{
	PORT_USI |= (1 << PIN_USI_SDA);  // Enable pullup on SDA, to set high as released state.
	PORT_USI |= (1 << PIN_USI_SCL);  // Enable pullup on SCL, to set high as released state.

	DDR_USI  |= (1 << PIN_USI_SCL);  // Enable SCL as output.
	DDR_USI  |= (1 << PIN_USI_SDA);  // Enable SDA as output.

	USIDR    =  0xFF;  // Preload dataregister with "released level" data.
	USICR    =  (0 << USISIE) | (0 << USIOIE)  // Disable Interrupts.
			| (1 << USIWM1) | (0 << USIWM0)  // Set USI in Two-wire mode.
			| (1 << USICS1) | (0 << USICS0) | (1 << USICLK)  // Software stobe as counter clock source
			| (0 << USITC);
	USISR   =   (1 << USISIF) | (1 << USIOIF)
			| (1 << USIPF) | (1 << USIDC)  // Clear flags,
			| (0x0 << USICNT0);  // and reset counter.
}

/*
 * Write len bytes from data to a slave with the address addr.
 */
usi_twi_result_t i2c_write_bytes(uint8_t addr, uint8_t *data, uint8_t len)
{
	_i2c_start();
	if (!_i2c_write_byte(addr << 1))
		return USI_TWI_NO_ACK_ON_ADDRESS;
	while (len--) {
		if (!_i2c_write_byte(*(data++)))
			return USI_TWI_NO_ACK_ON_DATA;
	}
	_i2c_stop();
	return USI_TWI_SUCCESS;
}

/*
 * Read len bytes from a slave with the address addr to data.
 */
usi_twi_result_t i2c_read_bytes(uint8_t addr, uint8_t *data, uint8_t len)
{
	_i2c_start();
	if (!_i2c_write_byte((addr << 1) | 1))
		return USI_TWI_NO_ACK_ON_ADDRESS;
	while (len--)
		*(data++) = _i2c_read_byte(len);
	_i2c_stop();
	return USI_TWI_SUCCESS;
}

/*
 * Write the reg byte to a slave with the address addr and then
 * read len bytes from the slave to data.
 */
usi_twi_result_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	_i2c_start();
	if (!_i2c_write_byte(addr << 1))
		return USI_TWI_NO_ACK_ON_ADDRESS;
	if (!_i2c_write_byte(reg))
		return USI_TWI_NO_ACK_ON_DATA;
	_i2c_start();
	if (!_i2c_write_byte((addr << 1) | 1))
		return USI_TWI_NO_ACK_ON_ADDRESS;
	while (len--)
		*(data++) = _i2c_read_byte(len);
	_i2c_stop();
	return USI_TWI_SUCCESS;
}

uint8_t _i2c_write_byte(uint8_t data)
{
	PORT_USI &= ~(1 << PIN_USI_SCL);  // Pull SCL LOW.
	USISR = (1 << USISIF) | (1 << USIOIF)
			| (1 << USIPF) | (1 << USIDC)  // Prepare register value to: Clear flags, and
			| (0x0 << USICNT0);  // set USI to shift 8 bits i.e. count 16 clock edges.
	USIDR = data;
	_i2c_transfer();
	return (!_i2c_read_bit());
}

void _i2c_write_bit(uint8_t data)
{
	USIDR = data;
	USISR = (1 << USISIF) | (1 << USIOIF)
			| (1 << USIPF) | (1 << USIDC)  // Prepare register value to: Clear flags, and
			| (0xE << USICNT0);  // set USI to shift 1 bit i.e. count 2 clock edges.
	_i2c_transfer();
}

uint8_t _i2c_read_byte(uint8_t ack)
{
	DDR_USI  &= ~(1 << PIN_USI_SDA);  // Enable SDA as input.
	uint8_t data;
	USISR = (1 << USISIF) | (1 << USIOIF)
			| (1 << USIPF) | (1 << USIDC)  // Prepare register value to: Clear flags, and
			| (0x0 << USICNT0);  // set USI to shift 8 bits i.e. count 16 clock edges.
	data = _i2c_transfer();
	_i2c_write_bit(ack ? 0x00 : 0xFF);
	return data;
}

uint8_t _i2c_read_bit(void)
{
	DDR_USI  &= ~(1 << PIN_USI_SDA);  // Enable SDA as input.
	USISR = (1 << USISIF) | (1 << USIOIF)
			| (1 << USIPF) | (1 << USIDC)  // Prepare register value to: Clear flags, and
			| (0xE << USICNT0);  // set USI to shift 1 bit i.e. count 2 clock edges.
	return _i2c_transfer() & 1;
}

uint8_t _i2c_transfer(void)
{
        /* Prepare clocking. */
	uint8_t temp  =  (0 << USISIE) | (0 << USIOIE) |  // Interrupts disabled
	         (1 << USIWM1) | (0 << USIWM0) |  // Set USI in Two-wire mode.
	         (1 << USICS1) | (0 << USICS0) | (1 << USICLK) |  // Software clock strobe as source.
	         (1 << USITC);  // Toggle Clock Port.
	do {
		_delay_us(T2_TWI / 4);
		USICR = temp;  // Generate positve SCL edge.
		while (!(PIN_USI & (1 << PIN_USI_SCL)));  // Wait for SCL to go high.
		_delay_us( T4_TWI / 4 );
		USICR = temp;  // Generate negative SCL edge.
	} while (!(USISR & (1 << USIOIF)));  // Check for transfer complete.

	_delay_us(T2_TWI / 4);
	temp = USIDR;  // Read out data.
	USIDR = 0xFF;  // Release SDA.
	DDR_USI |= (1 << PIN_USI_SDA);  // Enable SDA as output.

	return temp;  // Return the data from the USIDR
}

void _i2c_start(void)
{
	/* Release SCL to ensure that (repeated) Start can be performed */
	PORT_USI |= (1 << PIN_USI_SCL);  // Release SCL.
	while (!(PIN_USI & (1 << PIN_USI_SCL)));  // Verify that SCL becomes high.
#ifdef TWI_FAST_MODE
	_delay_us(T4_TWI/4);
#else
	_delay_us(T2_TWI/4);
#endif
	/* Generate Start Condition */
	PORT_USI &= ~(1 << PIN_USI_SDA);  // Force SDA LOW.
	_delay_us(T4_TWI / 4);
	PORT_USI &= ~(1 << PIN_USI_SCL);  // Pull SCL LOW.
	PORT_USI |= (1 << PIN_USI_SDA);  // Release SDA.
}

void _i2c_stop(void)
{
	PORT_USI &= ~(1 << PIN_USI_SDA);  // Pull SDA low.
	PORT_USI |= (1 << PIN_USI_SCL);  // Release SCL.
	while (!(PIN_USI & (1 << PIN_USI_SCL)));  // Wait for SCL to go high.
	_delay_us(T4_TWI / 4);
	PORT_USI |= (1 << PIN_USI_SDA);  // Release SDA.
	_delay_us(T2_TWI / 4);
}
