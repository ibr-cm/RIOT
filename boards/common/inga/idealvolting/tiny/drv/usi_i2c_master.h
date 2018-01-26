/*
 * Adapted from Atmel Application Note AVR310 - Using the USI module as a TWI Master
 */

#ifndef USI_I2C_MASTER_H
#define USI_I2C_MASTER_H

#include <avr/io.h>
#include <stdint.h>

typedef enum {
	USI_TWI_SUCCESS = 0x00,            /* Transmission buffer is empty */
	USI_TWI_NO_ACK_ON_DATA = 0x05,     /* The slave did not acknowledge all data */
	USI_TWI_NO_ACK_ON_ADDRESS = 0x06,  /* The slave did not acknowledge the address */
} usi_twi_result_t;

/*
 * Initialise the usi as i2c master
 */
void i2c_init_master(void);

/*
 * Write len bytes from data to a slave with the address addr.
 */
usi_twi_result_t i2c_write_bytes(uint8_t addr, uint8_t *data, uint8_t len);

/*
 * Read len bytes from a slave with the address addr to data.
 */
usi_twi_result_t i2c_read_bytes(uint8_t addr, uint8_t *data, uint8_t len);

/*
 * Write the reg byte to a slave with the address addr and then
 * read len bytes from the slave to data.
 */
usi_twi_result_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

#endif /* USI_I2C_MASTER_H */
