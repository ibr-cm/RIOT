#ifndef PERIPH_I2C_SLAVE_H
#define PERIPH_I2C_SLAVE_H

void i2c_init_slave(uint8_t address);
void i2c_stop_slave(void);

extern volatile uint8_t buffer_address;
extern volatile uint8_t txbuffer[0xFF];
extern volatile uint8_t rxbuffer[0xFF];

#endif /* PERIPH_I2C_SLAVE_H */
