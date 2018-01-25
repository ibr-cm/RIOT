#ifndef PERIPH_I2C_SLAVE_H
#define PERIPH_I2C_SLAVE_H

#include <stdint.h>

#define I2C_SLAVE_MAX_FRAME_SIZE 8

typedef void(*i2c_slave_rcb_t)(uint8_t n_received, uint8_t *data_reveiced);
typedef uint8_t(*i2c_slave_tcb_t)(uint8_t *txbuffer);

void i2c_init_slave(uint8_t address, i2c_slave_rcb_t rcb, i2c_slave_tcb_t tcb);
void i2c_stop_slave(void);

#endif /* PERIPH_I2C_SLAVE_H */
