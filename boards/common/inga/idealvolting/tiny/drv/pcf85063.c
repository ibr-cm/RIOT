#include "pcf85063.h"
#include "usi_i2c_master.h"
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>

usi_twi_result_t pcf85063_init(void) {
    usi_twi_result_t res;
    /// Software Reset
    uint8_t data[] = {PCF85063_REG_CONTROL_1, 0x58};
    res = i2c_write_bytes(PCF85063_ADDR, data, sizeof(data));
    if(res != USI_TWI_SUCCESS) {
        return res;
    }

    /// STOP
    res = pcf85063_set_stop();
    if(res != USI_TWI_SUCCESS) {
        return res;
    }

    _delay_ms(10);

    /// Clear STOP
    res = pcf85063_clear_stop();

    return res;
}

usi_twi_result_t pcf85063_set_stop(void) {
    usi_twi_result_t res;
    uint8_t data[2] = {0,0};

    data[0] = PCF85063_REG_CONTROL_1;
    data[1] = 0b00100000;
    res = i2c_write_bytes(PCF85063_ADDR, data, 2);

    return res;
}

usi_twi_result_t pcf85063_clear_stop(void) {
    usi_twi_result_t res;
    uint8_t data[2] = {0,0};

    data[0] = PCF85063_REG_CONTROL_1;
    data[1] = 0b00000000;
    res = i2c_write_bytes(PCF85063_ADDR, data, 2);

    return res;
}

usi_twi_result_t pcf85063_reset_flags(void) {
    usi_twi_result_t res;
    uint8_t data[2] = {0,0};
    /// Enable alarm interrupt, clear AF and TF
    data[0] = PCF85063_REG_CONTROL_2;
    data[1] = 0b10000000;
    res = i2c_write_bytes(PCF85063_ADDR, data, 2);
    return res;
}

/**
 * Set up countdown mode
 */

usi_twi_result_t pcf85063_set_countdown(uint8_t val) {
    usi_twi_result_t res;
    uint8_t data[2] = {0,0};

    /// Set Mode
    data[0] = PCF85063_REG_TIMER_MODE;
    data[1] = 0b00010111; // 1Hz: 0b00010111 // 1/60Hz: 0b00011111
    res = i2c_write_bytes(PCF85063_ADDR, data, 2);
    if(res != USI_TWI_SUCCESS) {
        return res;
    }

    /// Set Timer value register
    data[0] = PCF85063_REG_TIMER_VALUE;
    data[1] = val;

    res = i2c_write_bytes(PCF85063_ADDR, data, 2);
    if(res != USI_TWI_SUCCESS) {
        return res;
    }

    pcf85063_reset_flags();

    return res;
}