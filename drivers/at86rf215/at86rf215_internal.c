/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#include "periph/spi.h"
#include "periph/gpio.h"
#include "xtimer.h"
#include "at86rf215_internal.h"
#include "at86rf215_registers.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define SPIDEV          (dev->params.spi)
#define CSPIN           (dev->params.cs_pin)

static inline void getbus(const at86rf2xx_t *dev)
{
    spi_acquire(SPIDEV, CSPIN, SPI_MODE_0, dev->params.spi_clk);
}

void at86rf215_reg_write(const at86rf2xx_t *dev, uint16_t addr, uint8_t value)
{
    uint16_t cmd = (AT86RF215_ACCESS_WRITE | addr);

    getbus(dev);
	/*** must be MSB first ***/
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd>>8));
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd));
	//spi_transfer_bytes(SPIDEV, CSPIN, true, &cmd, NULL, 2);
	spi_transfer_bytes(SPIDEV, CSPIN, false, &value, NULL, 1);
    spi_release(SPIDEV);
}

uint8_t at86rf215_reg_read(const at86rf2xx_t *dev, uint16_t addr)
{
    uint16_t cmd = (AT86RF215_ACCESS_READ | addr);
    uint8_t value;

    getbus(dev);
	/*** must be MSB first ***/
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd>>8));
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd));
	value = spi_transfer_byte(SPIDEV, CSPIN, false, 0);
    spi_release(SPIDEV);

    return value;
}

void at86rf215_txfb_write(const at86rf2xx_t *dev, uint8_t offset,
                          const uint8_t *data, size_t len)
{
	uint16_t cmd = AT86RF215_ACCESS_WRITE;
	if(dev->rf == _RF24_) {
		cmd |= (AT86RF215_REG__BBC1_FBTXS + offset);
	} else {
		cmd |= (AT86RF215_REG__BBC0_FBTXS + offset);
	}

	getbus(dev);
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd>>8));
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd));
	spi_transfer_bytes(SPIDEV, CSPIN, false, data, NULL, len);
	spi_release(SPIDEV);
}

void at86rf215_rxfb_start(const at86rf2xx_t *dev)
{
	uint16_t cmd = AT86RF215_ACCESS_READ;
	if(dev->rf == _RF24_) {
		cmd |= AT86RF215_REG__BBC1_FBRXS;
	} else {
		cmd |= AT86RF215_REG__BBC0_FBRXS;
	}

    getbus(dev);
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd>>8));
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd));
}

void at86rf215_rxfb_read(const at86rf2xx_t *dev, uint8_t *data, size_t len)
{
    spi_transfer_bytes(SPIDEV, CSPIN, true, NULL, data, len);
}

void at86rf215_rxfb_stop(const at86rf2xx_t *dev)
{
    /* transfer one byte (which we ignore) to release the chip select */
    spi_transfer_byte(SPIDEV, CSPIN, false, 1);
    spi_release(SPIDEV);
}

void at86rf215_assert_awake(at86rf2xx_t *dev)
{
    if (at86rf215_get_state(dev) == AT86RF215_STATE_RF_SLEEP) {
        /* wake up and wait for transition to TRX_OFF */
        gpio_clear(dev->params.sleep_pin);
        xtimer_usleep(AT86RF215_WAKEUP_DELAY);

        do {
            dev->state = at86rf215_get_state(dev);
        } while (dev->state != AT86RF215_STATE_RF_TRXOFF);
    }
}

void at86rf215_hardware_reset(at86rf2xx_t *dev)
{
	DEBUG("[rf215] -- -- hardware_reset\n");

    /* trigger hardware reset */
    gpio_clear(dev->params.reset_pin);
    xtimer_usleep(AT86RF215_RESET_PULSE_WIDTH);
    gpio_set(dev->params.reset_pin);
    xtimer_usleep(AT86RF215_RESET_DELAY);

	/*** test ***/
	dev->state = at86rf215_get_state(dev);
	DEBUG("[rf215] -- -- hardware_reset : state 0x%x\n", dev->state);

    /* at86rf215: automatically end up in state TRXOFF */
    do {
        dev->state = at86rf215_get_state(dev);
	} while (dev->state != AT86RF215_STATE_RF_TRXOFF);
}

void at86rf215_configure_phy(at86rf2xx_t *dev)
{
    /* we must be in TRX_OFF before changing the PHY configuration */
    uint8_t prev_state = at86rf215_set_state(dev, AT86RF215_STATE_RF_TRXOFF);

    /* The TX power register must be updated after changing the channel if
     * moving between bands. */
	// TODO

//    uint8_t phy_cc_cca = at86rf215_reg_read(dev, AT86RF2XX_REG__PHY_CC_CCA);
//    /* Clear previous configuration for channel number */
//    phy_cc_cca &= ~(AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL);
//    /* Update the channel register */
//    phy_cc_cca |= (dev->netdev.chan & AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL);
//    at86rf215_reg_write(dev, AT86RF2XX_REG__PHY_CC_CCA, phy_cc_cca);

	/*** Channel ***/
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__CS, 0x30);
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__CCF0L, 0xF1);
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__CCF0H, 0x86);
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__CNL, dev->netdev.chan);
	/*** channel scheme ***/
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__CNM, 0);

    /* Update the TX power register to achieve the same power (in dBm) */
	// TODO

    /* Return to the state we had before reconfiguring */
    at86rf215_set_state(dev, prev_state);
}
