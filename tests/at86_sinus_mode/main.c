
#include "at86rf2xx.h"
#include "at86rf2xx_registers.h"
#include "at86rf2xx_internal.h"
#include "at86rf2xx_netdev.h"
#include "at86rf2xx_params.h"

void at86rf2xx_cw_mode(void);
void buttonPressHandler(void);

static at86rf2xx_t myRadio;
static uint8_t testModeRunning = 0;
static uint8_t cancelLoop = 1;

int main(void)
{
    /* Params are stored in at86rf2xx_params.h */
    at86rf2xx_setup(&myRadio, &at86rf2xx_params[0]);

    /*activate test mode on button press*/
}

void buttonPressHandler(void)
{
    if(testModeRunning == 0)
    {
        testModeRunning = 1;
        at86rf2xx_cw_mode();
    } else {
        cancelLoop = 1;
    }
}

void at86rf2xx_cw_mode(void)
{
    /* Look here for more details: http://ww1.microchip.com/downloads/en/devicedoc/atmel-8351-mcu_wireless-at86rf233_datasheet.pdf */

    /*reset Radio*/
    at86rf2xx_hardware_reset(&myRadio);      

    /*set Regs*/
    at86rf2xx_reg_write(&myRadio, 0x0E, 0x01);  
    at86rf2xx_reg_write(&myRadio, 0x04, 0x00);
    at86rf2xx_reg_write(&myRadio, 0x02, 0x03);
    at86rf2xx_reg_write(&myRadio, 0x03, 0x01);
    at86rf2xx_reg_write(&myRadio, 0x08, 0x33);
    at86rf2xx_reg_write(&myRadio, 0x05, 0x00);

    /*check Register Value (verify TRX State OFF)*/
    uint8_t regVal = at86rf2xx_reg_read(&myRadio, 0x01);
    if(regVal != 0x08)
    {
        puts("Something went wrong.\n");
        return;
    }

    /*set Regs*/
    at86rf2xx_reg_write(&myRadio, 0x36, 0x0F);
    /*only for CW Mode*/
    at86rf2xx_reg_write(&myRadio, 0x0C, 0x03);
    at86rf2xx_reg_write(&myRadio, 0x0A, 0x37);
    
    /*Frame Buffer Write Access*/
    uint8_t frameSize = 127;
    const uint8_t frame[127] = {  0 };
    at86rf2xx_sram_write(&myRadio, 1, frame , 127); /*write PSDU Data*/
    at86rf2xx_sram_write(&myRadio, 0, &frameSize, 1);   /*write length field*/

    
    /*set Regs*/
    at86rf2xx_reg_write(&myRadio, 0x1C, 0x54);
    at86rf2xx_reg_write(&myRadio, 0x1C, 0x46);
    at86rf2xx_reg_write(&myRadio, 0x02, 0x09);

    /*check Register Value (Wait for IRQ_0 (PLL_LOCK))*/
    regVal = at86rf2xx_reg_read(&myRadio, 0x0F);
    if(regVal != 0x01)
    {
        puts("Something went wrong.\n");
        return;
    }

    at86rf2xx_reg_write(&myRadio, 0x02, 0x02);

    while(!cancelLoop)
    {

    }
    cancelLoop = 0;
    /*disable continuos transmission mode*/
    at86rf2xx_reg_write(&myRadio, 0x1C, 0x00);
    /*reset Radio*/
    at86rf2xx_hardware_reset(&myRadio); 
    testModeRunning = 0;
}