
#include "at86rf2xx.h"
#include "at86rf2xx_registers.h"
#include "at86rf2xx_internal.h"
#include "at86rf2xx_netdev.h"
#include "at86rf2xx_params.h"

#include "periph/gpio.h"
#include "board.h"
#include "thread.h"
#include "xtimer.h"

void *at86rf2xx_cw_mode(void *arg);
void buttonPressHandler(void *arg);

static at86rf2xx_t myRadio;
static uint8_t testModeRunning = 0;
static uint8_t cancelLoop = 1;

char sinus_thread_stack[512+256];

int main(void)
{
    /* Params are stored in at86rf2xx_params.h */
    at86rf2xx_setup(&myRadio, &at86rf2xx_params[0]);
    xtimer_sleep(10);
    puts("wait done");
    //gpio_init_int(BTN0_PIN, BTN0_MODE, GPIO_RISING, buttonPressHandler, NULL);
    thread_create(sinus_thread_stack, sizeof(sinus_thread_stack), THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST, at86rf2xx_cw_mode, NULL, "sinus_thread");
    /*activate test mode on button press*/
}

void buttonPressHandler(void *arg)
{
    if(arg != NULL)
    {
        puts("I dont care about args!");
    }

    if(testModeRunning == 0)
    {
        puts("activate Sinus Mode!");
        testModeRunning = 1;
        thread_create(sinus_thread_stack, sizeof(sinus_thread_stack), THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST, at86rf2xx_cw_mode, NULL, "sinus_thread");
        //at86rf2xx_cw_mode(); /*This needs to run in a new Thread!*/
    } else {
        puts("deactivate Sinus Mode!");
        cancelLoop = 1;
    }
}

void *at86rf2xx_cw_mode(void *arg)
{
    /* Look here for more details: http://ww1.microchip.com/downloads/en/devicedoc/atmel-8351-mcu_wireless-at86rf233_datasheet.pdf */
    if(arg != NULL)
    {
        puts("I dont care about args!");
    }
    puts("Resetting Hardware");
    /*reset Radio, sleep is included into reset function!*/
    at86rf2xx_hardware_reset(&myRadio);      

    puts("Setting Regs");
    /*set Regs*/
    at86rf2xx_reg_write(&myRadio, 0x0E, 0x01);  
    at86rf2xx_reg_write(&myRadio, 0x04, 0x00);
    at86rf2xx_reg_write(&myRadio, 0x02, 0x03);
    at86rf2xx_reg_write(&myRadio, 0x03, 0x01);
    at86rf2xx_reg_write(&myRadio, 0x08, 0x33); /*Set Channel 19*/
    at86rf2xx_reg_write(&myRadio, 0x05, 0x00);

    /*check Register Value (verify TRX State OFF)*/
    puts("Check State");
    uint8_t regVal = at86rf2xx_reg_read(&myRadio, 0x01);
    if(regVal != 0x08)
    {
        puts("Something went wrong. #1\n");
        printf("RegVal was: %d .\n", regVal);
        testModeRunning = 0;
        return NULL;
    }

    /*set Regs*/
    puts("Setting Regs");
    at86rf2xx_reg_write(&myRadio, 0x36, 0x0F);
    /*only for CW Mode*/
    at86rf2xx_reg_write(&myRadio, 0x0C, 0x03);
    at86rf2xx_reg_write(&myRadio, 0x0A, 0x37);
    
    /*Frame Buffer Write Access*/
    puts("Writing Frame Buffer");
    uint8_t frameSize = 127;
    const uint8_t frame[127] = {  0 };
    at86rf2xx_sram_write(&myRadio, 1, frame , 127); /*write PSDU Data*/
    at86rf2xx_sram_write(&myRadio, 0, &frameSize, 1);   /*write length field*/

    
    /*set Regs*/
    puts("Setting Regs");
    at86rf2xx_reg_write(&myRadio, 0x1C, 0x54);
    at86rf2xx_reg_write(&myRadio, 0x1C, 0x46);
    at86rf2xx_reg_write(&myRadio, 0x02, 0x09); /*Enable PLL_ON state*/

    xtimer_usleep(1000);
    puts("Checking IRQ");
    /*
    for(uint8_t i = 20; i > 0; i--)
    {
        //check Register Value (Wait for IRQ_0 (PLL_LOCK))
        regVal = at86rf2xx_reg_read(&myRadio, 0x0F);
        if(regVal != 0x01)
        {
            puts("Something went wrong. #2");
            printf("RegVal was: %d . Tries left: %d\n", regVal, i);
            if(i == 1)
            {
                puts("End of Mode");
                testModeRunning = 0;
                return NULL;
            }
        }
        xtimer_sleep(1);
    }
    */
    at86rf2xx_reg_write(&myRadio, 0x02, 0x02);
    puts("Entering Loopz");
    while(1)
    {

    }
    puts("Loop was canceled!");
    cancelLoop = 0;
    /*disable continuos transmission mode*/
    at86rf2xx_reg_write(&myRadio, 0x1C, 0x00);
    /*reset Radio*/
    at86rf2xx_hardware_reset(&myRadio); 
    testModeRunning = 0;
    return NULL;
}