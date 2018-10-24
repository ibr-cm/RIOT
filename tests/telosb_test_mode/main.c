
#include "cc2420_internal.h"
#include "cc2420_netdev.h"
#include "cc2420_registers.h"
#include "cc2420_params.h"

#include "xtimer.h"
#include "thread.h"

cc2420_t myRadio;

char sinus_thread_stack[512+256];

void *testMode(void *arg)
{
    if(arg != NULL)
    {
        puts("I dont care about args!");
    }
    puts("First STROBE");
    myRadio.netdev.netdev.driver->init((netdev_t *)(&myRadio));
    
    puts("Setting Regs");
    cc2420_reg_write(&myRadio, CC2420_REG_MDMCTRL1, 0x050C); /* this could be wrong. but lets try it*/
    //cc2420_reg_write(&myRadio, CC2420_REG_DACTST, 0x1800);
    puts("Second STROBE");
    cc2420_strobe(&myRadio, CC2420_STROBE_TXON);
    puts("entering LOOP");
    while(1)
    {

    }


}

int main (void)
{
    /* Set up cc2420 params. */
    cc2420_setup(&myRadio, &cc2420_params[0]);
    puts("Radio set, wait");
    xtimer_sleep(1);
    puts("Wait done, lets go");
    thread_create(sinus_thread_stack, sizeof(sinus_thread_stack), THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST, testMode, NULL, "sinus_thread");
}