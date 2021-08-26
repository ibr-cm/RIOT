#include "sram_undervolting.h"
#include <avr/sleep.h>

//#define ACTIVATE_VERSION_2 (0)

const pin_t uv_irq_pins[] = {
    IIF_SIG,
};


__attribute__((always_inline)) static inline void write_undervolting_check(void) {
    __data_tracker = (uint8_t*)(&__data_start);
    __pattern_tracker = (uint8_t*)(&__pattern_start);
        /**save sum of bytes here**/
    __sum_of_bytes = 0x00;

    //set start value of the chain
    __prev_block_value = SPH;

    SIG_BLOCK_ONE_HIGH;

    /** first block**/
    while(__data_tracker < (const uint8_t*)(&__noinit_end)) {
        for(__loop_counter = 0; __loop_counter < 16; __loop_counter++) {
            __sum_of_bytes += *(__data_tracker);
            __data_tracker++;
        }

        *__pattern_tracker = (__sum_of_bytes ^ __prev_block_value);
        __prev_block_value = __sum_of_bytes;
        
        __pattern_tracker++;
        __sum_of_bytes = 0x00;
    }

    SIG_BLOCK_ONE_LOW;

    /** save border between the two blocks. First address of pattern of second block. **/
    __safe_cs_border = (uint16_t)__pattern_tracker;
    __mirror_cs_border = (uint16_t)__pattern_tracker;

    __sp_l_mirror = SPL;
    __sp_h_mirror = SPH;

    /**find start of stack, align with 16 B blocks for checksum**/
    __temp_reg =  SPL % 16;
    __temp_reg = SPL - __temp_reg;
    __data_tracker = (uint8_t*)((SPH << 8) | __temp_reg);

    //set start value of the chain
    __prev_block_value = SPL;

    SIG_BLOCK_TWO_HIGH;

    /**second block**/
    while(__data_tracker < (const uint8_t*)(&__pattern_start)) {
        for(__loop_counter = 0; __loop_counter < 16; __loop_counter++) {
            __sum_of_bytes += *(__data_tracker);
            __data_tracker++;
        }

        *__pattern_tracker = (__sum_of_bytes ^ __prev_block_value);
        __prev_block_value = __sum_of_bytes;
        
        __pattern_tracker++;
        __sum_of_bytes = 0x00;
    }

    SIG_BLOCK_TWO_LOW;

    //sleep_bod_disable();
    do { \
        __asm__ __volatile__("in %[__temp_reg], %[mcucr]" "\n\t" \
                            "ori %[__temp_reg], %[bods_bodse]" "\n\t" \
                            "out %[mcucr], %[__temp_reg]" "\n\t" \
                            "andi %[__temp_reg], %[not_bodse]" "\n\t" \
                            "out %[mcucr], %[__temp_reg]" \
                            : [__temp_reg] "=&d" (__temp_reg) \
                            : [mcucr] "I" _SFR_IO_ADDR(BOD_CONTROL_REG), \
                            [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                            [not_bodse] "i" (~_BV(BODSE))); \
    } while (0);

	MCUCR |= (1 << JTD);
	MCUCR |= (1 << JTD);
	//Clear Sleep reg
	SMCR &= 0x00;
	//Set sleepmode to POWER_DOWN
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable(); //set Sleep Enable Bit

    POWER_MIKROC_OFF;

    sleep_cpu(); //go to sleep.

}

__attribute__((always_inline)) static inline void verify_undervolting_check(void) {
    __data_tracker = (uint8_t *)(&__data_start);
    __pattern_tracker = (uint8_t *)(&__pattern_start);
    /**save sum of bytes here**/
    __sum_of_bytes = 0x00;
    /**this signals the result**/
    __data_invalid = 0x00;
    /**use this for the second version of the checksum**/
    __prev_block_value = __safe_stack_pointer_h;

    SIG_BLOCK_ONE_HIGH;

    /**first block**/
    while(__data_tracker < (const uint8_t *)(&__noinit_end)) { 
        for(__loop_counter = 0; __loop_counter < 16; __loop_counter++) {
            __sum_of_bytes += *(__data_tracker);
            __data_tracker++;
        }

        if(*__pattern_tracker != (__sum_of_bytes ^__prev_block_value)) {
            /**Signals, that checksum is not correct**/
            __data_invalid = 0xFF;
            break;   
        } else {
            __prev_block_value = __sum_of_bytes;
            __pattern_tracker++;
            __sum_of_bytes = 0x00;
        }
    }

    SIG_BLOCK_ONE_LOW;

    /**find start of stack, align with 16 B blocks for checksum**/
    __temp_reg       = __safe_stack_pointer_l % 16;
    __temp_reg       = __safe_stack_pointer_l - __temp_reg;
    __data_tracker   = (uint8_t*)( (__safe_stack_pointer_h << 8) | __temp_reg);

    //set start value of the second chain
    __prev_block_value = __safe_stack_pointer_l;

    SIG_BLOCK_TWO_HIGH;

    /**second block**/
    while(__data_tracker < (const uint8_t *)(&__pattern_start)) { 
        for(__loop_counter = 0; __loop_counter < 16; __loop_counter++) {
            __sum_of_bytes += *(__data_tracker);
            __data_tracker++;
        }

        if(*__pattern_tracker != (__sum_of_bytes ^__prev_block_value)) {
            /**Signals, that checksum is not correct**/
            __data_invalid = 0xFF;
            break;   
        } else {
            __prev_block_value = __sum_of_bytes;
            __pattern_tracker++;
            __sum_of_bytes = 0x00;
        }
    }

    SIG_BLOCK_TWO_LOW;

}


void undervolting_sleep(void) {/**push all regs on stack**/
    __asm__ volatile (
        "push __tmp_reg__                    \n\t"
        "in   __tmp_reg__, __SREG__          \n\t"
        "cli                                 \n\t"
        "push __tmp_reg__                    \n\t"
        "push r1                             \n\t"
        "clr  r1                             \n\t"
        "push r2                             \n\t"
        "push r3                             \n\t"
        "push r4                             \n\t"
        "push r5                             \n\t"
        "push r6                             \n\t"
        "push r7                             \n\t"
        "push r8                             \n\t"
        "push r9                             \n\t"
        "push r10                            \n\t"
        "push r11                            \n\t"
        "push r12                            \n\t"
        "push r13                            \n\t"
        "push r14                            \n\t"
        "push r15                            \n\t"
        "push r16                            \n\t"
        "push r17                            \n\t"
        "push r18                            \n\t"
        "push r19                            \n\t"
        "push r20                            \n\t"
        "push r21                            \n\t"
        "push r22                            \n\t"
        "push r23                            \n\t"
        "push r24                            \n\t"
        "push r25                            \n\t"
        "push r26                            \n\t"
        "push r27                            \n\t"
        "push r28                            \n\t"
        "push r29                            \n\t"
        "push r30                            \n\t"
        "push r31                            \n\t");
        
    /**save stack-pointer in pattern_segment**/
    __sp_l = SPL;
    __sp_h = SPH;
    __sp_l_mirror = SPL;
    __sp_h_mirror = SPH;

    /**save important data at end of stack**/
    __safe_data_start = (uint16_t)&__data_start;
    __mirror_data_start = (uint16_t)&__data_start;

    __safe_noinit_end = (uint16_t)&__noinit_end;
    __mirror_noinit_end = (uint16_t)&__noinit_end;

    __safe_stack_pointer_l = SPL;
    __safe_stack_pointer_h = SPH;

    __mirror_stack_pointer_l = SPL;
    __mirror_stack_pointer_h = SPH;

    /**write pattern and sleep**/
    write_undervolting_check();
    return;
}

void undervolting_restore(void) {
    __data_invalid = 0x00;
    if(__safe_stack_pointer_l != __mirror_stack_pointer_l || __safe_stack_pointer_h != __mirror_stack_pointer_h) {
        __data_invalid = 0xFF;
        return;
    }
    /**** check pattern
     * if pattern correct and state restore needed: load stackpointer from pattern section and pull all regs from stack. 
     * use return to jump back to old context **/
    verify_undervolting_check();
    /**TODO: implement case that no state restore is required**/
    if(__data_invalid == 0x00) {
        /**restore saved stackpointer**/
        SPL = __safe_stack_pointer_l;
        SPH = __safe_stack_pointer_h;
        __asm__ volatile (
            "pop  r31                            \n\t"
            "pop  r30                            \n\t"
            "pop  r29                            \n\t"
            "pop  r28                            \n\t"
            "pop  r27                            \n\t"
            "pop  r26                            \n\t"
            "pop  r25                            \n\t"
            "pop  r24                            \n\t"
            "pop  r23                            \n\t"
            "pop  r22                            \n\t"
            "pop  r21                            \n\t"
            "pop  r20                            \n\t"
            "pop  r19                            \n\t"
            "pop  r18                            \n\t"
            "pop  r17                            \n\t"
            "pop  r16                            \n\t"
            "pop  r15                            \n\t"
            "pop  r14                            \n\t"
            "pop  r13                            \n\t"
            "pop  r12                            \n\t"
            "pop  r11                            \n\t"
            "pop  r10                            \n\t"
            "pop  r9                             \n\t"
            "pop  r8                             \n\t"
            "pop  r7                             \n\t"
            "pop  r6                             \n\t"
            "pop  r5                             \n\t"
            "pop  r4                             \n\t"
            "pop  r3                             \n\t"
            "pop  r2                             \n\t"
            "pop  r1                             \n\t"
            "pop    __tmp_reg__                  \n\t"
            "out  __SREG__, __tmp_reg__          \n\t"
            "pop  __tmp_reg__                    \n\t");

        gpio_restore();

        check_uv_irqs();
        
        /** Now we are back in the old context!**/
        return;
    }
    return;
}