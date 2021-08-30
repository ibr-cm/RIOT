/** 
* This file allows access to the Variables, which are defined within the Linker Script.
* Every single one of them is located within the pattern.
**/

#ifndef PATTERN_VARS_H
#define PATTERN_VARS_H

/** Adresses are 16 Bit wide **/
extern uint8_t *__data_tracker;
extern uint8_t *__pattern_tracker;

/** Points to start and end of sections **/
extern uint8_t __data_start, __data_end, __noinit_start, __noinit_end, __pattern_start, __pattern_end; 

/** uint8_t Vars used for calculation, signaling and verifying data **/
extern uint8_t __sum_of_bytes, __loop_counter, __data_invalid, __prev_block_value, __var_7;
extern uint8_t __sp_l, __sp_h, __sp_l_mirror, __sp_h_mirror;

/** extern __temp_reg used by code that activates sleep mode **/
extern uint8_t __temp_reg;

/** vars used at end of stack. they are important for the dynamic checksum and need to stay the same after UV ends!**/
extern uint16_t __safe_data_start, __mirror_data_start,  __safe_noinit_end, __mirror_noinit_end, __safe_cs_border, __mirror_cs_border;
extern uint8_t __safe_stack_pointer_l, __mirror_stack_pointer_l, __safe_stack_pointer_h, __mirror_stack_pointer_h; 

#endif
