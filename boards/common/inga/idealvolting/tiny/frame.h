/*
 * frame.h
 *
 *  Created on: 11.07.2014
 *      Author: ulf
 */

#ifndef FRAME_H_
#define FRAME_H_

/* RX Frame:
 * MCU => SI */
#define REQUEST_FRAME_SIZE 6
enum rxframe {
	mcu_checksum,
	mcu_temperature,
	mcu_oscall,
	mcu_rstFlags,
	mcu_altByte,
	mcu_rst_disable
};

/* TX Frame:
 * SI => MCU */
#define REPLY_FRAME_SIZE 6
enum txframe {
	si_lock,
	si_oscall,
	si_voltage,
	si_dt_l,
	si_dt_h,
	si_debug
};



#endif /* FRAME_H_ */
