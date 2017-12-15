#ifndef IV_FRAME_H
#define IV_FRAME_H

#include <stdint.h>

#define SI_REG_LOCK       0x00
#define SI_REG_REQUEST    0x00
#define SI_REG_REPLY      0x01

enum {
	SI_BUSY = 0,
	SI_READY = 1,
	SI_BOOTING = 2
};

typedef struct iv_req {
	uint8_t checksum;
	uint8_t temperature;
	uint8_t osccal;
	uint8_t rst_flags;
	uint8_t alt_byte;
	uint8_t rst_disable;
} iv_req_t;

typedef struct iv_res {
	uint8_t osccal;
	uint8_t voltage;
	uint8_t dt_l;
	uint8_t dt_h;
	uint8_t debug;
} iv_res_t;

#endif /* IV_FRAME_H */
