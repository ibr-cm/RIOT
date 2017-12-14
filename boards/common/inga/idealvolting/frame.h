#ifndef IV_FRAME_H
#define IV_FRAME_H

#include <stdint.h>

typedef struct iv_req {
	uint8_t checksum;
	uint8_t temperature;
	uint8_t osccal;
	uint8_t rst_flags;
	uint8_t alt_byte;
	uint8_t rst_disable;
} iv_req_t;

typedef struct iv_res {
	uint8_t lock;
	uint8_t osccal;
	uint8_t voltage;
	uint16_t dt;
	uint8_t debug;
} iv_res_t;

#endif /* IV_FRAME_H */
