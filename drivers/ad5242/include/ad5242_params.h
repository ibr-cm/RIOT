#ifndef AD5242_PARAMS_H
#define AD5242_PARAMS_H

#include "board.h"

#ifndef AD5242_PARAMS
#ifdef AD5242_PARAMS_BOARD
#define AD5242_PARAMS AD5242_PARAMS_BOARD
#else
#error "No AD5242_PARAMS defined and no AD5242_PARAMS_BOARD defined in board config."
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

static const ad5242_params_t ad5242_params = AD5242_PARAMS;

#ifdef __cplusplus
}
#endif

#endif /* AD5242_PARAMS_H */
