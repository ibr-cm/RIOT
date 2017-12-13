#include "idealvolting.h"
#include "ad5242.h"
#include "ad5242_params.h"
#include "xtimer.h"
#include <stdio.h>

static ad5242_t ad5242_dev;

void idealvolting_init(void)
{
	ad5242_init(&ad5242_dev, &ad5242_params);
	ad5242_set_reg(&ad5242_dev, 0);

	puts("idealvolting_init");

	xtimer_ticks32_t last_wakeup = xtimer_now();
	for (int i = 0; i < 10; i++) {
		xtimer_periodic_wakeup(&last_wakeup, 1000000);
		printf("idealvolting %lld\n", last_wakeup);
	}
}
