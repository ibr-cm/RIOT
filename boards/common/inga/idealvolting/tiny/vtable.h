#ifndef VTABLE_H
#define VTABLE_H

#include <stdint.h>

#define VTABLE_SIZE 51
#define SI_PREDICTION_THRESHOLD 7
#define EEPROM_ADDR_AVAIL                   ((void *) 0x00)
#define EEPROM_ADDR_VOFF                    ((void *) 0x01)
#define EEPROM_ADDR_TABLE                   ((void *) 0x02)

#define get_entry() table[current_index]

enum vtable_state {
	VTABLE_VALUE_IS_EMPTY = 0xFF,
	VTABLE_VALUE_IS_MEASURED = 0x01,
	VTABLE_VALUE_IS_PREDICTED = 0x02
};

typedef struct {
	uint8_t voltage;
	uint8_t osccal;
	uint8_t info;
} table_entry_t;

extern table_entry_t table[VTABLE_SIZE];
extern uint8_t current_index, table_entries, eeprom_table_available;

void init_table(void);
table_entry_t *get_table_entry(void);
void create_table_entry(uint8_t voltage, uint8_t osccal);
void prediction(void);

#endif /* VTABLE_H */
