#include "vtable.h"
#include <avr/eeprom.h>
#include "math.h"

table_entry_t table[VTABLE_SIZE];
uint8_t current_index;
uint8_t table_entries = 0;
uint8_t eeprom_table_available = 0;
uint8_t v_offset = 0;

void init_table(void)
{
	if (eeprom_read_byte(0x0000) == 'y') {
		for (uint8_t i = 0; i < VTABLE_SIZE; i++) {
			void *entry_ptr = EEPROM_ADDR_TABLE + (i * sizeof(table_entry_t));
			eeprom_read_block(&table[i], entry_ptr, 3);
		}
		eeprom_table_available = 1;
		v_offset = eeprom_read_byte(EEPROM_ADDR_VOFF);
		for (uint8_t i = 0; i < VTABLE_SIZE; i++)
			table[i].voltage -= v_offset;
	} else {
		for (uint8_t i = 0; i < VTABLE_SIZE; i++) {
			table[i].voltage = VTABLE_VALUE_IS_EMPTY;
			table[i].info = VTABLE_VALUE_IS_EMPTY;
		}
	}
}

void create_table_entry(uint8_t voltage, uint8_t osccal)
{
	table[(current_index)].voltage = voltage;
	table[(current_index)].osccal = osccal;
	if (table[(current_index)].info != VTABLE_VALUE_IS_MEASURED) {
		table[(current_index)].info = VTABLE_VALUE_IS_MEASURED;
		table_entries++;
	}
	if (table_entries >= SI_PREDICTION_THRESHOLD)
		prediction();
}

void prediction_fill_table(double m_volt, double b_volt, double m_osc, double b_osc)
{
	uint8_t index;
	for (index = 0; index < VTABLE_SIZE; index++) {
		if (table[index].info != VTABLE_VALUE_IS_MEASURED) {
			table[index].voltage = round(m_volt * (double) index + b_volt);
			table[index].osccal = round(m_osc * (double) index + b_osc);
			table[index].info = VTABLE_VALUE_IS_PREDICTED;
		}
	}
}

uint8_t prediction_analyze_table(double *m_volt, double *b_volt, double *m_osc, double *b_osc)
{
	uint8_t n = 0;
	uint8_t index;
	double sum_xy = 0.0,
	       sum_x  = 0.0,
	       sum_x2 = 0.0,
	       sum_y  = 0.0;

	for (index = 0; index < VTABLE_SIZE; index++) {
		if (table[index].info == VTABLE_VALUE_IS_MEASURED) {
			n++;
			sum_x += (double) index;
			sum_x2 += (double) index * (double) index;
			sum_y += (double) table[index].voltage;
			sum_xy += (double) index * (double) table[index].voltage;
		}
	}
	*m_volt = (sum_xy - ((sum_x * sum_y) / n)) / (sum_x2 - ((sum_x * sum_x) / n));
	*b_volt = (sum_y / n) - (sum_x / n) * *m_volt;

	sum_y = 0.0;
	sum_xy = 0.0;
	for (index = 0; index < VTABLE_SIZE; index++) {
		if (table[index].info == VTABLE_VALUE_IS_MEASURED) {
			sum_y += (double) table[index].osccal;
			sum_xy += (double) index * (double) table[index].osccal;
		}
	}
	*m_osc = (sum_xy - ((sum_x * sum_y) / n)) / (sum_x2 - ((sum_x * sum_x) / n));
	*b_osc = (sum_y / n) - (sum_x / n) * *m_osc;
	return n;
}

void prediction(void)
{
	uint8_t n;
	double m_volt, b_volt, m_osc, b_osc;

	/* decrease table_entries - any failure forces a new prediction */
	table_entries--;

	/* runaways? */
	n = prediction_analyze_table(&m_volt, &b_volt, &m_osc, &b_osc);
	if (n >= SI_PREDICTION_THRESHOLD) {
		prediction_fill_table(m_volt, b_volt, m_osc, b_osc);
	} else {
		table_entries = (uint8_t) n;
	}

	/* Write characteristic curve to eeprom */
	for (uint8_t i = 0; i < VTABLE_SIZE; i++) {
		void *entry_ptr = EEPROM_ADDR_TABLE + (i * sizeof(table_entry_t));
		eeprom_write_block(&table[i], entry_ptr, 3);
	}

	/* Write 'y' to the first addr of the eeprom
	   to indicate that the cc already exists
	   and write the initial voltage offset */
	eeprom_write_byte(EEPROM_ADDR_AVAIL, 'y');
	eeprom_write_byte(EEPROM_ADDR_VOFF, 0);
}
