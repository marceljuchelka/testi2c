/*
 * PCF8574.c
 *
 *  Created on: 2010-09-07
 *       Autor: Miros³aw Kardaœ
 */
#include <avr/io.h>

#include "mk_i2c.h"




/*
 * 	8-bit EXPANDER
 * 	PDF: https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf
 *
	PCF8574 - Base addr 0x40

	A6|A5|A4|A3|A2|A1|A0|RW|ADR
	------------------------
	 0| 1| 0| 0|  |  |  |  |0x40 <-- base addr

	A6|A5|A4|A3|A2|A1|A0|RW|ADR
	------------------------
	 0| 1| 0| 0| 0| 0| 0|  |0x40
	 0| 1| 0| 0| 0| 0| 1|  |0x42
	 0| 1| 0| 0| 0| 1| 0|  |0x44
	 0| 1| 0| 0| 0| 1| 1|  |0x46
	 0| 1| 0| 0| 1| 0| 0|  |0x48
	 0| 1| 0| 0| 1| 0| 1|  |0x4A
	 0| 1| 0| 0| 1| 1| 0|  |0x4C
	 0| 1| 0| 0| 1| 1| 1|  |0x4E



	PCF8574A - Base addr 0x70

	A6|A5|A4|A3|A2|A1|A0|RW|ADR
	------------------------
	 0| 1| 1| 1|  |  |  |  |0x70 <-- base addr

	A6|A5|A4|A3|A2|A1|A0|RW|ADR
	------------------------
	 0| 1| 1| 1| 0| 0| 0|  |0x70
	 0| 1| 1| 1| 0| 0| 1|  |0x72
	 0| 1| 1| 1| 0| 1| 0|  |0x74
	 0| 1| 1| 1| 0| 1| 1|  |0x76
	 0| 1| 1| 1| 1| 0| 0|  |0x78
	 0| 1| 1| 1| 1| 0| 1|  |0x7A
	 0| 1| 1| 1| 1| 1| 0|  |0x7C
	 0| 1| 1| 1| 1| 1| 1|  |0x7E
*/


#if I2C_MODE == 0

void pcf8574_write( uint8_t SLA, uint8_t byte ) {

	i2c_start();
	i2c_write(SLA);
	i2c_write( byte );
	i2c_stop();
	PORTC ^= (1<<PC6);
}

uint8_t pcf8574_read( uint8_t SLA ) {

	uint8_t res = 0;
	i2c_start();
	i2c_write(SLA+1);
	res = i2c_read( NACK );
	i2c_stop();

	return res;
}

#endif
