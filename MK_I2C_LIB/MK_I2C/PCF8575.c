/*
 * PCF8574.c
 *
 *  Created on: 2010-09-07
 *       Autor: Miros³aw Kardaœ
 */
#include <avr/io.h>

#include "mk_i2c.h"

/*
 * 16-bit EXPANDER
 * PDF: https://www.nxp.com/docs/en/data-sheet/PCF8575.pdf?
 *
	PCF8575 - Base addr 0x40

	A6|A5|A4|A3|A2|A1|A0|RW|ADR
	------------------------
	 0| 1| 0| 0| x| x| x|  |0x40 <-- base addr



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

*/


#if I2C_MODE == 0

void pcf8575_write( uint8_t SLA, uint16_t data ) {

	i2c_start();
	i2c_write(SLA);
	i2c_write( data >> 8 );
	i2c_write( data >> 0 );
	i2c_stop();
}

uint16_t pcf8575_read( uint8_t SLA ) {

	uint16_t res = 0;
	uint8_t * buf = (uint8_t*)&res;
	uint8_t len = 2;

	i2c_start();
	i2c_write(SLA+1);
	while (len--) *buf++ = i2c_read( len ? ACK : NACK );
	i2c_stop();

	return res;
}

#endif
