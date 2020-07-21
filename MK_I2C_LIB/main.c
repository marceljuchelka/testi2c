/*
 * main.c
 *
 *  Created on: 2019-04-09
 *       Autor: Miros³aw Kardaœ
 *       MK_I2C Library
 *       ver: 1.00
 *
 *		testy:
 *		ATMega32 - Mode 0 (Master) + Soft I2C
 *		ATmega32 - MODE 1 (Slave TWI)
 *		ATtiny45 - MODE 2 (Slave USI)
 *
 *      MSTER/SLAVE (TWI/USI) dla mikrokontrolerów
 *      ATmega i ATtiny
 *      (ATtiny 2313 / 25/45/85 / 24(A)/44(A)/84 / 26 / 261/461/861)
 */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "MK_I2C/mk_i2c.h"
#include "MK_LCD/mk_lcd44780.h"

/*
 * 	MODE 0 - ATmega
 * 	MODE 1 - ATmega
 * 	MODE 2 - ATtiny 2313 / 25/45/85 / 24(A)/44(A)/84 / 26 / 261/461/861
 *
 *
 */

#if I2C_MODE < 2	// SLAVE TWI - ATmega
#include "MK_LCD/mk_lcd44780.h"
#endif


#if I2C_MODE == 1	// SLAVE TWI - ATmega

volatile uint8_t big_licz1 = 100;
volatile uint8_t big_licz2 = 100;

// w³asna funckcja CALLBACK - bajty odbierane z Mstera.
// UWAGA! callback wywo³ywany z przerwania - wiêc nale¿y
//        u¿ywaæ w tej funkcji zmiennych ze specyfikatorem
//        volatile (dok³adnie tak jak w przerwaniach)
// UWAGA! funkcja MUSI zajmowaæ jak najmniej czasu, najlepiej
//        w ogóle nie u¿ywaæ w niej wyœwietlania na LCD !!!
//        W tym wypadku u¿yto wyœwietlania aby uproœciæ przyk³ad.
void slave_rx( uint8_t data ) {
	lcd_locate( 1,0 );
	lcd_str( "rx: " );
	lcd_int( data );
	lcd_str( "  " );
	if( !(data%2) ) PORTC ^= (1<<PC6);
}

// w³asna funckcja CALLBACK - ¿¹danie Mastera o przes³anie
// pierwszego/pojedynczego bajtu.
// UWAGA! callback wywo³ywany z przerwania - wiêc nale¿y
//        u¿ywaæ w tej funkcji zmiennych ze specyfikatorem
//        volatile (dok³adnie tak jak w przerwaniach)
// UWAGA! funkcja MUSI zajmowaæ jak najmniej czasu, najlepiej
//        w ogóle nie u¿ywaæ w niej wyœwietlania na LCD !!!
//        W tym wypadku u¿yto wyœwietlania aby uproœciæ przyk³ad.
void slave_req( void ) {
	static uint8_t cnt1;
	lcd_locate( 2,0 );
	lcd_str( "REQ 0 cnt: " );
	lcd_int( cnt1++ );
	lcd_str( "  " );
	i2c_slave_send_byte( big_licz1++ );
}

// w³asna funckcja CALLBACK - ¿¹danie Mastera o przes³anie
// kolejnych bajtów jeœli Master odczytuje je sekwencyjnie.
// UWAGA! callback wywo³ywany z przerwania - wiêc nale¿y
//        u¿ywaæ w tej funkcji zmiennych ze specyfikatorem
//        volatile (dok³adnie tak jak w przerwaniach)
// UWAGA! funkcja MUSI zajmowaæ jak najmniej czasu, najlepiej
//        w ogóle nie u¿ywaæ w niej wyœwietlania na LCD !!!
//        W tym wypadku u¿yto wyœwietlania aby uproœciæ przyk³ad.
void slave_req_next( void ) {
	static uint8_t cnt2;
	lcd_locate( 3,0 );
	lcd_str( "REQ 1 cnt: " );
	lcd_int( cnt2++ );
	lcd_str( "  " );
	i2c_slave_send_byte( big_licz2-- );
}
#endif



#if I2C_MODE == 2	// SLAVE USI - ATtiny

volatile uint8_t big_licz1 = 100;
volatile uint8_t big_licz2 = 100;


// w³asna funckcja CALLBACK - bajty odbierane z Mstera.
// UWAGA! callback wywo³ywany z przerwania - wiêc nale¿y
//        u¿ywaæ w tej funkcji zmiennych ze specyfikatorem
//        volatile (dok³adnie tak jak w przerwaniach)
// UWAGA! funkcja MUSI zajmowaæ jak najmniej czasu, najlepiej
//        w ogóle nie u¿ywaæ w niej wyœwietlania na LCD !!!
//        W tym wypadku u¿yto wyœwietlania aby uproœciæ przyk³ad.
void slave_rx( uint8_t data ) {

	if( !(data%2) ) PORTB ^= (1<<PB4);
}

// w³asna funckcja CALLBACK - ¿¹danie Mastera o przes³anie
// pierwszego/pojedynczego bajtu.
// UWAGA! callback wywo³ywany z przerwania - wiêc nale¿y
//        u¿ywaæ w tej funkcji zmiennych ze specyfikatorem
//        volatile (dok³adnie tak jak w przerwaniach)
// UWAGA! funkcja MUSI zajmowaæ jak najmniej czasu, najlepiej
//        w ogóle nie u¿ywaæ w niej wyœwietlania na LCD !!!
//        W tym wypadku u¿yto wyœwietlania aby uproœciæ przyk³ad.
void slave_req( void ) {

	i2c_slave_send_byte( big_licz1++ );
}

// w³asna funckcja CALLBACK - ¿¹danie Mastera o przes³anie
// kolejnych bajtów jeœli Master odczytuje je sekwencyjnie.
// UWAGA! callback wywo³ywany z przerwania - wiêc nale¿y
//        u¿ywaæ w tej funkcji zmiennych ze specyfikatorem
//        volatile (dok³adnie tak jak w przerwaniach)
// UWAGA! funkcja MUSI zajmowaæ jak najmniej czasu, najlepiej
//        w ogóle nie u¿ywaæ w niej wyœwietlania na LCD !!!
//        W tym wypadku u¿yto wyœwietlania aby uproœciæ przyk³ad.
void slave_req_next( void ) {

	i2c_slave_send_byte( big_licz2-- );
}

#endif


int main(void) {

#if I2C_MODE < 2	// MASTER ATmega
	lcd_init();
	lcd_LED( 1 );
#endif


#if I2C_MODE == 0	// MASTER ATmega
	int i2c_bitrate;
	i2c_bitrate = i2c_init( 800 );

	lcd_int( i2c_bitrate );
	lcd_str_P( PSTR(" kHz"));
	_delay_ms(1000);
	lcd_cls();
	lcd_str( "I2C MASTER TWI");

	DDRC |= (1<<PC6);
	PORTC |= (1<<PC6);

	uint8_t licznik = 0;
#endif

#if I2C_MODE == 1	// SLAVE TWI - ATmega
	lcd_cls();
	lcd_str( "I2C SLAVE TWI");

	i2c_slave_reg_callbacks( slave_rx, slave_req, slave_req_next );
	i2c_slave_init( I2C_SLAVE_ADDRESS );

	DDRC |= (1<<PC6);
	PORTC |= (1<<PC6);
#endif


#if I2C_MODE == 2	// SLAVE USI - ATtiny
	i2c_slave_reg_callbacks( slave_rx, slave_req, slave_req_next );
	i2c_slave_init( I2C_SLAVE_ADDRESS );

	DDRB |= (1<<PB4);
	PORTB |= (1<<PB4);
#endif


	sei();

	while(1) {

		#if I2C_MODE == 0	// MASTER ATmega
			lcd_locate( 1,0 );
			lcd_str( "tx:" );
			lcd_int( licznik );
			lcd_str( "  " );
			i2c_send_byte( I2C_SLAVE_ADDRESS, licznik++ );

			uint16_t w = i2c_read_word( I2C_SLAVE_ADDRESS );
			lcd_locate( 1,8 );
			lcd_int( w>>8 );
			lcd_str( "," );
			lcd_int( w & 0x00FF );
			lcd_str( "  " );

			PORTC ^= (1<<PC6);

			_delay_ms(1000);
		#endif

	}
}

