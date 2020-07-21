/*
 * mk_i2c.c
 *
 *  Created on: 2019-04-09
 *       Autor: Miros³aw Kardaœ
 *       MK_I2C Library
 *       ver: 1.01
 *
 *      MSTER/SLAVE (TWI/USI) dla mikrokontrolerów
 *      ATmega i ATtiny
 *      (ATtiny 2313 / 25/45/85 / 24(A)/44(A)/84 / 26 / 261/461/861)
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <util/delay.h>

#include "mk_i2c.h"

#if I2C_MODE == 0

#define Q_DEL 	_delay_loop_2(3)
#define H_DEL 	_delay_loop_2(5)

uint8_t i2c_error_flag;
uint16_t i2c_tmo = I2C_TIMEOUT;

static void (*i2c_damage_callback)(void);

// funkcja do rejestracji funkcji zwrotnej
void register_i2c_damage_event_callback(void (*callback)(void)) {
	i2c_damage_callback = callback;
}

int i2c_init( uint16_t bitrateKHz ) {

#if USE_SOFT_I2C == 0
	return i2cSetBitrate( bitrateKHz );

#else

	SDA_PORT &= ~(1<<iSDA);
	SCL_PORT &= ~(1<<iSCL);

	SDA_HI;
	SCL_HI;

	return -1;
#endif

}

int i2cSetBitrate( uint16_t bitrateKHz ) {

#if USE_SOFT_I2C == 0


	uint8_t twbr_val;

	TWSR &= ~( (1<<TWPS1)|(1<<TWPS0) );

	twbr_val = ((F_CPU/1000ul)/bitrateKHz);
	if( twbr_val >= 16 ) twbr_val = (twbr_val-16+1) / 2;
	else twbr_val = 2;

	TWBR = twbr_val;

	return ( F_CPU / ( 16ul + 2ul * twbr_val ) ) / 1000ul;
#else

	return -1;
#endif

}

void i2c_start( void ) {

#if USE_SOFT_I2C == 0

	if( i2c_error_flag && i2c_damage_callback ) i2c_damage_callback();
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTA);
	i2c_tmo = I2C_TIMEOUT;
	while ( !(TWCR&(1<<TWINT)) ) {
		if( --i2c_tmo == 0 ) {
			i2c_error_flag = 1;
			break;
		}
	}
#else
	SCL_HI;
	H_DEL;

	SDA_LO;
	H_DEL;
#endif
}

void i2c_stop( void ) {

#if USE_SOFT_I2C == 0

	if( i2c_error_flag && i2c_damage_callback ) i2c_damage_callback();
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	i2c_tmo = I2C_TIMEOUT;
	while ( (TWCR&(1<<TWSTO)) ) {
		if( --i2c_tmo == 0 ) {
			i2c_error_flag = 1;
			break;
		}
	}
#else
	SDA_LO;
	H_DEL;
	SCL_HI;
	Q_DEL;
	SDA_HI;
	H_DEL;
#endif
}

uint8_t i2c_write( uint8_t byte ) {

#if USE_SOFT_I2C == 0

	if( i2c_error_flag && i2c_damage_callback ) i2c_damage_callback();
	TWDR = byte;
	TWCR = (1<<TWINT)|(1<<TWEN);
	i2c_tmo = I2C_TIMEOUT;
	while ( !(TWCR&(1<<TWINT))) {
		if( --i2c_tmo == 0 ) {
			i2c_error_flag = 1;
			break;
		}
	}
	return (TWSR & 0xF8) == 0x18;
#else

	for( uint8_t i=0; i<8; i++ ) {
		SCL_LO;
		Q_DEL;

		if(byte & 0x80) SDA_HI;
		else SDA_LO;

		H_DEL;

		SCL_HI;
		H_DEL;

		while( (SCL_PIN & (1<<iSCL))==0 );

		byte <<= 1;
	}

	//The 9th clock (ACK Phase)
	SCL_LO;
	Q_DEL;

	SDA_HI;
	H_DEL;

	SCL_HI;
	H_DEL;

	uint8_t ack =! (SDA_PIN & (1<<iSDA));

	SCL_LO;
	H_DEL;

	return ack;
#endif
}

uint8_t i2c_read( uint8_t ack ) {

#if USE_SOFT_I2C == 0
	if( i2c_error_flag && i2c_damage_callback ) i2c_damage_callback();
	TWCR = (1<<TWINT)|(ack<<TWEA)|(1<<TWEN);
	i2c_tmo = I2C_TIMEOUT;
	while ( !(TWCR & (1<<TWINT))) {
		if( --i2c_tmo == 0 ) {
			i2c_error_flag = 1;
			break;
		}
	}
	return TWDR;
#else

	uint8_t data = 0x00;

	SDA_HI;
	H_DEL;

	for( uint8_t i=0; i<8; i++ ) {

		SCL_LO;
		H_DEL;
		SCL_HI;
		H_DEL;

		while( (SCL_PIN & (1<<iSCL)) == 0 );

		if( SDA_PIN & (1<<iSDA) ) data |= (0x80>>i);

	}

	SCL_LO;
	Q_DEL;						//Soft_I2C_Put_Ack

	if(ack) SDA_LO;
	else SDA_HI;

	H_DEL;

	SCL_HI;
	H_DEL;

	SCL_LO;
	H_DEL;

	return data;

#endif
}

void i2c_write_buf( uint8_t SLA, uint8_t adr, uint16_t len, uint8_t *buf ) {

	i2c_start();
	i2c_write(SLA);
	i2c_write(adr);
	while (len--) {
		i2c_write(*buf++);
		if( i2c_error_flag && i2c_damage_callback ) i2c_damage_callback();
	}
	i2c_stop();
}

// sequential write into SLAVE
void i2c_write_buf1( uint8_t SLA, uint16_t len, uint8_t *buf ) {

	i2c_start();
	i2c_write(SLA);
	while (len--) {
		i2c_write(*buf++);
		if( i2c_error_flag && i2c_damage_callback ) i2c_damage_callback();
	}
	i2c_stop();
}

void i2c_read_buf( uint8_t SLA, uint8_t adr, uint16_t len, uint8_t *buf ) {

	i2c_start();
	i2c_write(SLA);
	i2c_write(adr);
	i2c_start();
	i2c_write(SLA + 1);
	while (len--) {
		*buf++ = i2c_read( len ? ACK : NACK );
		if( i2c_error_flag && i2c_damage_callback ) i2c_damage_callback();
	}
	i2c_stop();
}

// sequential read from SLAVE
void i2c_read_buf1( uint8_t SLA, uint16_t len, uint8_t *buf ) {

	i2c_start();
	i2c_write(SLA);
	i2c_start();
	i2c_write(SLA + 1);
	while (len--) {
		*buf++ = i2c_read( len ? ACK : NACK );
		if( i2c_error_flag && i2c_damage_callback ) i2c_damage_callback();
	}
	i2c_stop();
}


void i2c_send_byte( uint8_t SLA, uint8_t byte ) {

	i2c_start();
	i2c_write(SLA);
	i2c_write( byte );
	i2c_stop();

}

uint8_t i2c_read_byte( uint8_t SLA ) {

	uint8_t res = 0;
	i2c_start();
	i2c_write(SLA+1);
	res = i2c_read( NACK );
	i2c_stop();

	return res;
}


void i2c_send_word( uint8_t SLA, uint16_t data ) {

	i2c_start();
	i2c_write(SLA);
	i2c_write( data >> 8 );
	i2c_write( data >> 0 );
	i2c_stop();
}

uint16_t i2c_read_word( uint8_t SLA ) {

	uint16_t res = 0;
	uint8_t * buf = (uint8_t*)&res;
	uint8_t len = 2;

	i2c_start();
	i2c_write(SLA+1);
	while (len--) *buf++ = i2c_read( len ? ACK : NACK );
	i2c_stop();

	return res;
}

#endif			//************ MODE 0 - END

#if I2C_MODE == 1

static void (*i2c_slave_rx_callback)(uint8_t);
static void (*i2c_slave_req_callback)(void);
static void (*i2c_slave_req_next_callback)(void);

void i2c_slave_reg_callbacks(void (*rx)(uint8_t), void (*req)(), void (*req1)()) {
	i2c_slave_rx_callback = rx;
	i2c_slave_req_callback = req;
	i2c_slave_req_next_callback = req1;
}

void i2c_slave_init( uint8_t sla ) {
	uint8_t sreg = SREG;
	cli();
	// ustaw adres uk³adu SLAVE - adres 8 bitowy - kasowanie najm³odszego bitu
	TWAR = sla & 0xFE;

	TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
	SREG = sreg;
}

void i2c_slave_stop( void ) {
	uint8_t sreg = SREG;
	cli();
	TWCR = 0;
	TWAR = 0;
	SREG = sreg;
}

ISR( TWI_vect ) {

	switch ( TW_STATUS) {
	case TW_SR_DATA_ACK:
		if (i2c_slave_rx_callback)
			i2c_slave_rx_callback( TWDR);
		TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
		break;
	case TW_ST_SLA_ACK:
		// master ¿¹da danych - pojedynczy bajt
		if (i2c_slave_req_callback)
			i2c_slave_req_callback();
		TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
		break;
	case TW_ST_DATA_ACK:
		// master ¿¹da danych - przy sekwencyjnym odczycie ze SLAVE - master pobiera kolejne bajty
		if (i2c_slave_req_next_callback)
			i2c_slave_req_next_callback();
		TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
		break;
	case TW_BUS_ERROR:
		// w przypadku nieprzewidzianych awarii na magistrali
		TWCR = 0;
		TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
		break;
	default:
		TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
		break;
	}
}

#endif		//************ MODE 1 - END

#if I2C_MODE == 2

static void (*i2c_slave_rx_callback)(uint8_t);
static void (*i2c_slave_req_callback)(void);
static void (*i2c_slave_req_next_callback)(void);

void i2c_slave_reg_callbacks(void (*rx)(uint8_t), void (*req)(), void (*req1)()) {
	i2c_slave_rx_callback = rx;
	i2c_slave_req_callback = req;
	i2c_slave_req_next_callback = req1;
}



#if 	defined( __AVR_ATtiny2313__ )
		#define DDR_USI             DDRB
		#define PORT_USI            PORTB
		#define PIN_USI             PINB
		#define PORT_USI_SDA        PB5
		#define PORT_USI_SCL        PB7
		#define PIN_USI_SDA         PINB5
		#define PIN_USI_SCL         PINB7
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect

#elif   defined( __AVR_ATtiny24A__ ) | \
        defined( __AVR_ATtiny44A__ ) | \
        defined( __AVR_ATtiny24__ )  | \
        defined( __AVR_ATtiny44__ )  | \
        defined( __AVR_ATtiny84__ )
		#define DDR_USI             DDRA
		#define PORT_USI            PORTA
		#define PIN_USI             PINA
		#define PORT_USI_SDA        PA6
		#define PORT_USI_SCL        PA4
		#define PIN_USI_SDA         PINA6
		#define PIN_USI_SCL         PINA4
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_STR_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect

#elif 	defined( __AVR_ATtiny25__ ) | \
		defined( __AVR_ATtiny45__ ) | \
		defined( __AVR_ATtiny85__ )
		#define DDR_USI             DDRB
		#define PORT_USI            PORTB
		#define PIN_USI             PINB
		#define PORT_USI_SDA        PB0
		#define PORT_USI_SCL        PB2
		#define PIN_USI_SDA         PINB0
		#define PIN_USI_SCL         PINB2
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect

#elif 	defined( __AVR_ATtiny26__ )
		#define DDR_USI             DDRB
		#define PORT_USI            PORTB
		#define PIN_USI             PINB
		#define PORT_USI_SDA        PB0
		#define PORT_USI_SCL        PB2
		#define PIN_USI_SDA         PINB0
		#define PIN_USI_SCL         PINB2
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_STRT_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect

#elif 	defined( __AVR_ATtiny261__ ) | \
		defined( __AVR_ATtiny461__ ) | \
		defined( __AVR_ATtiny861__ )
		#define DDR_USI             DDRB
		#define PORT_USI            PORTB
		#define PIN_USI             PINB
		#define PORT_USI_SDA        PB0
		#define PORT_USI_SCL        PB2
		#define PIN_USI_SDA         PINB0
		#define PIN_USI_SCL         PINB2
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect

#elif 	defined( __AVR_ATmega165__ ) | \
		defined( __AVR_ATmega325__ ) | \
		defined( __AVR_ATmega3250__ ) | \
		defined( __AVR_ATmega645__ ) | \
		defined( __AVR_ATmega6450__ ) | \
		defined( __AVR_ATmega329__ ) | \
		defined( __AVR_ATmega3290__ )
		#define DDR_USI             DDRE
		#define PORT_USI            PORTE
		#define PIN_USI             PINE
		#define PORT_USI_SDA        PE5
		#define PORT_USI_SCL        PE4
		#define PIN_USI_SDA         PINE5
		#define PIN_USI_SCL         PINE4
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect

#elif 	defined( __AVR_ATmega169__ )
		#define DDR_USI             DDRE
		#define PORT_USI            PORTE
		#define PIN_USI             PINE
		#define PORT_USI_SDA        PE5
		#define PORT_USI_SCL        PE4
		#define PIN_USI_SDA         PINE5
		#define PIN_USI_SCL         PINE4
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect

#else
		#error "no USI-Slave definition for MCU available"

#endif



#define SET_USI_TO_SEND_ACK( ) 	{ USIDR = 0; \
								DDR_USI |= ( 1 << PORT_USI_SDA ); \
								USISR = ( 0 << USI_START_COND_INT ) | \
								( 1 << USIOIF ) | ( 1 << USIPF ) | \
								( 1 << USIDC )| \
								( 0x0E << USICNT0 );}

#define SET_USI_TO_READ_ACK( ) 	{ USIDR = 0; \
								DDR_USI &= ~( 1 << PORT_USI_SDA ); \
								USISR = ( 0 << USI_START_COND_INT ) | \
								( 1 << USIOIF) | \
								( 1 << USIPF ) | \
								( 1 << USIDC ) | \
								( 0x0E << USICNT0 );}

#define SET_USI_TO_TWI_START_CONDITION_MODE( ) { \
								USICR = ( 1 << USISIE ) | ( 0 << USIOIE ) | \
								( 1 << USIWM1 ) | ( 0 << USIWM0 ) | \
								( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
								( 0 << USITC ); \
								USISR = ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
								( 1 << USIDC ) | ( 0x0 << USICNT0 ); }

#define SET_USI_TO_SEND_DATA( ) { DDR_USI |=  ( 1 << PORT_USI_SDA ); \
								USISR = ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
								( 1 << USIDC) | \
								( 0x0 << USICNT0 ); \
								}

#define SET_USI_TO_READ_DATA( ) { DDR_USI &= ~( 1 << PORT_USI_SDA ); \
								USISR =	( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | \
								( 1 << USIPF ) | ( 1 << USIDC ) | \
								( 0x0 << USICNT0 ); \
								}




typedef enum {
	USI_SLAVE_CHECK_ADDRESS                = 0x00,
	USI_SLAVE_SEND_DATA                    = 0x01,
	USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA = 0x02,
	USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   = 0x03,
	USI_SLAVE_REQUEST_DATA                 = 0x04,
	USI_SLAVE_GET_DATA_AND_SEND_ACK        = 0x05
} TOSTATE;



 volatile uint8_t         	slaveAddress;
 volatile TOSTATE 			tostate;


void i2c_slave_init( uint8_t sla ) {

  slaveAddress = sla;

  DDR_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );
  PORT_USI |= ( 1 << PORT_USI_SCL );
  PORT_USI |= ( 1 << PORT_USI_SDA );
  DDR_USI &= ~( 1 << PORT_USI_SDA );
  USICR =
       ( 1 << USISIE ) |
       ( 0 << USIOIE ) |
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
       ( 0 << USITC );
  USISR = ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC );
}



ISR( USI_START_VECTOR )
{
	tostate = USI_SLAVE_CHECK_ADDRESS;
	DDR_USI &= ~( 1 << PORT_USI_SDA );

	while (	( PIN_USI & ( 1 << PIN_USI_SCL ) ) &&	!( ( PIN_USI & ( 1 << PIN_USI_SDA ) ) ));

	if ( !( PIN_USI & ( 1 << PIN_USI_SDA ) ) ) {
		USICR =
		( 1 << USISIE ) |
		( 1 << USIOIE ) |
		( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
		( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
		( 0 << USITC );

	} else {
		USICR =
		( 1 << USISIE ) |
		( 0 << USIOIE ) |
		( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
		( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
		( 0 << USITC );
	}

	USISR =
	( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) |
	( 1 << USIPF ) |( 1 << USIDC ) |
	( 0x0 << USICNT0);
}


ISR( USI_OVERFLOW_VECTOR ) {

	uint8_t data=0;

	switch ( tostate )
		{
		case USI_SLAVE_CHECK_ADDRESS:
			if (USIDR == 0 || (USIDR & ~1) == slaveAddress) {
				if (  USIDR & 0x01 ) {
					tostate = USI_SLAVE_SEND_DATA;
				} else {
					tostate = USI_SLAVE_REQUEST_DATA;
				}
				SET_USI_TO_SEND_ACK();
			} else {
				SET_USI_TO_TWI_START_CONDITION_MODE();
			}
			break;


		case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
			if ( USIDR ) {
				SET_USI_TO_TWI_START_CONDITION_MODE();
				break;
			}
			if( i2c_slave_req_next_callback ) i2c_slave_req_next_callback();
			tostate = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
			SET_USI_TO_SEND_DATA( );
			break;

		case USI_SLAVE_SEND_DATA:

			if( i2c_slave_req_callback ) i2c_slave_req_callback();

			tostate = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
			SET_USI_TO_SEND_DATA( );
			break;

		case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
			tostate = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
			SET_USI_TO_READ_ACK( );
			break;

		case USI_SLAVE_REQUEST_DATA:
			tostate = USI_SLAVE_GET_DATA_AND_SEND_ACK;
			SET_USI_TO_READ_DATA( );
			break;

		case USI_SLAVE_GET_DATA_AND_SEND_ACK:
			data = USIDR; 					// Read data received
			if( i2c_slave_rx_callback ) i2c_slave_rx_callback( data );

			tostate = USI_SLAVE_REQUEST_DATA;	// Next USI_SLAVE_REQUEST_DATA
				SET_USI_TO_SEND_ACK( );
			break;


		} // End switch
} // End ISR( USI_OVERFLOW_VECTOR )

#endif		//************ MODE 2 - END
