/*
 * mk_i2c.h
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

#ifndef I2C_TWI_USI_H_
#define I2C_TWI_USI_H_

//------------ konfiguracja biblioteki -------------------------

//^^^^^^^^^^^^^^^^^^^^^^^^^^ konfiguracja I2C SLAVE tylko TWI lub USI ^^^^^^^^^
#define I2C_MODE			0			// 0 - I2C MASTER
										// 1 - for ATmega uC's with TWI
										// 2 - for ATtiny uC's with USI ( ATtiny 2313 / 25/45/85 / 24(A)/44(A)/84 / 26 / 261/461/861 )

#define I2C_SLAVE_ADDRESS	0x10		// range: 0-255 (only for MODE = 1,2
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


//.......................... ustawienia pinów SOFT I2C .........

#define USE_SOFT_I2C		0			// soft I2C only for MODE = 0 (Master)

#define iSCL		PC0
#define iSDA		PC1

#define SCL_PORT	PORTC
#define SCL_DIR		DDRC
#define SCL_PIN		PINC

#define SDA_PORT	PORTC
#define SDA_DIR		DDRC
#define SDA_PIN		PINC
//...............................................................


//--------------- koniec konfiguracji I2C -----------------------



//------------------- funkcje I2C MASTER (TWI/Soft) MODE = 0 -------------------------------------------------------------

#if I2C_MODE == 0

#define I2C_TIMEOUT 0xFFFF


#define ACK 	1
#define NACK 	0

#define SDA_LO		SDA_DIR |= (1<<iSDA)
#define SDA_HI		SDA_DIR &= ~(1<<iSDA)

#define SCL_LO		SCL_DIR |= (1<<iSCL)
#define SCL_HI		SCL_DIR &= ~(1<<iSCL)



extern uint8_t i2c_error_flag;


int i2c_init( uint16_t bitrateKHz );

// ustawiamy prêdkoœæ na I2C w kHz (standardowa prêdkoœæ to 100 kHz)  i2cSetBitrate( 100 );
int i2cSetBitrate( uint16_t bitrateKHz );

void register_i2c_damage_event_callback( void (*callback)(void) );

void i2c_start( void );
void i2c_stop( void );
uint8_t i2c_write( uint8_t byte );
uint8_t i2c_read( uint8_t ack );

// write N-bytes from buf into SLAVE from address
void i2c_write_buf( uint8_t SLA, uint8_t adr, uint16_t len, uint8_t *buf );
// read N-bytes from SLAVE from address into a buf
void i2c_read_buf( uint8_t SLA, uint8_t adr, uint16_t len, uint8_t *buf );

// sequential write into SLAVE
void i2c_write_buf1( uint8_t SLA, uint16_t len, uint8_t *buf );
// sequential read from SLAVE
void i2c_read_buf1( uint8_t SLA, uint16_t len, uint8_t *buf );


void i2c_send_byte( uint8_t SLA, uint8_t byte );
uint8_t i2c_read_byte( uint8_t SLA );

void i2c_send_word( uint8_t SLA, uint16_t data );
uint16_t i2c_read_word( uint8_t SLA );


//---------- PHILIPS I2C Expanders - PCF857x -----------------
void pcf8574_write( uint8_t SLA, uint8_t byte );
uint8_t pcf8574_read( uint8_t SLA );

void pcf8575_write( uint8_t SLA, uint16_t data );
uint16_t pcf8575_read( uint8_t SLA );

#endif



//----------- funkcje I2C SLAVE (TWI) - MODE = 1 -----------------------------------------------------------------------

#if I2C_MODE == 1
void i2c_slave_reg_callbacks( void (*rx)(uint8_t), void (*req)(), void (*req1)() );

void i2c_slave_init( uint8_t sla );
void i2c_slave_stop( void );

inline void __attribute__((always_inline)) i2c_slave_send_byte( uint8_t data ) {
  TWDR = data;
}
#endif


//----------- funkcje I2C SLAVE (USI) - MODE = 2 -----------------------------------------------------------------------

#if I2C_MODE == 2

void i2c_slave_reg_callbacks(void (*rx)(uint8_t), void (*req)(), void (*req1)());

void i2c_slave_init( uint8_t sla );

inline void __attribute__((always_inline)) i2c_slave_send_byte( uint8_t data ) {
  USIDR = data;
}
#endif


#endif /* I2C_TWI_USI_H_ */
