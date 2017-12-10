///////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <SPI.h>
#include "mrf24j40.h"


MRF24J40::MRF24J40( int cs_pin, int reset_pin, int irq_pin ) 
 : _cs_pin( cs_pin ), _reset_pin( reset_pin ), _irq_pin( irq_pin )
{ 
  // ... 
}

void MRF24J40::begin() {
  pinMode( _cs_pin, OUTPUT );
  spi_cs_high();

  if ( _irq_pin != -1 ) {
     pinMode( _irq_pin, INPUT_PULLUP );
  }
  
  if ( _reset_pin != -1 ) {
     pinMode( _reset_pin, OUTPUT );
     digitalWrite( _reset_pin, HIGH );
     reset(); 
  }

  SPI.begin();
  SPI.setDataMode( SPI_MODE0 ); // set SPI Mode 0,0
  SPI.setBitOrder( MSBFIRST  ); // send MSB first
  SPI.setClockDivider( SPI_CLOCK_DIV2 ); // set SCK freq. divider  
}

MRF24J40::~MRF24J40( ) {
}

inline void MRF24J40::spi_cs_low( ) {
  digitalWrite( _cs_pin, LOW );
}

inline void MRF24J40::spi_cs_high() {
  digitalWrite( _cs_pin, HIGH );
}

void MRF24J40::spi_transfern( uint8_t *buf, uint16_t len ) {
  spi_cs_low();
  SPI.beginTransaction( SPISettings( 8000000, MSBFIRST, SPI_MODE0 ) );
  for ( int i=0; i < len; i++ ) {
     buf[i] = SPI.transfer( buf[i] );
  }
  SPI.endTransaction(); 
  spi_cs_high();
}

/*
 *  Read data from short address of memory location in mrf24j40
 *  Returns the value in the memory location specified.
 *  param addr - Short address memory location
 */

// 0b0xxxxxx0, x = address bits (short address read)

uint8_t MRF24J40::readShortAddr( uint8_t addr ) {
  // SPI CS enable
  // transfer one byte : addr = ((addr & 0b00111111) << 1) 
  // transfer one byte : 0x00 and read byte data -> value
  // SPI CS disable
  // return received byte (value)

  // bit 0 must be 0 (read)
  spi_buf[0] = (addr & 0b00111111) << 1; 
  spi_buf[1] = 0x00;   // dummy byte
  spi_transfern( spi_buf, 2 ); 
  return spi_buf[1];
}       

/*
 *  Write data to short address memory location
 *  Sets the memory location to the value specified
 *  param addr - Short address memory location 
 *  param data - Value that the memory location should be set to
 */

// 0b0xxxxxx1, x = address bits (short address write)

void MRF24J40::writeShortAddr( uint8_t addr, uint8_t data ) {
  // SPI CS enable
  // transfer one byte : addr = ((addr & 0b00111111) << 1) | 1; 
  // transfer one byte : data
  // SPI CS disable

  // bit 0 must be 1 (write) 
  spi_buf[0] = ((addr & 0b00111111) << 1) | 0x01; 
  spi_buf[1] = data;  
  spi_transfern( spi_buf, 2 ); 
} 

/* 
 *  Read data from long address of memory location in mrf24j40
 *  Returns the value in the memory location specified.
 *  param addr - Long address memory location (see mrf24h40_defines.h)
 */

// 0b1xxxxxxx_xxx00000, x = address bits (long address read)

uint8_t MRF24J40::readLongAddr( uint16_t addr ) {
  // set addr = ((addr & 0b0000001111111111) << 5) | 0x8000;
  // SPI CS enable
  // transfer one byte : high-byte addr  
  // transfer one byte : low-byte addr  
  // transfer one byte : dummy byte -> read value
  // SPI CS disable
  // return received byte (value)

#if 0
  // bit 15 = 1 (long), bit 4 = 0 (read)
  addr = ((addr & 0b0000001111111111) << 5) | 0x8000;
  spi_buf[0] = addr >> 8;    // high-byte address
  spi_buf[1] = addr & 0xff;  // low-byte address
  spi_buf[2] = 0x00;         // dummy byte
#else
  spi_buf[0] = 0x80 | (addr >> 3);  // high-byte address
  spi_buf[1] = (addr << 5) ;        // low-byte address
  spi_buf[2] = 0x00;                // dummy byte
#endif  

  spi_transfern( spi_buf, 3 ); 
  return spi_buf[2];
}       

/* 
 * Write data to long address memory location
 * Sets the memory location to the value specified 
 * param addr - Long address memory location 
 * param data - Value that the memory location should be set to
 */

// 0b1xxxxxxx_xxx10000, x = address bits (long address write)

void MRF24J40::writeLongAddr( uint16_t addr, uint8_t data ) {
  // set addr = ((addr & 0b0000001111111111) << 5) | 0x8010;
  // SPI CS enable
  // transfer one byte : high-byte addr  
  // transfer one byte : low-byte addr  
  // transfer one byte : data

#if 0
  // bit 15 = 1 (long), bit  4 = 1 (write)
  addr = ((addr & 0b0000001111111111) << 5) | 0x8010;
  spi_buf[0] = addr >> 8;    // high-byte address
  spi_buf[1] = addr & 0xff;  // low-byte address
  spi_buf[2] = data;         // data byte
#else
  spi_buf[0] = 0x80 | (addr >> 3);  // high-byte address
  spi_buf[1] = 0x10 | (addr << 5);  // low-byte address
  spi_buf[2] = data;         // data byte
#endif  

  spi_transfern( spi_buf, 3 ); 
}

bool MRF24J40::is_irq_active() { // is INT (interrupt) pin active (active-low)
  if ( _irq_pin == -1 ) {
    return false;
  }
  return ( digitalRead( _irq_pin) == LOW) ? true : false;
}

void MRF24J40::reset( uint16_t duration ) {
  if ( _reset_pin == -1 ) {
    return;
  }
  digitalWrite( _reset_pin, LOW );
  delay( duration );
  digitalWrite( _reset_pin, HIGH );
}
///////////////////////////////////////////////////////////////////////////////////

