///////////////////////////////////////////////////////////////////////
// File: mrf24j40.h
// Author: RSP @ KMUTNB
// Date: 2013-03-24

///////////////////////////////////////////////////////////////////////
#ifndef __MRF24J40_H__
#define __MRF24J40_H__
///////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <assert.h>
#include <SPI.h>

// RXMCR Register
#define MRF_RXMCR    0x00

// PAN ID Registers
#define MRF_PANIDL    0x01
#define MRF_PANIDH    0x02

// Short Address Registers
#define MRF_SADRL   0x03
#define MRF_SADRH   0x04

// Long (Extended) Address Registers
#define MRF_EADR0   0x05
#define MRF_EADR1   0x06
#define MRF_EADR2   0x07
#define MRF_EADR3   0x08
#define MRF_EADR4   0x09
#define MRF_EADR5   0x0A
#define MRF_EADR6   0x0B
#define MRF_EADR7   0x0C

// RXFLUSH Register
#define MRF_RXFLUSH   0x0D

// 0x0E - Reserved
// 0x0F - Reserved

// ORDER Register
#define MRF_ORDER     0x10

// TXMCR Register
#define MRF_TXMCR     0x11

// ACKTMOUT Register
#define MRF_ACKTMOUT  0x12

// ESLOTG1 Register
#define MRF_ESLOTG1   0x13

// SYMTICKL Register
#define MRF_SYMTICKL  0x14

// SYMTICKH Register
#define MRF_SYMTICKH  0x15

// PACON0..PACON2 Register
#define MRF_PACON0    0x16
#define MRF_PACON1    0x17
#define MRF_PACON2    0x18

// 0x19 - Reserved

// TXBCON0 Register
#define MRF_TXBCON0   0x1A

// TXNCON Register
#define MRF_TXNCON    0x1B

// TXG1CON1 and TXG2CON Register
#define MRF_TXG1CON   0x1C
#define MRF_TXG2CON   0x1D

// ESLOTG23 Register
#define MRF_ESLOTG23  0x1E

// ESLOTG45 Register
#define MRF_ESLOTG45  0x1F

// ESLOTG67 Register
#define MRF_ESLOTG67  0x20

// TXPEND Register
#define MRF_TXPEND    0x21

// WAKECON Register
#define MRF_WAKECON   0x22

// FRMOFFSET Register
#define MRF_FRMOFFSET 0x23

// TXSTAT Register
#define MRF_TXSTAT    0x24

// TXBCON1 Register
#define MRF_TXBCON1   0x25

// GATECLK Register
#define MRF_GATECLK   0x26

// TXTIME Register
#define MRF_TXTIME    0x27

// HSYMTMRL Register
#define MRF_HSYMTMRL  0x28

// HSYMTMRH Register
#define MRF_HSYMTMRH  0x29

// SOFTRST Register
#define MRF_SOFTRST   0x2A

// 0x2B - Reserved

// SECCON0 Register
#define MRF_SECCON0   0x2C

// SECCON1 Register
#define MRF_SECCON1   0x2D

// TXSTBL Register
#define MRF_TXSTBL    0x2E

// 0x2F - Reserved

// RXSR Register
#define MRF_RXSR      0x30

// INTSTAT Register
#define MRF_INTSTAT   0x31

// MRF_INTCON Register
#define MRF_INTCON    0x32

// GPIO Register
#define MRF_GPIO      0x33

// TRISGPIO Register
#define MRF_TRISGPIO  0x34

// SLPACK Register
#define MRF_SLPACK    0x35

// RFCTL Register
#define MRF_RFCTL     0x36

// SECCR2 Register
#define MRF_SECCR2    0x37

// BBREG0 .. BBREG4, BBREG6 Register
#define MRF_BBREG0    0x38
#define MRF_BBREG1    0x39
#define MRF_BBREG2    0x3A
#define MRF_BBREG3    0x3B
#define MRF_BBREG4    0x3C

// 0x3D - Reserved

#define MRF_BBREG6    0x3E

// CCAEDTH Register
#define MRF_CCAEDTH   0x3F

// RFCON0 Register
#define MRF_RFCON0    0x200

// RFCON1 .. RFCON3 Register
#define MRF_RFCON1    0x201
#define MRF_RFCON2    0x202
#define MRF_RFCON3    0x203

// 0x204 - Reserved

// RFCON5 .. RFCON8 Register
#define MRF_RFCON5    0x205
#define MRF_RFCON6    0x206
#define MRF_RFCON7    0x207
#define MRF_RFCON8    0x208

// SLPCAL0 Register
#define MRF_SLPCAL0   0x209

// SLPCAL1, SLPCAL2 Register
#define MRF_SLPCAL1   0x20A
#define MRF_SLPCAL2   0x20B

// 0x20C - Reserved
// 0x20D - Reserved
// 0x20E - Reserved

// RFSTATE Register
#define MRF_RFSTATE   0x20F

// RSSI Register
#define MRF_RSSI    0x210

// SLPCON0 Register
#define MRF_SLPCON0   0x211

// 0x212 - Reserved
// 0x213 - Reserved
// 0x214 - Reserved
// 0x215 - Reserved
// 0x216 - Reserved
// 0x217 - Reserved
// 0x218 - Reserved
// 0x219 - Reserved
// 0x21A - Reserved
// 0x21B - Reserved
// 0x21C - Reserved
// 0x21D - Reserved
// 0x21E - Reserved
// 0x21F - Reserved

// SLPCON1 Register
#define MRF_SLPCON1   0x220

// 0x221 - Reserved

// WAKETIMEL Register
#define MRF_WAKETIMEL 0x222

// WAKETIMEH Register
#define MRF_WAKETIMEH 0x223

// REMCNTL Register
#define MRF_REMCNTL   0x224

// REMCNTH Register
#define MRF_REMCNTH   0x225

// MAINCNT0 .. MAINCNT3 Register
#define MRF_MAINCNT0  0x226
#define MRF_MAINCNT1  0x227
#define MRF_MAINCNT2  0x228
#define MRF_MAINCNT3  0x229

// 0x22A - Reserved
// 0x22B - Reserved
// 0x22C - Reserved
// 0x22D - Reserved
// 0x22E - Reserved

// TESTMODE Register
#define MRF_TESTMODE  0x22F

#define MRF_ASSOEADR0 0x230
#define MRF_ASSOEADR1 0x231
#define MRF_ASSOEADR2 0x232
#define MRF_ASSOEADR3 0x233
#define MRF_ASSOEADR4 0x234
#define MRF_ASSOEADR5 0x235
#define MRF_ASSOEADR6 0x236
#define MRF_ASSOEADR7 0x237
#define MRF_ASSOSADR0 0x238
#define MRF_ASSOSADR1 0x239

// 0x23A - Reserved
// 0x23B - Reserved
// 0x23C - Unimplemented
// 0x23D - Unimplemented
// 0x23E - Unimplemented
// 0x23F - Unimplemented

#define MRF_UPNONCE0  0x240
#define MRF_UPNONCE1  0x241
#define MRF_UPNONCE2  0x242
#define MRF_UPNONCE3  0x243
#define MRF_UPNONCE4  0x244
#define MRF_UPNONCE5  0x245
#define MRF_UPNONCE6  0x246
#define MRF_UPNONCE7  0x247
#define MRF_UPNONCE8  0x248
#define MRF_UPNONCE9  0x249
#define MRF_UPNONCE10 0x24A
#define MRF_UPNONCE11 0x24B
#define MRF_UPNONCE12 0x24C

#define MRF_TX_NORMAL_FIFO  0x000
#define MRF_TX_BEACON_FIFO  0x080
#define MRF_TX_GTS1_FIFO    0x100
#define MRF_TX_GTS2_FIFO    0x180
#define MRF_SKEY_FIFO       0x280
#define MRF_RX_FIFO  0x300

///////////////////////////////////////////////////////////////////////

// see MRF_RXMCR register
union _mrf_reg {
  uint8_t value;
  struct {
    uint8_t _BIT0   : 1;
    uint8_t _BIT1   : 1;
    uint8_t _BIT2   : 1;
    uint8_t _BIT3   : 1;
    uint8_t _BIT4   : 1;
    uint8_t _BIT5   : 1;
    uint8_t _BIT6   : 1; 
    uint8_t _BIT7   : 1;
  } bits;
  struct {
    uint8_t PROMI        : 1;
    uint8_t ERRPKT       : 1;
    uint8_t COORD        : 1;
    uint8_t PANCOORD     : 1;
    uint8_t _reserved4   : 1;
    uint8_t NOACKRSP     : 1;
    uint8_t _reserved6   : 1;
    uint8_t _reserved7   : 1;
  } rxmcr;
  struct {
    uint8_t RXFLUSH      : 1;
    uint8_t BCNONLY      : 1;
    uint8_t DATAONLY     : 1;
    uint8_t CMDONLY      : 1;
    uint8_t _reserved4   : 1;
    uint8_t WAKEPAD      : 1;
    uint8_t WAKEPOL      : 1;
    uint8_t _reserved7   : 1; 
  } rxflush;
  struct {
    uint8_t CSMABF       : 3;  
    uint8_t MACMINBE     : 2; 
    uint8_t SLOTTED      : 1;  
    uint8_t BATLIFEXT    : 1;  
    uint8_t NOCSMA       : 1; 
  } txmcr;
  struct {
    uint8_t TXNTRIG      : 1; 
    uint8_t TXNSECEN     : 1; 
    uint8_t TXNACKREQ    : 1; 
    uint8_t INDIRECT     : 1; 
    uint8_t FPSTAT       : 1; 
    uint8_t _reserved5   : 1;
    uint8_t _reserved6   : 1;
    uint8_t _reserved7   : 1; 
  } txncon;
  struct {
    uint8_t TXNIF        : 1; 
    uint8_t TXG1IF       : 1; 
    uint8_t TXG2IF       : 1; 
    uint8_t RXIF         : 1; 
    uint8_t SECIF        : 1; 
    uint8_t HSYMTMRIF    : 1; 
    uint8_t WAKEIF       : 1; 
    uint8_t SLPIF        : 1; 
  } intstat;
  struct {
    uint8_t TXNSTAT      : 1; 
    uint8_t TXG1STAT     : 1; 
    uint8_t TXG2STAT     : 1; 
    uint8_t TXG1FNT      : 1; 
    uint8_t TXG2FNT      : 1; 
    uint8_t CCAFAIL      : 1; 
    uint8_t TXNRETRY     : 2;
  } txstat;
};

typedef union _mrf_reg mrf_reg_t;

struct _rx_packet {
  uint32_t ts_sec;
  uint32_t ts_usec;
  uint8_t  frame_len;
  uint8_t  frame_data[128];
  uint8_t  lqi;
  uint8_t  rssi;
};

typedef _rx_packet rx_packet_t;

///////////////////////////////////////////////////////////////////////

class MRF24J40 {
  public:
    MRF24J40( int cs_pin, int reset_pin=-1, int irq_pin=-1 );
    virtual ~MRF24J40( );
    void     begin();
    uint8_t  readShortAddr( uint8_t addr );
    uint8_t  readLongAddr( uint16_t addr );
    void     writeShortAddr( uint8_t addr, uint8_t data );
    void     writeLongAddr( uint16_t addr, uint8_t data );
    bool     is_irq_active();
    void     sendResetPulse( uint16_t duration = 100 /* msec */ );

 protected:
    void spi_cs_low();
    void spi_cs_high();
    void spi_transfern( uint8_t *buf, uint16_t len );

 private:
    uint8_t spi_buf[ 128 ];
    int _cs_pin, _reset_pin, _irq_pin;

};

#endif /* __MRF24J40_H__ */
///////////////////////////////////////////////////////////////////////


