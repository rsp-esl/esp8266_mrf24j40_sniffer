///////////////////////////////////////////////////////////////////////////////////////
// Author: Rawat S. 
//         Department of Electrical & Computer Engineering (ECE)
//         Faculty of Engineering, KMUTNB, Thailand
//
// Hardware/Software used for testing: 
//   - ESP8266 (WeMos D1 mini) + MRF24J40MA module
//   - Arduino IDE 1.8.2 / ESP8266 package v2.3.0 
// 
// Date: 2017-12-03
///////////////////////////////////////////////////////////////////////////////////////

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include "mrf24j40.h"

// Specify the radio channel 
// (according to IEEE 802.15.4: select channel number between 11..26)
int channel = 11; 

// Specify the PAN ID (a 16-bit hex integer value)
#define PAN_ID                (0xFFFF)

// Specify the short (16-bit) network address for the sender
#define SENDER_ADDR           (0x0000)

// Specify the baudrate of the serial port for debugging purposes
#define BAUD_RATE             (1000000) 

IPAddress server_ip(192, 168, 1, 9); // specify the IP address of the remote machine (UDP server)
int server_port = 8888;     // UDP port number of the remote machine
int local_port  = 22222;    // local UDP port

WiFiUDP udp;

const char* WIFI_SSID = "XXXXXXXXXXXX";
const char* WIFI_PASS = "XXXXXXXXXXXX";

///////////////////////////////////////////////////////////////////////////////////////
// Pin definitions for ESP8266 (NodeMCU, WeMos D1 mini, ...)
/*
 * --------------------
 * SCK     GPIO-14  D5
 * MISO    GPIO-15  D6
 * MOSI    GPIO-16  D7  
 * CS      GPIO-17  D8
 * --------------------
 * /RESET  GPIO-5   D1
 * INT     GPIO-4   D2
 * --------------------
 */
#define CS_PIN   D8             // the chip select pin of MRF24J40 (active-low)
#define RST_PIN  D1             // the reset pin of MRF24J40
#define INT_PIN  D2             // the interrupt pin of MRF24J40 (active-low)

///////////////////////////////////////////////////////////////////////////////////////

MRF24J40 mrf( CS_PIN /*cs*/, RST_PIN /*reset*/, INT_PIN /*irq*/ ); 

volatile bool irq_rx_flag = false; // interrupt (INT) request flag for RX reception
uint8_t frame_buf[132];

uint32_t ts;                    // used to keep timestamp
char buf[256];                  // string buffer
rx_packet_t rx_packet;          // used for the received packet
rx_packet_t rx_packet_saved;     

void rf_default_settings() {
   mrf.writeShortAddr( MRF_PACON2,  0x98 );
   mrf.writeShortAddr( MRF_TXSTBL,  0x95 );
   mrf.writeShortAddr( MRF_BBREG2,  0x80 );         // set CCA mode to ED (energy detection)
   mrf.writeShortAddr( MRF_CCAEDTH, 0x60 );         // set CCA ED thredshold value
   mrf.writeShortAddr( MRF_ORDER,   0xFF );         // set BO=15 and SO=15

   mrf.writeShortAddr( MRF_ACKTMOUT, 0x80 | 0x3F ); // set the ACK timeout, set request ACK bit   
   mrf.writeShortAddr( MRF_BBREG6,  0x40 );         // set appended RSSI value to RXFIFO
  
   mrf.writeLongAddr( MRF_RFCON0,   0x03 );
   mrf.writeLongAddr( MRF_RFCON1,   0x01 );
   mrf.writeLongAddr( MRF_RFCON2,   0x80 );
   mrf.writeLongAddr( MRF_RFCON6,   0x90 );
   mrf.writeLongAddr( MRF_RFCON7,   0x80 );
   mrf.writeLongAddr( MRF_RFCON8,   0x10 );
   mrf.writeLongAddr( MRF_SLPCON1,  0x21 );

   //  INTEDGE = 0 : falling-edge or active-low
   // #SLPCKEN = 0 : enable low-power clock selection
   mrf.writeLongAddr( MRF_SLPCON0,  0x00 );

   // RXIE  = 0:  Enable RX FIFO reception interrupt
   // TXNIE = 1:  Disable TX Normal FIFO transmission interrupt
   mrf.writeShortAddr( MRF_INTCON,  0b11110111 ); 

#ifdef USE_MRF24J40MB
   // enable PA LNA for MRF24J40MB
   mrf.writeShortAddr( MRF_GPIO,     0x00 );
   mrf.writeShortAddr( MRF_TRISGPIO, 0x00 );
   mrf.writeLongAddr( MRF_TESTMODE,  0x07 );     // set TESTMODE bits [2..0] = 0b111 (enable LNA power)
   mrf.writeLongAddr( MRF_RFCON3,    0x18 );     // default TX power setting
#else
   mrf.writeLongAddr( MRF_RFCON3,    0x00 );     // default TX power setting /* 0dB */
#endif

   mrf.writeShortAddr( MRF_RFCTL,    0x04 );     // set RFCTL = 0x04 (reset RF state machine)
   mrf.writeShortAddr( MRF_RFCTL,    0x00 );     // set RFCTL = 0x00 (release RF reset)

   delayMicroseconds( 200 );                     // delay for at least 192us
}

// set the TX power level
void rf_set_tx_power( uint8_t level ) {
   mrf.writeLongAddr( MRF_RFCON3, level & 0b11111000 );
}

// get the TX power level
uint8_t rf_get_tx_power( ) {
   return mrf.readLongAddr( MRF_RFCON3 );
}

// set radio channel between 11..26
void rf_set_channel( uint8_t channel ) { /* channel: 11..26 */
   if (channel < 11) { channel = 11; }
   else if (channel > 26) { channel = 26; }
   channel = (channel - 11) & 0x0f;
   mrf.writeLongAddr( MRF_RFCON0, ((channel << 4) | 0x03) );
}

// set the IEEE 802.15.4 (64-bit) extended address
void rf_set_device_long_addr( uint8_t *addr_bytes ) {
   uint8_t i;
   for ( i=0; i < 8; i++) { // high-byte first
      mrf.writeShortAddr( MRF_EADR7-i, addr_bytes[i] );
   }
}

// set the 16-bit short address of the MRF24J40 module.
void rf_set_device_short_addr( uint16_t addr ) {
   mrf.writeShortAddr( MRF_SADRH, addr >> 8 );   // high-byte
   mrf.writeShortAddr( MRF_SADRL, addr & 0xff ); // low-byte
}

// set the 16-bit PAN ID of the network
void rf_set_pan_id( uint16_t pan_id ) {
   mrf.writeShortAddr( MRF_PANIDH, pan_id >> 8 );   // high-byte
   mrf.writeShortAddr( MRF_PANIDL, pan_id & 0xff ); // low-byte
}

// get the radio channel: 11..26
uint8_t rf_get_channel() {
   uint8_t value = mrf.readLongAddr( MRF_RFCON0 );
   value = ((value >> 4) & 0x0f) + 11;
   return value;
}

// get the 16-bit short device address
uint16_t rf_get_device_short_addr( ) {
   uint8_t value = mrf.readShortAddr( MRF_SADRH );
   uint16_t addr = value << 8;
   value = mrf.readShortAddr( MRF_SADRL );
   addr |= value;
   return addr;
}

// get the 16-bit PAN ID
uint16_t rf_get_pan_id( ) {
   uint8_t value = mrf.readShortAddr( MRF_PANIDH );
   uint16_t addr = value << 8;
   value = mrf.readShortAddr( MRF_PANIDL );
   addr |= value;
   return addr;
}

// flush RX Buffer
inline void rf_flush_rx_buffer() {
   mrf_reg_t _reg;
   _reg.value = mrf.readShortAddr( MRF_RXFLUSH );  // read the RXFLUSH register
   _reg.rxflush.RXFLUSH = 1;      // set RXFLUSH bit = 1 (Reset the RX buffer pointer to 0)
   mrf.writeShortAddr( MRF_RXFLUSH, _reg.value );  // write the new value to the register
}

void irq_handler() {
   mrf.writeShortAddr( MRF_BBREG1, 0x40 );    // disable RX (set RXDECINV bit)
   // read data from RX FIFO and save to frame buffer (frame_buf)
   uint16_t frame_pos = MRF_RX_FIFO;
   int len = mrf.readLongAddr( frame_pos++ ); // read the frame length
   uint8_t *ptr = (uint8_t *)&frame_buf[0];
   *ptr++ = len; // save the frame length
   for ( int i=0; i < 5; i++ ) {
      *ptr++ = mrf.readLongAddr( frame_pos++ );
   }
   if ( len == 5 ) {
      *ptr++ = mrf.readLongAddr( frame_pos++ );  // LQI
      *ptr++ = mrf.readLongAddr( frame_pos++ );  // RSSI
   }
   rf_flush_rx_buffer();
   mrf.writeShortAddr( MRF_BBREG1, 0x00 );    // enable RX (clear RXDECINV bit)
   irq_rx_flag = true;
   mrf.readShortAddr( MRF_INTSTAT );  // read the status and clear interrupt
}

bool rf_packet_receive( rx_packet_t *pkt ) {
   if ( !irq_rx_flag ) {
      return false;
   }
   irq_rx_flag = false;
   int len = frame_buf[0];
   uint8_t *ptr =  pkt->frame_data;
   memcpy( ptr, (const uint8_t *)&frame_buf[1], len );
   if ( len == 5 ) { // ACK frame
      pkt->lqi  = frame_buf[len+1];
      pkt->rssi = frame_buf[len+2];
      pkt->frame_len = len;
   } else {
      uint16_t frame_pos = MRF_RX_FIFO + 6; // skip the first 6 bytes in RX FIFO
      ptr = ptr+5; // skip the first 5 bytes in data buffer
      for ( int i=5; i < len; i++ ) {
        *ptr++ = mrf.readLongAddr( frame_pos++ );
      }
      pkt->lqi  = mrf.readLongAddr( frame_pos++ );  // LQI
      pkt->rssi = mrf.readLongAddr( frame_pos++ );  // RSSI
      pkt->frame_len = len;
   }
   
   uint32_t micro_ts = micros();   
   pkt->ts_sec  = micro_ts / 1000000UL;
   pkt->ts_usec = micro_ts % 1000000UL; 
   return true;
}

void show_packet( rx_packet_t *pkt, uint8_t max_len = 127 ) {

#if 0 
   int len = pkt->frame_len;
   if ( len > max_len ) { len = max_len; }
   char *ptr = buf;
   for ( int i=0; i < len; i++ ) {
      sprintf( ptr, "%02X", pkt->frame_data[i] );
      ptr +=2;
   }
   sprintf( ptr, ",%02X,%02X (%d) @%lu.%03lu\n", pkt->lqi, pkt->rssi, len,
                    pkt->ts_sec, pkt->ts_usec/1000 ); 

   Serial.println( buf );
   Serial.flush();
   
#endif
   
   udp.beginPacket( server_ip, server_port );
   udp.write( pkt->frame_len+3 );
   udp.write( pkt->frame_data, pkt->frame_len );
   udp.write( pkt->lqi );
   udp.write( pkt->rssi );
   udp.endPacket();

   int packetSize = udp.parsePacket();
   if( packetSize > 0 ) { // check whether we have an incoming message from the server
      udp.read( buf, 255 );
   }
}

void wifi_off() {
   WiFi.mode( WIFI_STA );
   WiFi.disconnect(); 
   WiFi.mode( WIFI_OFF );
   WiFi.forceSleepBegin();
   delay(1);
}

mrf_reg_t reg;
int count = 0;
int duplicate_count = 0;

void setup() { 
   Serial.begin( BAUD_RATE );
   Serial.println( F("\n\n\n\n\n\n\n") );
   Serial.flush();

   WiFi.begin( WIFI_SSID, WIFI_PASS );
   WiFi.config( 
      IPAddress(192, 168, 1, 20),
      IPAddress(192, 168, 1, 1), 
      IPAddress(255, 255, 255, 0) );
  
   Serial.println( F("Connecting to WiFi...") );
   while ( WiFi.status() != WL_CONNECTED ) {
     delay(500);
     Serial.print(".");
   }

   Serial.println( F("\n\nWiFi connected") );  
   Serial.print( F("IP address: ") );
   Serial.println( WiFi.localIP() );

   udp.begin( local_port );
  
   mrf.begin();
   mrf.reset();
   delay(10);
   
   rf_default_settings();
   rf_set_pan_id( PAN_ID );
   rf_set_channel( channel );

   // For promiscuous mode
   reg.value = mrf.readShortAddr( MRF_RXMCR );
   reg.rxmcr.NOACKRSP = 1; // disable auto ACK response
   reg.rxmcr.PANCOORD = 0; // not used as a PAN coordinator
   reg.rxmcr.COORD    = 0; // not used as a coordinator
   reg.rxmcr.ERRPKT   = 0; // accept only packets with good CRC
   reg.rxmcr.PROMI    = 1; // enable promiscuous mode: receive all packet types with good CRC
   mrf.writeShortAddr( MRF_RXMCR, reg.value );
   
   Serial.printf( "channel = %d\n", rf_get_channel() );
   Serial.printf( "short address = 0x%04X\n", rf_get_device_short_addr() );
   Serial.printf( "PAN ID = 0x%04X\n", rf_get_pan_id() );

   if ( (rf_get_channel() != channel) || (rf_get_pan_id() != PAN_ID) ) {
      Serial.println( F("MRF24J40 initialization ERROR!!!") );
      while (1) {
         Serial.println( '.' );
         delay(1000);
      }
   } else {
      Serial.println( F("MRF24J40 initialization OK!") );
   }
   rf_flush_rx_buffer();   
   attachInterrupt( digitalPinToInterrupt(INT_PIN), irq_handler, FALLING ); 

   delay(1000);
   ts = millis();
}

int silence_count = 0;

void loop() {
   if ( irq_rx_flag ) {
      if ( rf_packet_receive( &rx_packet ) ) {
        bool packet_equal = true;
        for ( int i=0; i < 5; i++ ) { // compare only the first 5 bytes
           if ( rx_packet.frame_data[i] != rx_packet_saved.frame_data[i] ) {
              packet_equal = false;
              break;
           }
        } // end-of-for
        if ( packet_equal ) {
           duplicate_count++;
        } else {
           show_packet( &rx_packet );
        }
        memcpy( &rx_packet_saved, &rx_packet, sizeof(rx_packet_t) );
        count++;
      }
      ts = millis();
   }
   if ( millis() - ts >= 2000 ) {
      ts = millis();
      if ( count > 0 ) {
         Serial.println( F("=====================================") );
         Serial.printf( "#received packets  : %d\n", count );
         Serial.printf( "#duplicate packets : %d\n\n", duplicate_count );
         silence_count = 0;
         udp.begin( local_port );
      } 
      else {
         //Serial.println( '.' );
         silence_count++;
      }
      if ( silence_count > 10 ) {
         silence_count = 0;
         if ( channel == 26 ) {
           channel = 11;
         } else {
           channel++;
         }
         rf_set_channel( channel );
         Serial.printf( "change to channel number %d\n", rf_get_channel() );
      }
      duplicate_count = count = 0;
   }
}
/////////////////////////////////////////////////////////////////////////////////////

