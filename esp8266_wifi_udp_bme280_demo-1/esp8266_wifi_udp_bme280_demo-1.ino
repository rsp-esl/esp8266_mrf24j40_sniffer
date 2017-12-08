///////////////////////////////////////////////////////////////////////////////////////////
// Author: Rawat S.
//    Department of Electrical & Computer Engineering, 
//    Faculty of Engineering, KMUTNB, Bangkok/Thailand
// Date: 2017-12-06
// Description:
//   This sketch enables the ESP8266 hardware to read data values from the BME280 sensor
//   using I2C bus and send them over WiFi to the remote UDP server.
///////////////////////////////////////////////////////////////////////////////////////////

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "BME280.h"

#define BAUDRATE (115200)

const char* WIFI_SSID = "XXXXXXXXXXX";
const char* WIFI_PASS = "XXXXXXXXXXX";

#define BME280_ADDR     (0x76)
#define SDA_PIN         (D4)    // D4 pin (GPIO-2)
#define SCL_PIN         (D3)    // D3 pin (GPIO-0)

BME280 bme;

IPAddress server_ip(192, 168, 1, 9); // specify the IP address of the remote machine (UDP server)
int server_port = 8888;     // UDP port number of the remote machine
int local_port  = 22222;    // local UDP port

WiFiUDP udp;

#define INTERVAL_MSEC  (5000)

#define MAX_LEN (255)
char buf[ MAX_LEN+1 ];
uint32_t ts;
float humid, temp, pres;

void read_sensor() {
  temp  = bme.readTemperature();
  humid = bme.readHumidity();
  pres  = bme.readPressure() /100.0f; // hPa
}

void send_data() {
  static uint8_t cnt = 0;

  String str;
  sprintf( buf, "{\"id\":\"%s\",\"cnt\":%u,", "esp8266", cnt );
  cnt++;
  str += buf;
  dtostrf( temp, 3, 1, buf );
  str += "\"temp\":";
  str += buf;
  dtostrf( humid, 3, 1, buf );
  str += ",\"humid\":";
  str += buf;
  dtostrf( pres, 5, 1, buf );
  str += ",\"press\":";
  str += buf;
  str += "}";
  
  const uint8_t *data = (const uint8_t *)str.c_str();
  udp.beginPacket( server_ip, server_port );
  udp.write( (const uint8_t *)data, strlen((const char *)data) );
  udp.endPacket();
}

inline void process() {
  read_sensor();
  send_data();
}

void setup() {
  Serial.begin( BAUDRATE );
  Serial.println( F("\n\n\n\n") );
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
  
  if ( bme.begin( BME280_ADDR, SDA_PIN, SCL_PIN, 400000 ) ) {
     Serial.println( F("BME280 OK") );
     Serial.flush();
     delay(100);
     read_sensor();
     delay(10);
  } 
  else {
     Serial.println( F("BME280 FAILED !!!") );
     Serial.flush();
  }

  ts = millis();
  if ( ts > INTERVAL_MSEC ) {
     ts -= INTERVAL_MSEC;
  }
}

void loop() {
  if ( millis() - ts >= INTERVAL_MSEC ) {
     ts += INTERVAL_MSEC;
     process();
     delay(5);
  }

  int packetSize = udp.parsePacket();
  if( packetSize > 0 ) { // check whether we have an incoming message from the server
     udp.read( buf, MAX_LEN );
     String str( buf );
     str.trim();
     const char *data = str.c_str();
     Serial.printf( "recv: '%s' (%d bytes)\n", data, strlen(data) );
  }
  delay(1);
}
///////////////////////////////////////////////////////////////////////////////////////////

