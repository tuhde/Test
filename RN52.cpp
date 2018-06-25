#include <Arduino.h>
#include <HardwareSerial.h>
#include "RN52.h"

unsigned long RN52::_stateChangeMillis = 0;

RN52::RN52():
  _serial( Serial1 )
{
}

RN52::RN52( HardwareSerial& serial, int8_t gpio9, int8_t gpio2, int8_t pwren, int8_t gpio4, int8_t gpio3, int8_t gpio7 ):
 // Serial, GPIO9 Command/Data Pin, GPIO2 State Change Pin, PwrEn Pin, GPIO4 Factory Reset Pin, GPIO3 DFU Pin, GPIO7 Bitrate Select Pin
  _serial( serial )
{
  _gpio9 = gpio9;
  _gpio2 = gpio2;
  _pwren = pwren;
  _gpio4 = gpio4;
  _gpio3 = gpio3;
  _gpio7 = gpio7;
//  RN52::stateChanged = 0;
}

void RN52::setCDPin( uint8_t gpio9 ){ _gpio9 = gpio9; };

uint8_t RN52::getCDPin(){ return _gpio9; };

void RN52::setSCPin( uint8_t gpio2 ){  _gpio2 = gpio2; };

uint8_t RN52::getSCPin(){  return _gpio2; };

void RN52::setPEPin( uint8_t pwren){  _pwren = pwren; };

uint8_t RN52::getPEPin(){  return _pwren; };

void RN52::setFRPin( uint8_t gpio4){  _gpio4 = gpio4; };

uint8_t RN52::getFRPin(){  return _gpio4; };

void RN52::setDFUPin( uint8_t gpio3 ){  _gpio3 = gpio3; };

uint8_t RN52::getDFUPin(){  return _gpio3; };

void RN52::setBRPin( uint8_t gpio7 ){  _gpio7 = gpio7; };

uint8_t RN52::getBRPin(){  return _gpio7; };

void RN52::begin( uint8_t br ){
  if( _gpio9 ){ // Command / Data Pin
    pinMode( _gpio9, OUTPUT );
    digitalWrite( _gpio9, HIGH );
  }
  if( _gpio4 ){ // Factory reset Pin
    pinMode( _gpio4, OUTPUT );
    digitalWrite( _gpio4, LOW );
  }
  if( _gpio3 ){ // DFU Mode Pin
    pinMode( _gpio3, OUTPUT );
    digitalWrite( _gpio3, LOW );
  }
  if( _gpio7 ){ // Bitrate Select Pin
    pinMode( _gpio7, OUTPUT );
    if( br == BR115200 ){
      digitalWrite( _gpio7, HIGH ); // 115200 Bit/s
      _serial.begin( 115200 );
    } else {
      digitalWrite( _gpio7, LOW ); // 9600 Bit/s
      _serial.begin( 9600 );
    }
  }
  if( _pwren ){ // Power Enable Pin
    pinMode( _pwren, OUTPUT );
    digitalWrite( _pwren, HIGH );
    delay( 200 );
  }
  if( _gpio2 ){
    pinMode( _gpio2, INPUT );
    attachInterrupt( digitalPinToInterrupt( _gpio2 ), stateChangeInt, FALLING );
  }
}

void RN52::setBitrate( uint8_t br ){
  if( _gpio7 ){ // Bitrate Select Pin
    pinMode( _gpio7, OUTPUT );
    if( br == BR115200 ){
      digitalWrite( _gpio7, HIGH ); // 115200 Bit/s
      _serial.begin( 115200 );
    } else {
      digitalWrite( _gpio7, LOW ); // 9600 Bit/s
      _serial.begin( 9600 );
    }
  }
  
}

void RN52::factoryReset(){ // untested
  if( _pwren )
    if( _gpio4 ){
      digitalWrite( _gpio4, HIGH ); // switching on while pulling gpio4 low triggers the factory reset sequence
      digitalWrite( _pwren, HIGH );
      delay( 1000 );
      digitalWrite( _gpio4, LOW );
      delay( 1000 );
      digitalWrite( _gpio4, HIGH );
      delay( 1000 );
      digitalWrite( _gpio4, LOW );
      delay( 1000 );
      digitalWrite( _gpio4, HIGH );
      delay( 1000 );
      digitalWrite( _pwren, LOW );
      digitalWrite( _gpio4, LOW );
    }
}

void RN52::stateChangeInt(){
  RN52::_stateChangeMillis = millis();
}

unsigned long RN52::stateChanged(){
  return RN52::_stateChangeMillis;
}

 // Serial, GPIO9 Command/Data Pin, GPIO2 State Change Pin, PwrEn Pin, GPIO4 Factory Reset Pin, GPIO3 DFU Pin, GPIO7 Baud Rate Pin

uint8_t RN52::waitForCommandStart(){
  String s;

  digitalWrite( _gpio9, LOW );
  s = _serial.readStringUntil( '\n' );
//  Serial.println( s );
  if( s.startsWith( "CMD" )){
//    Serial.println( "Command Mode!" );
    return 1;
  }
  return 0;
}

uint8_t RN52::waitForCommandStop(){
  String s;

  digitalWrite( _gpio9, HIGH );
  s = _serial.readStringUntil( '\n' );
//  Serial.println( s );
  if( s.startsWith( "END" )){
//    Serial.println( "Command Mode ended!" );
    return 1;
  }
  return 0;
}

uint16_t RN52::queryState(){
  char state[6];
  uint16_t s;

  _serial.print( "Q\r\n" );
  uint8_t charsRead = _serial.readBytesUntil( '\n', state, 5 );
  state[charsRead] = '\0';
  s = (uint16_t) strtol( state, 0, 16 );
//  Serial.print( "State: " ); Serial.println( s, HEX );
  return( s );
}

void RN52::printState( uint16_t state ){
  if(( state & 0x000F ) == 0 ) Serial.println( "Limbo" );
  if(( state & 0x000F ) == 1 ) Serial.println( "Connectable" );
  if(( state & 0x000F ) == 2 ) Serial.println( "Connectable and discoverable" );
  if(( state & 0x000F ) == 3 ) Serial.println( "Connected" );
  if(( state & 0x000F ) == 4 ) Serial.println( "Outgoing call established" );
  if(( state & 0x000F ) == 5 ) Serial.println( "Incoming call established" );
  if(( state & 0x000F ) == 6 ) Serial.println( "Active call" );
  if(( state & 0x000F ) == 7 ) Serial.println( "Test mode" );
  if(( state & 0x000F ) == 8 ) Serial.println( "Three-way call waiting" );
  if(( state & 0x000F ) == 9 ) Serial.println( "Three-way call on hold" );
  if(( state & 0x000F ) == 10 ) Serial.println( "Three-way call multi-call" );
  if(( state & 0x000F ) == 11 ) Serial.println( "Incoming call on hold" );
  if(( state & 0x000F ) == 12 ) Serial.println( "Active call" );
  if(( state & 0x000F ) == 13 ) Serial.println( "Audio streaming" );
  if(( state & 0x000F ) == 14 ) Serial.println( "Low battery" );
  if(( state & 0x000F ) == 15 ) Serial.println( "Unknown" );
  if( state & 0x0010 ) Serial.println( "HFP Volume change" );
  if( state & 0x0020 ) Serial.println( "HFP Microphone change" );
  if( state & 0x0100 ) Serial.println( "iAP wireless connected" );
  if( state & 0x0200 ) Serial.println( "SPP connected" );
  if( state & 0x0400 ) Serial.println( "A2DP connected" );
  if( state & 0x0800 ) Serial.println( "HFP/HSP connected" );
  if( state & 0x1000 ) Serial.println( "Caller ID Event" );
  if( state & 0x2000 ) Serial.println( "Track Change Event" );
}

