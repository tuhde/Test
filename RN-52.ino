#include "RN52.h"

const byte ledPin = 13;
const byte cdPin = 3;
const byte intPin = 4;
//unsigned long stateChanged = 0;
unsigned long m;
int c = 0;

RN52 rn52( Serial1, cdPin, intPin ); // RN52 Bluetooth Module on HardwareSerial Serial1
 // Serial, GPIO9 Command/Data Pin, GPIO2 State Change Pin, PwrEn Pin, GPIO4 Factory Reset Pin, GPIO3 DFU Pin, GPIO7 Baud Rate Pin
//RN52 rn52( Serial1 ); // RN52 Bluetooth Module on HardwareSerial Serial1

void setup() {
  pinMode( ledPin, OUTPUT );
  digitalWrite( ledPin, LOW );
  pinMode( intPin, INPUT );
//  attachInterrupt( digitalPinToInterrupt( intPin ), stateChange, FALLING );
  pinMode( cdPin, OUTPUT );
  digitalWrite( cdPin, HIGH );
  Serial.begin( 115200 );
  while( !Serial );
  Serial1.begin( 115200 );
  Serial1.setTimeout( 1000 );
  rn52.begin();
}

void loop() {
  while( Serial1.available()){
    c = Serial1.read();
    Serial.write( c );
  }

  while( Serial.available()){
    c = Serial.read();
    Serial1.write( c );
  }

  if( rn52.stateChanged() > 0 ){
    digitalWrite( ledPin, HIGH );
    if( rn52.waitForCommandStart()){
      Serial.print( "State changed: " ); Serial.println( rn52.queryState(), HEX);
      while( !rn52.waitForCommandStop());
    }
    rn52._stateChangeMillis = 0;
  }

  if( millis() - rn52.stateChanged() > 1000 ){
    digitalWrite( ledPin, LOW );
    rn52._stateChangeMillis = 0;
  }

  if( millis() - m > 2000 ){
    m = millis();
    if( rn52.waitForCommandStart()){
      rn52.printState( rn52.queryState());
      while( !rn52.waitForCommandStop());
    }
    Serial.println( "Test" );
  }
}


