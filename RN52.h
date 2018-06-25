#ifndef _RN52_H_
#define _RN52_H_

#include <Arduino.h>
#include <HardwareSerial.h>

#define BR115200 1
#define BR9600 2

class RN52 {
  public:
    static unsigned long _stateChangeMillis;

    RN52();
    RN52( HardwareSerial&, int8_t gpio9 = -1, int8_t gpio2 = -1, int8_t pwren = -1, int8_t gpio4 = -1, int8_t gpio3 = -1, int8_t gpio7 = -1 ); // Serial, GPIO9 Command/Data Pin, GPIO2 State Change Pin, PwrEn Pin, GPIO4 Factory Reset Pin, GPIO3 DFU Pin, GPIO7 Baud Rate Pin
 // Serial, GPIO9 Command/Data Pin, GPIO2 State Change Pin, PwrEn Pin, GPIO4 Factory Reset Pin, GPIO3 DFU Pin, GPIO7 Baud Rate Pin

    void begin( uint8_t br = 1 );
    void end();
    void setBitrate( uint8_t br );
    void factoryReset();
    static void stateChangeInt();
    static unsigned long stateChanged();
    uint8_t waitForCommandStart();
    uint8_t waitForCommandStop();
    uint16_t queryState();
    void printState( uint16_t );
    void setCDPin( uint8_t );
    uint8_t getCDPin();
    void setSCPin( uint8_t );
    uint8_t getSCPin();
    void setPEPin( uint8_t );
    uint8_t getPEPin();
    void setFRPin( uint8_t );
    uint8_t getFRPin();
    void setDFUPin( uint8_t );
    uint8_t getDFUPin();
    void setBRPin( uint8_t );
    uint8_t getBRPin();

  private:
    HardwareSerial& _serial;
    int8_t _gpio9 = -1;
    int8_t _gpio2 = -1;
    int8_t _pwren = -1;
    int8_t _gpio4 = -1;
    int8_t _gpio3 = -1;
    int8_t _gpio7 = -1;
};

#endif
