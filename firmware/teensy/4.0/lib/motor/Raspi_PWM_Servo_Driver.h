#ifndef _Raspi_PWM_Servo_Driver_H_
#define _Raspi_PWM_Servo_Driver_H_

#include <stdio.h>
#include <math.h> 
#include "I2Cdev.h"
#include "ArduinoLog.h"

/*
  // Registers/etc.
#define MODE1               0x00
#define MODE2               0x01
#define SUBADR1             0x02
#define SUBADR2             0x03
#define SUBADR3             0x04
#define PRESCALE            0xFE
#define LED0_ON_L           0x06
#define LED0_ON_H           0x07
#define LED0_OFF_L          0x08
#define LED0_OFF_H          0x09
#define ALL_LED_ON_L        0xFA
#define ALL_LED_ON_H        0xFB
#define ALL_LED_OFF_L       0xFC
#define ALL_LED_OFF_H       0xFD

  // Bits
#define RESTART             0x80
#define SLEEP               0x10
#define ALLCALL             0x01
#define INVRT               0x10
#define OUTDRV              0x04

#define BUFFER_OFFSET_LENGTH7   0x07
*/

//uint8_t pwmList[] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

// cf https://github.com/Alictronix/Raspi-MotorHat/blob/308cfb728477b929d2333be077d82c1b49c14b5f/Raspi_PWM_Servo_Driver.py#L11
class Raspi_PWM_Servo_Driver {
    public:
      Raspi_PWM_Servo_Driver(uint8_t address,uint8_t frequency);
      bool initialize();
      bool isConnected();
      bool isConnected(uint8_t address);
      bool setPWMFreq(uint8_t freq);
      bool setPWM(uint8_t channel, uint16_t on, uint16_t off);
      bool setAllPWM(uint16_t on, uint16_t off);
      void DEBUG_writeByte (uint8_t _address,uint16_t on, uint16_t off);
      void DEBUG_readByte(uint8_t _address,uint16_t on, uint16_t off);
    private:
      uint8_t _address;
      uint8_t _frequency;
      uint8_t _buffer[14];
    
    private:
      // cf https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
      // Constants: Registers/etc.
      static const int MODE1=         0x00;
      static const int MODE2=         0x01;
      static const int SUBADR1=       0x02;
      static const int SUBADR2=       0x03;
      static const int SUBADR3=       0x04;
      static const int PRESCALE=      0xFE;
      static const int LED0_ON_L=     0x06;
      static const int LED0_ON_H=     0x07;
      static const int LED0_OFF_L=    0x08;
      static const int LED0_OFF_H=    0x09;
      static const int ALL_LED_ON_L=  0xFA;
      static const int ALL_LED_ON_H=  0xFB;
      static const int ALL_LED_OFF_L= 0xFC;
      static const int ALL_LED_OFF_H= 0xFD;

      // Constants: Bits
      static const int ESTART=        0x80;
      static const int SLEEP=         0x10;
      static const int ALLCALL=       0x01;
      static const int INVRT=         0x10;
      static const int OUTDRV=        0x04;

      static const int BUFFER_OFFSET_LENGTH7=     0x08;
          // Bits
      enum Bits
      {
          kRestart     = 0x80,
          kSleep       = 0x10,
          kAllCall     = 0x01,
          kInvert      = 0x10,
          kOutDrive    = 0x04,
      };
      
      enum Registers
      {
          kMode1       = 0x00,
          kMode2       = 0x01,
          kSubAddress1 = 0x02,
          kSubAddress2 = 0x03,
          kSubAddress3 = 0x04,
          kPreScale    = 0xFE,
          kLed0OnL     = 0x06,
          kLed0OnH     = 0x07,
          kLed0OffL    = 0x08,
          kLed0OffH    = 0x09,
          kAllLedOnL   = 0xFA,
          kAllLedOnH   = 0xFB,
          kAllLedOffL  = 0xFC,
          kAllLedOffH  = 0xFD,
      };

};
      
#endif /* _Raspi_PWM_Servo_Driver_H_ */