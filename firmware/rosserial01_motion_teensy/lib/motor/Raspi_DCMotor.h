#ifndef _Raspi_DCMotor_H_
#define _Raspi_DCMotor_H_

#include <stdio.h>
#include "Raspi_MotorHAT.h"
#include "ArduinoLog.h"

class Raspi_MotorHAT;

class Raspi_DCMotor {
    public:
        Raspi_DCMotor(Raspi_MotorHAT* controller, uint8_t num);
        bool run(uint8_t command);
        bool setSpeed(uint8_t speed);
    public:
      static const int FORWARD=       1;
      static const int BACKWARD=      2;
      static const int BRAKE =        3;
      static const int RELEASE=       4;
      static const int SINGLE =       1;
      static const int DOUBLE =       2;
      static const int INTERLEAVE =   3;
      static const int MICROSTEP =    4;
    private:
      Raspi_MotorHAT* _MC;
      uint8_t _PWMpin;
      uint8_t _IN1pin;
      uint8_t _IN2pin;
      uint8_t _motornum;
};
        
#endif /* _Raspi_DCMotor_H_ */