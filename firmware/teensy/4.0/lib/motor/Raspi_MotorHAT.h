//cf https://github.com/Alictronix/Raspi-MotorHat/blob/308cfb728477b929d2333be077d82c1b49c14b5f/Raspi_MotorHAT.py
#ifndef _Raspi_MotorHAT_H_
#define _Raspi_MotorHAT_H_

#include <stdio.h>
#include "Raspi_StepperMotor.h"
#include "Raspi_DCMotor.h"
#include "Raspi_PWM_Servo_Driver.h"
#include "ArduinoLog.h"

//#define RaspiMotorHAT_DEBUG

class Raspi_StepperMotor;
class Raspi_DCMotor;
class Raspi_PWM_Servo_Driver;

class Raspi_MotorHAT {
    public:
        Raspi_MotorHAT(uint8_t address,uint8_t frequency);
        bool initialize();
        bool isConnected();
        bool isConnected(uint8_t address);
        void setPin(uint8_t pin, uint8_t value);
        Raspi_StepperMotor* getStepper(uint8_t steps, uint8_t num);
        Raspi_DCMotor* getMotor(uint8_t num);

    private:
        bool _initialized;
        uint8_t _i2caddr;
        uint8_t _frequency;
        Raspi_DCMotor *_motors[4];
        Raspi_StepperMotor *_steppers[2];
    public:
        Raspi_PWM_Servo_Driver* _pwm;
 
};

#endif /* _Raspi_MotorHAT_H_ */
