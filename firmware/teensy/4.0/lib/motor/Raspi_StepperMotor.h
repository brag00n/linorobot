#ifndef _Raspi_StepperMotor_H_
#define _Raspi_StepperMotor_H_


#include <stdio.h>
#include "Raspi_MotorHAT.h"

class Raspi_MotorHAT;

class Raspi_StepperMotor {
    public:
        Raspi_StepperMotor(Raspi_MotorHAT* controller, uint8_t num, uint8_t steps);
        void setSpeed(uint8_t rpm);
        uint8_t oneStep(uint8_t dir, uint8_t style);
        void step(uint8_t steps, uint8_t direction, uint8_t stepstyle);
        
    private:
        Raspi_MotorHAT* controller;
};
        
#endif /* _Raspi_StepperMotor_H_ */