#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h> 
#include <Arduino.h>
#include "Raspi_MotorHAT.h"
#include "ArduinoLog.h"

class Motor
{
    public:
        enum driver {L298, BTS7960, ESC,RASPIMOTORHAT};
        Motor(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB, int minPwm, int maxPwm);
        bool initialize();
        bool spin(int pwm);
        bool isConnected();
        bool isConnected(uint8_t address);

    private:
        Servo motor_;
        driver motor_driver_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
        int motor_minPwm_;
        int motor_maxPwm_;
};

#endif
