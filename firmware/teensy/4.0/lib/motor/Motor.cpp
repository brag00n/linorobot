#include "Motor.h"
            
Raspi_MotorHAT MotorHAT(0x6F, 160);

Motor::Motor(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB, int minPwm, int maxPwm):
    motor_driver_(motor_driver),
    pwm_pin_(pwm_pin),
    motor_pinA_(motor_pinA),
    motor_pinB_(motor_pinB),
    motor_minPwm_(minPwm),
    motor_maxPwm_(maxPwm)
{      
    //Log.setLevel(LOG_LEVEL_VERBOSE);
#ifdef RaspiMotorHAT_DEBUG    
    Log.trace("START Motor::Motor()\n");
#endif 
    initialize();
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Motor::Motor()\n");
#endif
}

bool Motor::initialize() {
    //Log.setLevel(LOG_LEVEL_VERBOSE);
    
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Motor::initialize()\n");
#endif
    switch (motor_driver_)
    {
        case L298:
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));

            break;

        case BTS7960:
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(motor_pinB_, 0);
            analogWrite(motor_pinA_, 0);

            break;

        case ESC:
            motor_.attach(motor_pinA_);

            //ensure that the motor is in neutral state during bootup
            motor_.writeMicroseconds(1500);

            break;
        case RASPIMOTORHAT:
            
            //ensure that the motor "pwm_pin" is in neutral state during bootup
            if (!MotorHAT.initialize()) return false;
            spin(0);
            MotorHAT.getMotor(pwm_pin_)->run(Raspi_DCMotor::RELEASE);
            break;
    }
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Motor::initialize()\n");
#endif
    return true;
}

bool Motor::isConnected() {
    if (motor_driver_==RASPIMOTORHAT)
      return MotorHAT.isConnected();
    else
      return true;
}

bool Motor::isConnected(uint8_t address) {
    if (motor_driver_==RASPIMOTORHAT)
      return MotorHAT.isConnected(address);
    else
      return true;
}

bool Motor::spin(int pwm)
{
    switch (motor_driver_)
    {
        case L298:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;

        case BTS7960:
            if (pwm > 0)
            {
                analogWrite(motor_pinA_, 0);
                analogWrite(motor_pinB_, abs(pwm));
            }
            else if (pwm < 0)
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, abs(pwm));
            }
            else
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, 0);
            }

            break;
        
        case ESC:
            motor_.writeMicroseconds(1500 + pwm);

            break;
            
        case RASPIMOTORHAT:
            if (!MotorHAT.isConnected() && !MotorHAT.initialize()) {
               Log.error("The motor %d is not initialized\n",pwm_pin_);
               return false;
            }

            if (abs(pwm)>motor_maxPwm_) {
               if (pwm>0)
                  pwm=motor_maxPwm_;
               else
                  pwm=-motor_maxPwm_;
            }else
               if (abs(pwm)<=motor_minPwm_) pwm=0;       // set to 0 to be sure the motor will not move
                           
            // Set motor Speed
            if (!MotorHAT.getMotor(pwm_pin_)->setSpeed(abs(pwm))) return false;
            
            // Run the motor
            if (pwm > 0){
               MotorHAT.getMotor(pwm_pin_)->run(Raspi_DCMotor::FORWARD);
            }else{
               MotorHAT.getMotor(pwm_pin_)->run(Raspi_DCMotor::BACKWARD);
            }
            break;
    }
    return true;
}
