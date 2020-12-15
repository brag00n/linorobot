//cf https://github.com/Alictronix/Raspi-MotorHat/blob/308cfb728477b929d2333be077d82c1b49c14b5f/Raspi_MotorHAT.py#L215
//cf https://github.com/Alictronix/Raspi-MotorHat/blob/master/Raspi_MotorHAT.py
#include "Raspi_MotorHAT.h"

Raspi_MotorHAT::Raspi_MotorHAT(uint8_t address,uint8_t frequency){

    //Serial.begin(9600);
    //Log.begin (LOG_LEVEL_VERBOSE, &Serial);
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_MotorHAT::Raspi_MotorHAT\n");
#endif
    Wire.begin();
    _i2caddr = address;            // default addr on HAT
		_frequency = frequency;		       // default @1600Hz PWM freq
    
    _pwm = new Raspi_PWM_Servo_Driver (_i2caddr, _frequency); 
        
		_motors[0] = new Raspi_DCMotor (this, 1);
    _motors[1] = new Raspi_DCMotor (this, 0);
    _motors[2] = new Raspi_DCMotor (this, 2);
    _motors[3] = new Raspi_DCMotor (this, 3);
		_steppers[0] =  new Raspi_StepperMotor (this, 1, 10);
		_steppers[1] =  new Raspi_StepperMotor (this, 2, 10);
    
    _initialized= false;
    
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_MotorHAT::Raspi_MotorHAT\n");
#endif
}

bool Raspi_MotorHAT::initialize() {
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_MotorHAT::initialize()\n");
#endif

    if (!_initialized){
      if (_pwm==NULL) {
#ifdef RaspiMotorHAT_DEBUG
         Log.error("ERROR Raspi_MotorHAT::initialize() _pwm is NULL\n");
#endif
         return false;
      }
      if (_pwm->initialize()==0) {
#ifdef RaspiMotorHAT_DEBUG
         Log.error("ERROR Raspi_MotorHAT::initialize() can't initilize Raspi_PWM_Servo_Driver\n");
#endif
         return false;
      }
      if (!_pwm->setPWMFreq(_frequency)){
#ifdef RaspiMotorHAT_DEBUG
         Log.error("ERROR Raspi_MotorHAT::initialize() can't set  frequency\n");
#endif
         return false;
      }
      _initialized=true;
    }
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_MotorHAT::initialize()\n");
#endif
    return true;
}

bool Raspi_MotorHAT::isConnected() {
    if (!_initialized) return false;
    return _pwm->isConnected();
}

bool Raspi_MotorHAT::isConnected(uint8_t address) {
    if (!_initialized) return false;
    return _pwm->isConnected(address);
}

void Raspi_MotorHAT::setPin(uint8_t pin, uint8_t value){
#ifdef RaspiMotorHAT_DEBUG
   Log.trace("START Raspi_MotorHAT::setPin(pin=%d, value=%d)\n",pin,value);
#endif
   if ((pin < 0) || (pin > 15)){
#ifdef RaspiMotorHAT_DEBUG
      Log.error("ERROR Raspi_MotorHAT::setPin(pin=%d, value=%d): pin can't be <0 and pin can't be >15\n",pin,value);
#endif
      return;
   }
   if ((value != 0) && (value != 1)){
#ifdef RaspiMotorHAT_DEBUG
      Log.error("ERROR Raspi_MotorHAT::setPin(pin=%d, value=%d): value should be 0 or 1\n",pin,value);
#endif
      return;
   }
   if (value == 0)
      _pwm->setPWM(pin, 0, 4096);
   if (value == 1)
      _pwm->setPWM(pin, 4096, 0);
#ifdef RaspiMotorHAT_DEBUG
   Log.trace("END Raspi_MotorHAT::setPin(pin=%d, value=%d)\n",pin,value);
#endif      

}

Raspi_StepperMotor* Raspi_MotorHAT::getStepper(uint8_t steps, uint8_t num){
  return NULL;
}


Raspi_DCMotor* Raspi_MotorHAT::getMotor(uint8_t num){
  if (num<0 || num >3) 
     return NULL;

  return _motors[num];
}
    