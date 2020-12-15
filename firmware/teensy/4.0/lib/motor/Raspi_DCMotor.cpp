#include "Raspi_DCMotor.h"

Raspi_DCMotor::Raspi_DCMotor( Raspi_MotorHAT* controller, uint8_t num){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_DCMotor::Raspi_DCMotor(num=%d)\n",num );
#endif
    _MC = controller;
    _motornum = num;

    if (_motornum == 0){
      _PWMpin = 8;
      _IN2pin = 9;
      _IN1pin = 10;
    } else if (_motornum == 1){
      _PWMpin = 13;
      _IN2pin = 12;
      _IN1pin = 11;
    } else if (_motornum == 2){
      _PWMpin = 2;
      _IN2pin = 3;
      _IN1pin = 4;
    } else if (_motornum == 3){
      _PWMpin = 7;
      _IN2pin = 6;
      _IN1pin = 5;
    } else {
      _PWMpin=_IN1pin=_IN2pin=0;
    }
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_DCMotor::Raspi_DCMotor(num=%d) _PWMpin=%d, _IN1pin=%d, _IN2pin=%d\n",num,_PWMpin,_IN1pin,_IN2pin );
#endif
}

bool Raspi_DCMotor::run(uint8_t command){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_DCMotor::run(command=%d)\n",command );
#endif
		if (_MC==NULL){
#ifdef RaspiMotorHAT_DEBUG
      Log.error("ERROR Raspi_DCMotor::run(command=%d): _MC is NULL\n",command );
#endif
      return false;
    }
      
		if (command == Raspi_DCMotor::FORWARD){
			_MC->setPin(9, 0);
			_MC->setPin(10, 1);
		} else if (command == Raspi_DCMotor::BACKWARD){
			_MC->setPin(_IN1pin, 0);
			_MC->setPin(_IN2pin, 1);
		} else if (command == Raspi_DCMotor::RELEASE){
			_MC->setPin(_IN1pin, 0);
			_MC->setPin(_IN2pin, 0);
    }
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_DCMotor::run(command=%d)\n",command );
#endif
    return true;
}

bool Raspi_DCMotor::setSpeed(uint8_t speed){

#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_DCMotor::setSpeed(speed=%d)\n",speed );
#endif

		if (speed < 0)
			speed = 0;
		if (speed > 255)
			speed = 255;
		if (!_MC->_pwm->setPWM(_PWMpin, 0, speed*16)) return false;

#ifdef RaspiMotorHAT_DEBUG    
    Log.trace("END Raspi_DCMotor::setSpeed(speed=%d)\n",speed );
#endif
    return true;
}