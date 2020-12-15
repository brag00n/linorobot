#include "Raspi_PWM_Servo_Driver.h"

Raspi_PWM_Servo_Driver::Raspi_PWM_Servo_Driver(uint8_t address,uint8_t frequency){

#ifdef RaspiMotorHAT_DEBUG
    Log.trace ("START Raspi_PWM_Servo_Driver::Raspi_PWM_Servo_Driver(address=%X,frequency=%d)\n",address,frequency);
#endif
    _address =address;
    _frequency = frequency;
#ifdef RaspiMotorHAT_DEBUG
    Log.trace ("END Raspi_PWM_Servo_Driver::Raspi_PWM_Servo_Driver\n");
#endif
}

bool Raspi_PWM_Servo_Driver::initialize(){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_PWM_Servo_Driver::initialize\n");
#endif
    Wire.begin();
    if (!isConnected()) return false;
    
    //Set ALL PWM off
    setAllPWM(0, 0);
      
    //Reseting PCA9685 MODE1 (without SLEEP) and MODE2
    I2Cdev::writeByte(_address, MODE2,  OUTDRV);
    I2Cdev::writeByte(_address, MODE1,  ALLCALL);
    // wait for oscillator
    delay(5); 

    I2Cdev::readByte(_address, MODE1,  _buffer);
    // reset sleep (not SLEEP)
    int mode = _buffer[0] & ~SLEEP;
    I2Cdev::writeByte(_address, MODE1,  mode);

    // wait for oscillator
    delay(5);
    
    //setPWMFreq(_frequency);
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_PWM_Servo_Driver::initialize\n");
#endif
    return true; 
}


bool Raspi_PWM_Servo_Driver::isConnected(){

   return isConnected(_address);
}

bool Raspi_PWM_Servo_Driver::isConnected(uint8_t address){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_PWM_Servo_Driver::isConnected\n");
#endif
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error==0){
#ifdef RaspiMotorHAT_DEBUG
      Log.trace("Raspi_PWM_Servo_Driver: connected\n");
      Log.trace("END Raspi_PWM_Servo_Driver::isConnected\n");
#endif
      return true;
    }else{
#ifdef RaspiMotorHAT_DEBUG
      Log.warning("Raspi_PWM_Servo_Driver: NOT connected (error=%d)\n",error);
      Log.trace("END Raspi_PWM_Servo_Driver::isConnected\n");
#endif
      return false;
    }
}

bool Raspi_PWM_Servo_Driver::setPWMFreq(uint8_t freq){
    // Sets the PWM frequency

#ifdef RaspiMotorHAT_DEBUG    
    Log.trace("START Raspi_PWM_Servo_Driver::setPWMFreq(freq=%i)\n",freq);
#endif
    
    float prescaleval = 25000000.0;    // 25MHz
    prescaleval /= 4096.0 ;            // 12-bit
    prescaleval /= float(freq);
    prescaleval -= 1.0;
    
    float prescale = floor(prescaleval + 0.5);

    I2Cdev::readByte(_address, Raspi_PWM_Servo_Driver::MODE1, Raspi_PWM_Servo_Driver:: _buffer);
    uint8_t oldmode=_buffer[0];
    
    uint8_t newmode = (oldmode & 0x7F) | 0x10;             // sleep

    I2Cdev::writeByte(_address, MODE1,  newmode);    
    I2Cdev::writeByte(_address, PRESCALE,  uint8_t(floor(prescale)));
    I2Cdev::writeByte(_address, MODE1,  oldmode);
    
    delay(5);              // wait for oscillator 
    
    I2Cdev::writeByte(_address, MODE1,  oldmode| 0x80);
    
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_PWM_Servo_Driver::setPWMFreq\n");
#endif
    
    return true;
}

bool Raspi_PWM_Servo_Driver::setPWM(uint8_t channel, uint16_t on, uint16_t off){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_PWM_Servo_Driver::setPWM\n");
#endif
    
    //Log.trace("Raspi_PWM_Servo_Driver.setPWM");
    if (!I2Cdev::writeByte(_address, LED0_ON_L+4*channel,  on & 0xFF)) return false;
   
    //DEBUG_writeByte(_address, LED0_ON_L+4*channel,  on & 0xFF); 
   
    I2Cdev::writeByte(_address, LED0_ON_H+4*channel,  on >> 8);
    I2Cdev::writeByte(_address, LED0_OFF_L+4*channel,  off & 0xFF);
    I2Cdev::writeByte(_address, LED0_OFF_H+4*channel,  off >> 8);
    
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_PWM_Servo_Driver::setPWM\n");
#endif
    return true;
}

bool Raspi_PWM_Servo_Driver::setAllPWM(uint16_t on, uint16_t off){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_PWM_Servo_Driver::setAllPWM\n");
#endif
    // Sets a all PWM channels
    /*if (I2Cdev::writeByte(_address, ALL_LED_ON_L,  on & 0xFF)==0) return false;
    if (I2Cdev::writeByte(_address, ALL_LED_ON_H,  on >> 8)==0) return false;
    if (I2Cdev::writeByte(_address, ALL_LED_OFF_L,  off & 0xFF)==0) return false;
    if (I2Cdev::writeByte(_address, ALL_LED_OFF_H,  off >> 8)==0) return false;*/
    
    I2Cdev::writeByte(_address, ALL_LED_ON_L,  on & 0xFF);
    //DEBUG_writeByte(_address, ALL_LED_ON_L,  on & 0xFF); 
    
    I2Cdev::writeByte(_address, ALL_LED_ON_H,  on >> 8);
    I2Cdev::writeByte(_address, ALL_LED_OFF_L,  off & 0xFF);
    I2Cdev::writeByte(_address, ALL_LED_OFF_H,  off >> 8);

#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_PWM_Servo_Driver::setAllPWM\n");
#endif
    return true;
}