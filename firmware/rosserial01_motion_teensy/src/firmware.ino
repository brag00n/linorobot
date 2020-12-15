//#define __IMXRT1062__ //teensy 4.0

//#define ENCODER_DEBUG
//#define DISABLE_LOGGING

#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include "ArduinoLog.h"            //header file for Log management
char ftoaBuffer[100];

#include "ros.h"                   //header file for ros: main
#include "ros/time.h"              //header file for ros: time functionnality
#include "lino_msgs/Velocities.h"  //header file for ros: publishing velocities for odometry
#include "geometry_msgs/Twist.h"   //header file for ros: cmd_subscribing to "cmd_vel"
#include "lino_msgs/PID.h"         //header file for ros: pid server
#include "lino_msgs/Imu.h"         //header file for ros: imu message 

#include "lino_base_config.h"      //header file for robot global configuration
#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"               //header file for robot encoder driver (used for odometry)
#include "Imu.h"                   //header file for robot Imu driver
#include "Motor.h"                 //header file for robot motor driver
#include "Kinematics.h"            //header file for robot velocily tools
#include "PID.h"                   //header file for robot PID tool

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 1 //hz

#define SERIAL_TEST
//#define ROBOT_LOG_LEVEL LOG_LEVEL_PLOT
//#define ROBOT_LOG_LEVEL LOG_LEVEL_SILENT
#define ROBOT_LOG_LEVEL LOG_LEVEL_NOTICE
//#define ROBOT_LOG_LEVEL LOG_LEVEL_TRACE

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV);
IntervalTimer myTimer;
volatile float rpm[4][11]; //  average RPM stored into "rpm[<motorid>][0]", sample of values stored into "rpm[<motorid>][<1 to 10>]"
volatile int idxRpmSample[4];

Motor motor1_controller(Motor::RASPIMOTORHAT, 0, 0, 0, PWM_MIN, PWM_MAX);
Motor motor2_controller(Motor::RASPIMOTORHAT, 1, 0, 0, PWM_MIN, PWM_MAX);
Motor motor3_controller(Motor::RASPIMOTORHAT, 2, 0, 0, PWM_MIN, PWM_MAX);
Motor motor4_controller(Motor::RASPIMOTORHAT, 3, 0, 0, PWM_MIN, PWM_MAX);
/*Motor motor3_controller=motor1_controller;
Motor motor4_controller=motor2_controller;
*/
Servo steering_servo;

PID motor1_pid(PWM_MIN-PWM_MIN, PWM_MAX-PWM_MIN, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN-PWM_MIN, PWM_MAX-PWM_MIN, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN-PWM_MIN, PWM_MAX-PWM_MIN, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN-PWM_MIN, PWM_MAX-PWM_MIN, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

boolean isPidUpdated=false;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

unsigned long g_prev_command_time = 0;

static bool isImuInitialized=true;
static bool isMotorInitialized=false;
static bool isROSInitialized=false;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);

static unsigned long prev_control_time = 0;
static unsigned long prev_imu_time = 0;
static unsigned long prev_debug_time = 0;

// ROS Structure
lino_msgs::Imu raw_imu_msg;        //IMU data
lino_msgs::Velocities raw_vel_msg; //Odhometrie data

#ifndef SERIAL_TEST
  ros::NodeHandle nh;
  
  ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
  ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);
  
  ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
  ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
#endif

void PIDCallback(const lino_msgs::PID& pid){
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
    isPidUpdated=true;
    pid_p=pid.p;
    pid_i=pid.i;
    pid_d=pid.d;
}

void commandCallback(const geometry_msgs::Twist& cmd_msg){
    char s0[100],s1[100],s2[100],s3[100];
    Log.notice("[linobase1] Motor> Recieved cmd_msg info from cmd_vel ROS topic (linear.x=%s, linear.y=%s, linear.z=%s)\n", Log.ftoa(s0,cmd_msg.linear.x), Log.ftoa(s1,cmd_msg.linear.y), Log.ftoa(s2,cmd_msg.linear.z));
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored

/*    g_req_linear_vel_x = cmd_msg.linear.x*MAX_RPM;       
    g_req_linear_vel_y = cmd_msg.linear.y*MAX_RPM;
    g_req_angular_vel_z = cmd_msg.angular.z*MAX_RPM;

    g_req_linear_vel_x = (cmd_msg.linear.x)*(MAX_RPM/60)*(2* 3.1415926535897932384626433832795 * WHEEL_DIAMETER/2);       
    g_req_linear_vel_y = (cmd_msg.linear.y)*(MAX_RPM/60)*(2* 3.1415926535897932384626433832795 * WHEEL_DIAMETER/2);
    g_req_angular_vel_z = (cmd_msg.angular.z)*(MAX_RPM/60)*(2* 3.1415926535897932384626433832795 * WHEEL_DIAMETER/2);*/

    g_req_linear_vel_x = cmd_msg.linear.x;       
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;
    
    g_prev_command_time = millis();
}

float steer(float steering_angle){
    //steering function for ACKERMANN base
    float servo_steering_angle;

    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    servo_steering_angle = mapFloat(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);

    steering_servo.write(servo_steering_angle);

    return steering_angle;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long previous_ticks[4];
        
void moveBase(){
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
      //get the required rpm for each motor based on required velocities, and base used
      char s0[100],s1[100],s2[100],s3[100];
      //Log.trace("[linobase1] Motor> RPM required for Robot is (linear_vel_x:%s, linear_vel_y:%s, angular_vel_z:%s)]\n", Log.ftoa(s0,g_req_linear_vel_x), Log.ftoa(s1,g_req_linear_vel_y), Log.ftoa(s2,g_req_angular_vel_z));
      //Log.plot("Motor_linear_vel_x:%s,Motor_linear_vel_y:%s,Motor_angular_vel_z:%s,", Log.ftoa(s0,g_req_linear_vel_x), Log.ftoa(s1,g_req_linear_vel_y), Log.ftoa(s2,g_req_angular_vel_z));
  
      Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
  
      //Log.trace("[linobase1] Motor> RPM Calculated is for motors (RPMCalcMotor1:%s, RPMCalcMotor2:%s, RPMCalcMotor3:%s, RPMCalcMotor4:%s)\n",Log.ftoa(s0,req_rpm.motor1),Log.ftoa(s1,req_rpm.motor2),Log.ftoa(s2,req_rpm.motor3),Log.ftoa(s3,req_rpm.motor4));
      Log.plot("Motor1_RPMReq:%s,Motor2_RPMReq:%s,Motor3_RPMReq:%s,Motor4_RPMReq:%s,",Log.ftoa(s0,req_rpm.motor1),Log.ftoa(s1,req_rpm.motor2),Log.ftoa(s2,req_rpm.motor3),Log.ftoa(s3,req_rpm.motor4));
  
      //get the current speed of each motor
      double current_rpm1 = rpm[0][0];//motor1_encoder.getRPM();
      double current_rpm2 = rpm[1][0];//motor2_encoder.getRPM();
      double current_rpm3 = rpm[2][0];//motor3_encoder.getRPM();
      double current_rpm4 = rpm[3][0];//motor4_encoder.getRPM();
  
      Log.trace("[linobase1] Motor1> Ticks read from encoder (TickReadMotor1:%d, TickReadMotor2:%d, TickReadMotor3:%d, TickReadMotor4:%d)\n",motor1_encoder.getTick(),motor2_encoder.getTick(),motor3_encoder.getTick(),motor4_encoder.getTick());
      //Log.plot("Motor1_TickRead:%d,Motor2_TickRead:%d,Motor3_TickRead:%d,Motor4_TickRead:%d",motor2_encoder.getTick(),motor1_encoder.getTick(),motor3_encoder.getTick(),motor4_encoder.getTick());
      
      //Log.trace("[linobase1] Motor> RPM read from encoder (Motor1_RPMRead:%s, Motor2_RPMRead:%s, Motor3_RPMRead:%s, Motor4_RPMRead:%s\n",Log.ftoa(s0,current_rpm1),Log.ftoa(s1,current_rpm2),Log.ftoa(s0,current_rpm3),Log.ftoa(s1,current_rpm4));
      //Log.plot("Motor1_RPMRead:%s,Motor2_RPMRead:%s,Motor3_RPMRead:%s,Motor4_RPMRead:%s,",Log.ftoa(s0,current_rpm1),Log.ftoa(s1,current_rpm2),Log.ftoa(s0,current_rpm3),Log.ftoa(s1,current_rpm4));
  
      //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
      //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
      //Log.trace("[linobase1] Motor> run motors with velocity to match required RPMs with PID(Motor1_PID:%s, Motor2_PID:%s, Motor3_PID:%s, Motor4_PID:%s)\n",Log.ftoa(s0,motor1_pid.compute(req_rpm.motor1, current_rpm1)),Log.ftoa(s1,motor2_pid.compute(req_rpm.motor2, current_rpm2)),Log.ftoa(s0,motor3_pid.compute(req_rpm.motor3, current_rpm3)),Log.ftoa(s1,motor4_pid.compute(req_rpm.motor4, current_rpm4)));
      //Log.plot("Motor1_PID:%s,Motor2_PID:%s,Motor3_PID:%s,Motor4_PID:%s,",Log.ftoa(s0,motor1_pid.compute(req_rpm.motor1, current_rpm1)),Log.ftoa(s1,motor2_pid.compute(req_rpm.motor2, current_rpm2)),Log.ftoa(s0,motor3_pid.compute(req_rpm.motor3, current_rpm3)),Log.ftoa(s1,motor4_pid.compute(req_rpm.motor4, current_rpm4)));
      Log.plot("Motor1_PID:%s,Motor2_PID:%s,",Log.ftoa(s0,motor1_pid.compute(req_rpm.motor1, current_rpm1)),Log.ftoa(s1,motor2_pid.compute(req_rpm.motor2, current_rpm2)));


/*      if (abs(g_req_linear_vel_x)>0){
        motor1_controller.spin(120);
        motor2_controller.spin(120);
      }else{
        motor1_controller.spin(0);
        motor2_controller.spin(0);        
      }*/

      long pidSpin[4];
      pidSpin[0]=round(motor1_pid.compute(req_rpm.motor1, current_rpm1)+PWM_MIN); //if (pidSpin[0]<=PWM_MIN) pidSpin[0]=0;
      pidSpin[1]=round(motor2_pid.compute(req_rpm.motor2, current_rpm2)+PWM_MIN); //if (pidSpin[1]<=PWM_MIN) pidSpin[1]=0;
      pidSpin[2]=round(motor3_pid.compute(req_rpm.motor3, current_rpm3)+PWM_MIN); //if (pidSpin[2]<=PWM_MIN) pidSpin[2]=0; 
      pidSpin[3]=round(motor4_pid.compute(req_rpm.motor4, current_rpm4)+PWM_MIN); //if (pidSpin[3]<=PWM_MIN) pidSpin[3]=0;

      motor1_controller.spin(pidSpin[0]);
      motor2_controller.spin(pidSpin[1]);
      motor3_controller.spin(pidSpin[2]);  
      motor4_controller.spin(pidSpin[3]);

      
/*      motor1_controller.spin(round(motor1_pid.compute(req_rpm.motor1, current_rpm1)));
      motor2_controller.spin(round(motor2_pid.compute(req_rpm.motor2, current_rpm2)));
      motor3_controller.spin(round(motor3_pid.compute(req_rpm.motor3, current_rpm3)));  
      motor4_controller.spin(round(motor4_pid.compute(req_rpm.motor4, current_rpm4)));
*/
      long newTicks=motor1_encoder.getTick()-previous_ticks[0]+
                    motor2_encoder.getTick()-previous_ticks[1]+
                    motor3_encoder.getTick()-previous_ticks[2]+
                    motor4_encoder.getTick()-previous_ticks[3];
                    
  #ifndef SERIAL_TEST
      if (g_req_linear_vel_x!=0 || g_req_linear_vel_y!=0 || g_req_angular_vel_z!=0 || newTicks>0 || isPidUpdated){
        char log_msg[1024];
        char s4[100],s5[100],s6[100],s7[100],s8[100];
        nh.loginfo("[linobase1]    ==== START Debug slot");
        sprintf(log_msg,"[linobase1] ROS> RPM required for Robot is (linear_vel_x:%s, linear_vel_y:%s, linear_vel_z:%s)]", Log.ftoa(s0,g_req_linear_vel_x), Log.ftoa(s1,g_req_linear_vel_y), Log.ftoa(s2,g_req_angular_vel_z)); nh.loginfo(log_msg);
        sprintf(log_msg,"[linobase1] Motor> RPM Calculated is for motors (RPMCalcMotor1:%s, RPMCalcMotor2:%s, RPMCalcMotor3:%s, RPMCalcMotor4:%s)",Log.ftoa(s0,req_rpm.motor1),Log.ftoa(s1,req_rpm.motor2),Log.ftoa(s2,req_rpm.motor3),Log.ftoa(s3,req_rpm.motor4)); nh.loginfo(log_msg);
        sprintf(log_msg,"[linobase1] Motor2> News Ticks read from encoder TickReadMotor1:%s (DeltaTickReadMotor1:%s, DeltaTickReadMotor2:%s, DeltaTickReadMotor3:%s, DeltaTickReadMotor4:%s)",Log.ftoa(s4,motor1_encoder.getTick()),Log.ftoa(s0,motor1_encoder.getTick()-previous_ticks[0]),Log.ftoa(s1,motor2_encoder.getTick()-previous_ticks[1]),Log.ftoa(s2,motor3_encoder.getTick()-previous_ticks[2]),Log.ftoa(s3,motor4_encoder.getTick()-previous_ticks[3])); nh.loginfo(log_msg);
        sprintf(log_msg,"[linobase1] Motor> RPM read from encoder (Motor1_RPMRead:%s, Motor2_RPMRead:%s, Motor3_RPMRead:%s, Motor4_RPMRead:%s",Log.ftoa(s0,current_rpm1),Log.ftoa(s1,current_rpm2),Log.ftoa(s2,current_rpm3),Log.ftoa(s3,current_rpm4)); nh.loginfo(log_msg);
        sprintf(log_msg,"[linobase1] Motor> Run motors with velocity to match required RPMs with PID(Motor1_PID:%s, Motor2_PID:%s, Motor3_PID:%s, Motor4_PID:%s)",
          Log.ftoa(s0,round(motor1_pid.compute(req_rpm.motor1, current_rpm1))),
          Log.ftoa(s1,round(motor2_pid.compute(req_rpm.motor2, current_rpm2))),
          Log.ftoa(s2,round(motor3_pid.compute(req_rpm.motor3, current_rpm3))),
          Log.ftoa(s3,round(motor4_pid.compute(req_rpm.motor4, current_rpm4))));
          nh.loginfo(log_msg);
        if (isPidUpdated){
           sprintf(log_msg,"[linobase1] PID> PID constants was updated(P:%s, I:%s, D:%s)",Log.ftoa(s0,pid_p),Log.ftoa(s1,pid_i),Log.ftoa(s2,pid_d));nh.loginfo(log_msg);
        }
        sprintf(log_msg,"[Linobase1] IMU> (x,y,z)[lin_acc=(%s,%s,%s),ang_vel=(%s,%s,%s),mag_field=(%s,%s,%s)]", Log.ftoa(s0,raw_imu_msg.linear_acceleration.x),Log.ftoa(s1,raw_imu_msg.linear_acceleration.y),Log.ftoa(s2,raw_imu_msg.linear_acceleration.z),Log.ftoa(s3,raw_imu_msg.angular_velocity.x),Log.ftoa(s4,raw_imu_msg.angular_velocity.y),Log.ftoa(s5,raw_imu_msg.angular_velocity.z),Log.ftoa(s6,raw_imu_msg.magnetic_field.x),Log.ftoa(s7,raw_imu_msg.magnetic_field.y),Log.ftoa(s8,raw_imu_msg.magnetic_field.z)); nh.loginfo(log_msg);
        nh.loginfo("[linobase1]    ==== END Debug slot");
        previous_ticks[0]=motor1_encoder.getTick();
        previous_ticks[1]=motor2_encoder.getTick();
        previous_ticks[2]=motor3_encoder.getTick();
        previous_ticks[3]=motor4_encoder.getTick();
        isPidUpdated=false;
      }
  #else
      if (g_req_linear_vel_x!=0 || g_req_linear_vel_y!=0 || g_req_angular_vel_z!=0 || newTicks>0 || isPidUpdated){
        char log_msg[1024];
        char s4[100],s5[100],s6[100],s7[100],s8[100];
        Log.trace("[linobase1]    ==== START Debug slot\n");
        sprintf(log_msg,"[linobase1] ROS> RPM required for Robot is (linear_vel_x:%s, linear_vel_y:%s, linear_vel_z:%s)]\n", Log.ftoa(s0,g_req_linear_vel_x), Log.ftoa(s1,g_req_linear_vel_y), Log.ftoa(s2,g_req_angular_vel_z)); Log.trace(log_msg);
        sprintf(log_msg,"[linobase1] Motor> RPM Calculated is for motors (RPMCalcMotor1:%s, RPMCalcMotor2:%s, RPMCalcMotor3:%s, RPMCalcMotor4:%s)\n",Log.ftoa(s0,req_rpm.motor1),Log.ftoa(s1,req_rpm.motor2),Log.ftoa(s2,req_rpm.motor3),Log.ftoa(s3,req_rpm.motor4)); Log.trace(log_msg);
        sprintf(log_msg,"[linobase1] Motor2> News Ticks read from encoder TickReadMotor1:%s (DeltaTickReadMotor1:%s, DeltaTickReadMotor2:%s, DeltaTickReadMotor3:%s, DeltaTickReadMotor4:%s)\n",Log.ftoa(s4,motor1_encoder.getTick()),Log.ftoa(s0,motor1_encoder.getTick()-previous_ticks[0]),Log.ftoa(s1,motor2_encoder.getTick()-previous_ticks[1]),Log.ftoa(s2,motor3_encoder.getTick()-previous_ticks[2]),Log.ftoa(s3,motor4_encoder.getTick()-previous_ticks[3])); Log.trace(log_msg);
        //sprintf(log_msg,"[linobase1] Motor> RPM read from encoder (Motor1_RPMRead:%s, Motor2_RPMRead:%s, Motor3_RPMRead:%s, Motor4_RPMRead:%s)\n",Log.ftoa(s0,current_rpm1),Log.ftoa(s1,current_rpm2),Log.ftoa(s2,current_rpm3),Log.ftoa(s3,current_rpm4)); Log.trace(log_msg);
        sprintf(log_msg,"[linobase1] Motor> RPM read from encoder (Motor1_RPMRead:%s, Motor2_RPMRead:%s, Motor3_RPMRead:%s, Motor4_RPMRead:%s)\n",Log.ftoa(s0,current_rpm1),Log.ftoa(s1,current_rpm2),Log.ftoa(s2,current_rpm3),Log.ftoa(s3,current_rpm4)); Log.trace(log_msg);
        
        sprintf(log_msg,"Motor1_RPMRead:%s,Motor2_RPMRead:%s,Motor3_RPMRead:%s,Motor4_RPMRead:%s,",Log.ftoa(s0,current_rpm1),Log.ftoa(s1,current_rpm2),Log.ftoa(s2,current_rpm3),Log.ftoa(s3,current_rpm4));Log.plot(log_msg);

        sprintf(log_msg,"[linobase1] Motor> Run motors with velocity to match required RPMs with PID(Motor1_PID:%s, Motor2_PID:%s, Motor3_PID:%s, Motor4_PID:%s)\n",
          Log.ftoa(s0,pidSpin[0]),
          Log.ftoa(s1,pidSpin[1]),
          Log.ftoa(s2,pidSpin[2]),
          Log.ftoa(s3,pidSpin[3]));
          Log.trace(log_msg);
        if (isPidUpdated){
           sprintf(log_msg,"[linobase1] PID> PID constants was updated(P:%s, I:%s, D:%s)\n",Log.ftoa(s0,pid_p),Log.ftoa(s1,pid_i),Log.ftoa(s2,pid_d));Log.trace(log_msg);
        }
        sprintf(log_msg,"[Linobase1] IMU> (x,y,z)[lin_acc=(%s,%s,%s),ang_vel=(%s,%s,%s),mag_field=(%s,%s,%s)]\n", Log.ftoa(s0,raw_imu_msg.linear_acceleration.x),Log.ftoa(s1,raw_imu_msg.linear_acceleration.y),Log.ftoa(s2,raw_imu_msg.linear_acceleration.z),Log.ftoa(s3,raw_imu_msg.angular_velocity.x),Log.ftoa(s4,raw_imu_msg.angular_velocity.y),Log.ftoa(s5,raw_imu_msg.angular_velocity.z),Log.ftoa(s6,raw_imu_msg.magnetic_field.x),Log.ftoa(s7,raw_imu_msg.magnetic_field.y),Log.ftoa(s8,raw_imu_msg.magnetic_field.z)); Log.trace(log_msg);
        Log.trace("[linobase1]    ==== END Debug slot\n");
        previous_ticks[0]=motor1_encoder.getTick();
        previous_ticks[1]=motor2_encoder.getTick();
        previous_ticks[2]=motor3_encoder.getTick();
        previous_ticks[3]=motor4_encoder.getTick();
        isPidUpdated=false;
      }
      
  #endif
  
      Kinematics::velocities current_vel;
  
      if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
      {
          float current_steering_angle;
          
          current_steering_angle = steer(g_req_angular_vel_z);
          current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
      }
      else
      {
          current_vel = kinematics.getVelocities(current_rpm1, current_rpm2,current_rpm3, current_rpm4  );
      }
      
      //pass velocities to publisher object
      raw_vel_msg.linear_x = current_vel.linear_x;
      raw_vel_msg.linear_y = current_vel.linear_y;
      raw_vel_msg.angular_z = current_vel.angular_z;
  
      Log.plot("\n");
  
      //publish raw_vel_msg
  #ifndef SERIAL_TEST
      raw_vel_pub.publish(&raw_vel_msg);
  #endif
        prev_control_time = millis();
    }
}

bool initilizeMotors(){

    Log.trace("[Linobase1] Motor> initilizing Motors 1\n");
    isMotorInitialized=motor1_controller.initialize();
    if (!isMotorInitialized) {
#ifndef SERIAL_TEST
      if (isROSInitialized)
         nh.logfatal("[Linobase1] Motor> failed to initialize motor1. Check your connection.");
#else
      Log.error("[Linobase1] Motor> failed to initialize motor1. Check your connection.\n");
#endif
      return false;
    }  
    Log.trace("[Linobase1] Motor> initilizing Motors 2\n");
    isMotorInitialized=isMotorInitialized & motor2_controller.initialize();
    if (!isMotorInitialized) {
#ifndef SERIAL_TEST
      if (isROSInitialized)
         nh.logfatal("[Linobase1] Motor> failed to initialize motor2. Check your connection.");
#else
      Log.error("[Linobase1] Motor> failed to initialize motor2. Check your connection.\n");
#endif
      return false;
    }
    
    
    Log.trace("[Linobase1] Motor> initilizing Motors 3\n");
    isMotorInitialized=isMotorInitialized & motor3_controller.initialize();
    if (!isMotorInitialized) {
#ifndef SERIAL_TEST
       if (isROSInitialized)
          nh.logfatal("[Linobase1] Motor> failed to initialize motor3. Check your connection.");
#else
      Log.error("[Linobase1] Motor> failed to initialize motor3. Check your connection.\n");
#endif
      return false;
    }
    
    Log.trace("[Linobase1] Motor> initilizing Motors 4\n");
    isMotorInitialized=isMotorInitialized & motor4_controller.initialize();
    if (!isMotorInitialized) {
#ifndef SERIAL_TEST
      if (isROSInitialized)
         nh.logfatal("[Linobase1] Motor> failed to initialize motor4. Check your connection.");
#else
      Log.error("[Linobase1] Motor> failed to initialize motor4. Check your connection.\n");
#endif
      return false;
    }

    if (isMotorInitialized)
#ifndef SERIAL_TEST
       if (isROSInitialized)
          nh.loginfo("[Linobase1] Motor> Connected");
#else
       Log.notice("[Linobase1] Motor> Connected\n");
#endif

    isMotorInitialized=true;
    return isMotorInitialized;
}

void stopBase()
{
    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
      g_req_linear_vel_x = 0;
      g_req_linear_vel_y = 0;
      g_req_angular_vel_z = 0;
/*
#ifndef SERIAL_TEST
      nh.loginfo("[Linobase1] === Motor> No command received so Stopping the motors");
#else
      Log.notice("[Linobase1] === Motor> No command received so Stopping the motors\n");
#endif
*/ 
    }
}

void publishIMU()
{
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!isImuInitialized)
        {
            if(initIMU()){
#ifndef SERIAL_TEST
                nh.loginfo("[Linobase1] === IMU> Connected");
#else
                Log.notice("[Linobase1] === IMU> Connected\n");
#endif             
            }else{
 #ifndef SERIAL_TEST
               nh.logfatal("[Linobase1] IMU> failed to initialize. Check your IMU connection.");
#else
               Log.error("[Linobase1] IMU> failed to initialize. Check your IMU connection.\n");
#endif                 
               isImuInitialized=false;
            }
        }
        
        if (isImuInitialized)
        {
          //pass accelerometer data to imu object
          raw_imu_msg.linear_acceleration = readAccelerometer();
      
          //pass gyroscope data to imu object
          raw_imu_msg.angular_velocity = readGyroscope();
      
          //pass accelerometer data to imu object
          raw_imu_msg.magnetic_field = readMagnetometer();
      
          //publish raw_imu_msg
      #ifndef SERIAL_TEST 
          raw_imu_pub.publish(&raw_imu_msg);
      #else
          char s0[100],s1[100],s2[100],s3[100],s4[100],s5[100],s6[100],s7[100],s8[100];
          Log.trace("[Linobase1] IMU> Publish IMU data\n"); 
          //Log.plot( "IMU_lin_acc_x:%s,IMU_lin_acc_y:%s,IMU_lin_acc_z:%s,IMU_ang_vel_x:%s,IMU_ang_vel_y:%s,IMU_ang_vel_z:%s,IMU_mag_field_x:%s,IMU_mag_field_y:%s,IMU_mag_field_z:%s,", Log.ftoa(s0,raw_imu_msg.linear_acceleration.x),Log.ftoa(s1,raw_imu_msg.linear_acceleration.y),Log.ftoa(s2,raw_imu_msg.linear_acceleration.z),Log.ftoa(s3,raw_imu_msg.angular_velocity.x),Log.ftoa(s4,raw_imu_msg.angular_velocity.y),Log.ftoa(s5,raw_imu_msg.angular_velocity.z),Log.ftoa(s6,raw_imu_msg.magnetic_field.x),Log.ftoa(s7,raw_imu_msg.magnetic_field.y),Log.ftoa(s8,raw_imu_msg.magnetic_field.z));     
          //Log.notice( "[Linobase1] IMU> publish(x,y,z)[lin_acc=(Min:0,lin_acc_x:%s,lin_acc_y:%s,lin_accz:%s,Max:1),ang_vel=(Min:0,ang_vel_x:%s,ang_vel_y:%s,ang_vel_z:%s,Max:1),mag_field=(Min:0,mag_field_x:%s,mag_field_y:%s,mag_field_z:%s,Max:1)]\n", Log.ftoa(s0,raw_imu_msg.linear_acceleration.x),Log.ftoa(s1,raw_imu_msg.linear_acceleration.y),Log.ftoa(s2,raw_imu_msg.linear_acceleration.z),Log.ftoa(s3,raw_imu_msg.angular_velocity.x),Log.ftoa(s4,raw_imu_msg.angular_velocity.y),Log.ftoa(s5,raw_imu_msg.angular_velocity.z),Log.ftoa(s6,raw_imu_msg.magnetic_field.x),Log.ftoa(s7,raw_imu_msg.magnetic_field.y),Log.ftoa(s8,raw_imu_msg.magnetic_field.z)); 
      #endif
        }
        prev_imu_time = millis();
    }
}

void printDebug()
{
    if(false)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
          if (!isImuInitialized) {
              isImuInitialized = initIMU();
              if(!isImuInitialized){
      #ifndef SERIAL_TEST
                 nh.logfatal("[Linobase1] IMU> failed to initialize. Check your IMU connection.");
      #else
                 Log.error("[Linobase1] IMU> failed to initialize. Check your IMU connection.\n");
      #endif
              }
          }else{
              char s0[100],s1[100],s2[100],s3[100],s4[100],s5[100],s6[100],s7[100],s8[100];
              //Log.trace( "[Linobase1] IMU> read (x,y,z)[lin_acc=(:Min:0,lin_acc_x:%s,lin_acc_y:%s,lin_accz:%s,Max:1),ang_vel=(Min:0,ang_vel_x:%s,ang_vel_y:%s,ang_vel_z:%s,Max:1),mag_field=(Min:0,mag_field_x:%s,mag_field_y:%s,mag_field_z:%s,Max:1)]\n", Log.ftoa(s0,raw_imu_msg.linear_acceleration.x),Log.ftoa(s1,raw_imu_msg.linear_acceleration.y),Log.ftoa(s2,raw_imu_msg.linear_acceleration.z),Log.ftoa(s3,raw_imu_msg.angular_velocity.x),Log.ftoa(s4,raw_imu_msg.angular_velocity.y),Log.ftoa(s5,raw_imu_msg.angular_velocity.z),Log.ftoa(s6,raw_imu_msg.magnetic_field.x),Log.ftoa(s7,raw_imu_msg.magnetic_field.y),Log.ftoa(s8,raw_imu_msg.magnetic_field.z)); 
              //Log.plot( "IMU_lin_acc_x:%s,IMU_lin_acc_y:%s,IMU_lin_acc_z:%s,IMU_ang_vel_x:%s,IMU_ang_vel_y:%s,IMU_ang_vel_z:%s,IMU_mag_field_x:%s,IMU_mag_field_y:%s,IMU_mag_field_z:%s,", Log.ftoa(s0,raw_imu_msg.linear_acceleration.x),Log.ftoa(s1,raw_imu_msg.linear_acceleration.y),Log.ftoa(s2,raw_imu_msg.linear_acceleration.z),Log.ftoa(s3,raw_imu_msg.angular_velocity.x),Log.ftoa(s4,raw_imu_msg.angular_velocity.y),Log.ftoa(s5,raw_imu_msg.angular_velocity.z),Log.ftoa(s6,raw_imu_msg.magnetic_field.x),Log.ftoa(s7,raw_imu_msg.magnetic_field.y),Log.ftoa(s8,raw_imu_msg.magnetic_field.z)); 
              //Log.plot("\n");
          }
          
          if (!isMotorInitialized) {
              isMotorInitialized = initilizeMotors();
          }
          if (!isMotorInitialized)
              Log.trace("[Linobase1] Motor> Disconnected\n");
          else{
              char s0[100],s1[100],s2[100];
              //Log.trace("[Linobase1] Motor> Connected,[req_linear_vel(x,y,z)=(%s,%s,%s)]\n", Log.ftoa(s0,g_req_linear_vel_x),Log.ftoa(s1,g_req_linear_vel_y),Log.ftoa(s2,g_req_angular_vel_z));
          }
          if(!isMotorInitialized){
            isMotorInitialized = initilizeMotors();
          }
      
          //calculate RPM required for the robot
          Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
          //get the current speed of each motor
          double current_rpm1 = motor1_encoder.getRPM();
          double current_rpm2 = motor2_encoder.getRPM();
          double current_rpm3 = motor3_encoder.getRPM();
          double current_rpm4 = motor4_encoder.getRPM();
          char s0[100],s1[100],s2[100],s3[100],s4[100],s5[100],s6[100],s7[100],s8[100];
          
      #ifndef SERIAL_TEST
      /*    char log_msg[1024];
          nh.loginfo("[linobase1]    ==== START Debug slot");
          sprintf(log_msg,"[linobase1] ROS> RPM required for Robot is (linear_vel_x:%s, linear_vel_y:%s, linear_vel_z:%s)]", Log.ftoa(s0,g_req_linear_vel_x), Log.ftoa(s1,g_req_linear_vel_y), Log.ftoa(s2,g_req_angular_vel_z)); nh.loginfo(log_msg);
          sprintf(log_msg,"[linobase1] Motor> RPM Calculated is for motors (RPMCalcMotor1:%s, RPMCalcMotor2:%s, RPMCalcMotor3:%s, RPMCalcMotor4:%s)",Log.ftoa(s0,req_rpm.motor1),Log.ftoa(s1,req_rpm.motor2),Log.ftoa(s2,req_rpm.motor3),Log.ftoa(s3,req_rpm.motor4)); nh.loginfo(log_msg);
          sprintf(log_msg,"[linobase1] Motor> Ticks read from encoder (TickReadMotor1:%d, TickReadMotor2:%d, TickReadMotor3:%d, TickReadMotor4:%d)",motor2_encoder.getTick(),motor1_encoder.getTick(),motor3_encoder.getTick(),motor4_encoder.getTick()); nh.loginfo(log_msg);
          sprintf(log_msg,"[linobase1] Motor> RPM read from encoder (Motor1_RPMRead:%s, Motor2_RPMRead:%s, Motor3_RPMRead:%s, Motor4_RPMRead:%s",Log.ftoa(s0,current_rpm1),Log.ftoa(s1,current_rpm2),Log.ftoa(s0,current_rpm3),Log.ftoa(s1,current_rpm4)); nh.loginfo(log_msg);
          sprintf(log_msg,"[linobase1] Motor> Run motors with velocity to match required RPMs with PID(Motor1_PID:%s, Motor2_PID:%s, Motor3_PID:%s, Motor4_PID:%s)",Log.ftoa(s0,motor1_pid.compute(req_rpm.motor1, current_rpm1)),Log.ftoa(s1,motor2_pid.compute(req_rpm.motor2, current_rpm2)),Log.ftoa(s0,motor3_pid.compute(req_rpm.motor3, current_rpm3)),Log.ftoa(s1,motor4_pid.compute(req_rpm.motor4, current_rpm4)));nh.loginfo(log_msg);
          sprintf(log_msg,"[Linobase1] IMU> (x,y,z)[lin_acc=(%s,%s,%s),ang_vel=(%s,%s,%s),mag_field=(%s,%s,%s)]", Log.ftoa(s0,raw_imu_msg.linear_acceleration.x),Log.ftoa(s1,raw_imu_msg.linear_acceleration.y),Log.ftoa(s2,raw_imu_msg.linear_acceleration.z),Log.ftoa(s3,raw_imu_msg.angular_velocity.x),Log.ftoa(s4,raw_imu_msg.angular_velocity.y),Log.ftoa(s5,raw_imu_msg.angular_velocity.z),Log.ftoa(s6,raw_imu_msg.magnetic_field.x),Log.ftoa(s7,raw_imu_msg.magnetic_field.y),Log.ftoa(s8,raw_imu_msg.magnetic_field.z)); nh.loginfo(log_msg);
          nh.loginfo("[linobase1]    ==== END Debug slot");
      */
      #else
          Log.trace("[linobase1] ROS> RPM required for Robot is (linear_vel_x:%s, linear_vel_y:%s, linear_vel_z:%s)]\n", Log.ftoa(s0,g_req_linear_vel_x), Log.ftoa(s1,g_req_linear_vel_y), Log.ftoa(s2,g_req_angular_vel_z));
          Log.trace("[linobase1] Motor> RPM Calculated is for motors (RPMCalcMotor1:%s, RPMCalcMotor2:%s, RPMCalcMotor3:%s, RPMCalcMotor4:%s)\n",Log.ftoa(s0,req_rpm.motor1),Log.ftoa(s1,req_rpm.motor2),Log.ftoa(s2,req_rpm.motor3),Log.ftoa(s3,req_rpm.motor4));
          Log.trace("[linobase1] Motor3> Ticks read from encoder (TickReadMotor1:%d, TickReadMotor2:%d, TickReadMotor3:%d, TickReadMotor4:%d)\n",motor1_encoder.getTick(),motor2_encoder.getTick(),motor3_encoder.getTick(),motor4_encoder.getTick());
          Log.trace("[linobase1] Motor> RPM read from encoder (Motor1_RPMRead:%s, Motor2_RPMRead:%s, Motor3_RPMRead:%s, Motor4_RPMRead:%s\n",Log.ftoa(s0,current_rpm1),Log.ftoa(s1,current_rpm2),Log.ftoa(s0,current_rpm3),Log.ftoa(s1,current_rpm4));
          Log.trace("[linobase1] Motor> Run motors with velocity to match required RPMs with PID(Motor1_PID:%s, Motor2_PID:%s, Motor3_PID:%s, Motor4_PID:%s)\n",Log.ftoa(s0,motor1_pid.compute(req_rpm.motor1, current_rpm1)),Log.ftoa(s1,motor2_pid.compute(req_rpm.motor2, current_rpm2)),Log.ftoa(s0,motor3_pid.compute(req_rpm.motor3, current_rpm3)),Log.ftoa(s1,motor4_pid.compute(req_rpm.motor4, current_rpm4)));
          Log.trace("[Linobase1] IMU> (x,y,z)[lin_acc=(%s, %s, %s),ang_vel=(%s, %s, %s),mag_field=(%s, %s, %s)]\n", Log.ftoa(s0,raw_imu_msg.linear_acceleration.x),Log.ftoa(s1,raw_imu_msg.linear_acceleration.y),Log.ftoa(s2,raw_imu_msg.linear_acceleration.z),Log.ftoa(s3,raw_imu_msg.angular_velocity.x),Log.ftoa(s4,raw_imu_msg.angular_velocity.y),Log.ftoa(s5,raw_imu_msg.angular_velocity.z),Log.ftoa(s6,raw_imu_msg.magnetic_field.x),Log.ftoa(s7,raw_imu_msg.magnetic_field.y),Log.ftoa(s8,raw_imu_msg.magnetic_field.z)); 
      #endif
          //Log.plot( "Motor1_FrontLeftTicks:%d,Motor2_FrontRightTicks:%d,", motor1_encoder.getTick(), motor2_encoder.getTick());
            prev_debug_time = millis();
        }
    }
}

bool initilizeIMU(){
  isImuInitialized = initIMU();
  if(isImuInitialized){
#ifndef SERIAL_TEST
      nh.loginfo("[Linobase1] IMU> Connected");
#else
      Log.notice("[Linobase1] IMU> Connected\n");
#endif
  }else{
#ifndef SERIAL_TEST
        nh.logfatal("[Linobase1] IMU> failed to initialize. Check your IMU connection.");
#else
        Log.error("[Linobase1] IMU> failed to initialize. Check your IMU connection.\n");
#endif
  }
  return isImuInitialized;
}

//For each motor calculate average velocity (RPM) based on last 10 values
void calculateRPM(){

   // Read actual velocity
   rpm[0][idxRpmSample[0]+1]=motor1_encoder.getRPM(); //store RPM value in the RPM sample table
   rpm[1][idxRpmSample[1]+1]=motor2_encoder.getRPM(); //store RPM value in the RPM sample table
   rpm[2][idxRpmSample[2]+1]=motor3_encoder.getRPM(); //store RPM value in the RPM sample table
   rpm[3][idxRpmSample[3]+1]=motor4_encoder.getRPM(); //store RPM value in the RPM sample table

   // Increase index value
   idxRpmSample[0]=(idxRpmSample[0]+1)%10;  //increase sample table index (modulo 10)
   // Calculate average velocity
   rpm[0][0]= (rpm[0][1]+rpm[0][2]+rpm[0][3]+rpm[0][4]+rpm[0][5]+rpm[0][6]+rpm[0][7]+rpm[0][8]+rpm[0][9]+rpm[0][10])/10; //calculate average RPM value and store it

   idxRpmSample[1]=(idxRpmSample[1]+1)%10;  //increase sample table index (modulo 10)
   rpm[1][0]= (rpm[1][1]+rpm[1][2]+rpm[1][3]+rpm[1][4]+rpm[1][5]+rpm[1][6]+rpm[1][7]+rpm[1][8]+rpm[1][9]+rpm[1][10])/10; //calculate average RPM value and store it

   idxRpmSample[2]=(idxRpmSample[2]+1)%10;  //increase sample table index (modulo 10)
//   rpm[2][0]= (rpm[2][1]+rpm[2][2]+rpm[2][3]+rpm[2][4]+rpm[2][5]+rpm[2][6]+rpm[2][7]+rpm[2][8]+rpm[2][9]+rpm[2][10])/10; //calculate average RPM value and store it

   idxRpmSample[3]=(idxRpmSample[3]+1)%10;  //increase sample table index (modulo 10)
//   rpm[3][0]= (rpm[3][1]+rpm[3][2]+rpm[3][3]+rpm[3][4]+rpm[3][5]+rpm[3][6]+rpm[3][7]+rpm[3][8]+rpm[3][9]+rpm[3][10])/10; //calculate average RPM value and store it

   // For tests only 2 motors are available (other are set to 2 first one)
   rpm[2][0]=rpm[0][0];
   rpm[3][0]=rpm[1][0];
}
bool initilizeEncoder(){

  myTimer.begin(calculateRPM, 120000);  // blinkLED to run every 0.15 seconds
#ifndef SERIAL_TEST
   nh.loginfo("[Linobase1] Encoder> Connected");
#else
   Log.notice("[Linobase1] Encoder> Connected\n");
#endif
   return true;
}

bool checkROSAvailability(){
#ifndef SERIAL_TEST
  isROSInitialized=true;;
  int led = 13;
  int wait=0;
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  while (!nh.connected())
  {
      initilizeMotors();
      isROSInitialized=false;
      if (wait % 10 == 0){
         digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
         wait=0;
         delay(10000);
      }
      nh.spinOnce();
      wait=(wait+1)%10;
  }
  if (!isROSInitialized){
     nh.loginfo("[Linobase1] ROS> Re-connected");
     isROSInitialized=true;
  }
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
#else
  if (!isROSInitialized)
    Log.notice("[Linobase1] ROS> not Connected but using SERIAL for debugging\n");
#endif
  isROSInitialized=true; 
}

bool initilizeROS(){
#ifndef SERIAL_TEST
    isROSInitialized=false;
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
      
    Log.notice("[Linobase1] ROS> Connecting to ROS\n");
    int wait=0;
    while (!nh.connected())
    {
        delay(10000);
        nh.spinOnce();
    }
    
    nh.loginfo("[Linobase1] ROS> Connected");
    Log.notice("\n[Linobase1] ROS> Connected\n");
#else
    Log.notice("[Linobase1] ROS> not Connected but using SERIAL for debugging\n");
#endif
   isROSInitialized=true;
   return true;
}

bool initilizeSteeringServo(){
/*
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(90); 
*/
#ifndef SERIAL_TEST
   nh.loginfo("[Linobase1] SteeringServo> Not Connected (to be implemented)");
#else
   Log.notice("[Linobase1] SteeringServo> Not Connected (to be implemented)\n");
#endif
  return true;
}

bool initilizeCameraServo(){
/*
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(90); 
*/
#ifndef SERIAL_TEST
   nh.loginfo("[Linobase1] CameraServo> Not Connected (to be implemented)");
#else
   Log.notice("[Linobase1] CameraServo> Not Connected (to be implemented)\n");
#endif
  return true;
}

void setup()
{
   
    Log.setLevel(ROBOT_LOG_LEVEL);
    Log.notice("[Linobase1] === Start Setup\n");

    // Wait 5s blinking the led
    int led = 13;
    pinMode(led, OUTPUT);
    for (int i=4; i>0; i--){
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      if (i==2)                  // Reset Motors after 2s
         initilizeMotors();
      delay(500);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(500);               // wait for a second      
    }

    // Led is on during initialization
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    stopBase();
    
    initilizeROS();
    initilizeIMU();
    initilizeSteeringServo();
    initilizeCameraServo();
    initilizeEncoder();
    initilizeMotors();
    
    Log.notice("[Linobase1] === End Setup\n");

    // Wait 3s blinking the led
    for (int i=0; i<3; i++){
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(500);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(500);               // wait for a second      
    }
}

geometry_msgs::Twist cmd_msg_test;

void loop()
{
#ifdef SERIAL_TEST    
    //Log.notice("[Linobase1] === Start loop\n");
#endif

    readCommand();
    checkROSAvailability();
    moveBase();    //this block drives the robot based on defined rate
    //stopBase();    //this block stops the motor when no command is received
    publishIMU();  //this block publishes the IMU data based on defined rate
    //printDebug();

#ifndef SERIAL_TEST
    //call all the callbacks waiting to be called 
    nh.spinOnce();
#endif
    //Log.notice("[Linobase1] === Stop loop\n");
    //delay(10);
}

bool isMove=false;

void readCommand(){

#ifdef SERIAL_TEST
    char car;
    String com="",speed_S;
    float speed_F=50;
    if (Serial.available())
       com = Serial.readString();
    
    if (!com.equals("") && com.length()>2 && com.charAt(0)=='#'){
        isMove=false;
        car=com.charAt(1);
        switch (car){
          case 'm': //move E.g.: #m3
             isMove=true;
             if (com.length()>3){
                speed_S=com.substring(2,com.length()-1);
                speed_F=speed_S.toFloat();
                char s0[100];Log.trace("[linobase1] Motor> Set speed to %s percent\n", Log.ftoa(s0,speed_F));
             }
             break;
          case 's': //stop E.g.: #s
             isMove=false;         
             break;
          default:
             break;
        }

        if (isMove){
          g_req_linear_vel_x = (speed_F/100)*(MAX_RPM/60)*(2* 3.1415926535897932384626433832795 * WHEEL_DIAMETER/2); //= 50% de MAX_RPM (en tour/s)* 2 * pi * r  => 1.7 m/s = 6.94 km/h
          g_req_linear_vel_y = 0;
          g_req_angular_vel_z = 0; 
        }else{
          g_req_linear_vel_x = 0;
          g_req_linear_vel_y = 0;
          g_req_angular_vel_z = 0;      
        }
        char s0[100];Log.trace("[linobase1] Motor> Set speed to %s m/s\n", Log.ftoa(s0,g_req_linear_vel_x));
    }
    #endif
}