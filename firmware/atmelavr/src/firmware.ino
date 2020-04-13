/* 
 * rosserial Ultrasound Example
 * 
 * This example is for the Maxbotix Ultrasound rangers.
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>

//#include "lino_base_config.h"

//#include "lino_msgs/Imu.h"  // IMU
//#include "Imu.h"            // IMU

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

// >>>>>> ULTRASOUND configuration
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5
// <<<<<< ULTRASOUND configuration

// >>>>>> IMU configuration
//#define IMU_PUBLISH_RATE 20 //hz
//lino_msgs::Imu raw_imu_msg;
//ros::Publisher raw_imu_pub("/raw_imu", &raw_imu_msg);
//static bool imu_is_initialized=false; 
// <<<<<< IMU configuration

float getRange_Ultrasound(int pPinTrig, int pPinEcho){
  pinMode(pPinTrig, OUTPUT); // Sets the trigPin as an Output
  pinMode(pPinEcho , INPUT); // Sets the echoPin as an Input
  
  // Clears the trigPin
  digitalWrite(pPinTrig, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(pPinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pPinTrig, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  volatile unsigned long duration = pulseIn(pPinEcho, HIGH);

  // Calculating the distance
  // The speed of sound is 340 m/s or 29 microseconds per cm.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  float RangeCm= duration*0.034/2;
  
  return RangeCm / 100; //return range in meter
}

void setup()
{
  nh.initNode();

  // IMU init
  //nh.advertise(raw_imu_pub);

  // TF init (Transform) coordinate frames
  broadcaster.init(nh);
  t.header.frame_id = "ultrasound";
  t.transform.translation.x = 1.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  // Coordinates for Ultrasound front Right
  t.child_frame_id = "ultrasound_FR";
  t.transform.rotation.y = 0.0;
  broadcaster.sendTransform(t);

  // Coordinates for Ultrasound front Center
  t.child_frame_id = "ultrasound_FC";
  t.transform.rotation.y = 0.5;
  broadcaster.sendTransform(t);

  // Coordinates for Ultrasound front left
  t.child_frame_id = "ultrasound_FL";
  t.transform.rotation.y = 1.0;
  broadcaster.sendTransform(t);

  // Ultrasound Init 
  nh.advertise(pub_range);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.5235987756;  //  in Radian (=30°)
  range_msg.min_range = 0.002; // min (in meter): 2cm
  range_msg.max_range = 5;     // max (in meter): 500 cm
  
  pinMode(8,OUTPUT);
  digitalWrite(8, LOW);
  pinMode(4,OUTPUT);
  digitalWrite(4, LOW);
}


unsigned long range_time;
boolean isJustConnected=true;

void loop()
{
  
  //wait until you are actually connected
  while (!nh.connected())
  {
    nh.spinOnce();
    isJustConnected=true;
  }
  if (isJustConnected){
     nh.loginfo("CONNECTED");
     isJustConnected=false;
  }

  //publish the adc value every 50 milliseconds (20hz)
  //since it takes that long for the sensor to stablize
  if ( millis() >= range_time ){

    range_msg.range = getRange_Ultrasound(4,3);
    range_msg.header.frame_id =  "ultrasound_FL";
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    //nh.logdebug("Ultrasound FL published");

    range_msg.range = getRange_Ultrasound(4,5);
    range_msg.header.frame_id =  "ultrasound_FC";
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    //nh.logdebug("Ultrasound FC published");

    range_msg.range = getRange_Ultrasound(8,9);
    range_msg.header.frame_id =  "ultrasound_FR";
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    //nh.logdebug("Ultrasound FR published");

    range_time =  millis() + 50;
  }
  
  nh.spinOnce();
}
