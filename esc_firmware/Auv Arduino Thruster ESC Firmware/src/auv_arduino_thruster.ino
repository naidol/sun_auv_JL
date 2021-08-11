#include <Arduino.h>
#include <ESC.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

#define LED_PIN (13)              // Arduino Nano Pin for the ON-BOARD LED
#define ESC_L (5)                 // Arduino Nano Pin for left thruster ESC
#define ESC_R (6)                 // Arduino Nano Pin for right thruster ESC
#define ESC_D (9)                 // Arduino Nano Pin for depth thruster ESC
#define SPEED_MIN (1000)          // Set the Minimum Speed in microseconds - 5% duty cycle
#define SPEED_MAX (2000)          // Set the Minimum Speed in microseconds - 10% duty cycle
#define SPEED_STOP (1500)         // The STOP setting in mircoseconds for bi-directional ESC - 7.5% duty cycle
#define SPEED_REV_HIGH (1100)     // Reverse HIGH = 3.5 kg F = 3.5 x 9.81 N (force)
#define SPEED_REV_MED (1270)      // Reverse MED = 1.5 kg F = 1.5 x 9.81 N (force)
#define SPEED_REV_LOW (1400)      // Reverse LOW = 0.5 kg F = 0.5 x 9.81 N (force)
#define SPEED_FWD_HIGH (1900)     // Forward HIGH = 4.5 kg F = 4.5 x 9.81 N (force)
#define SPEED_FWD_MED (1700)      // Forward MED = 1.5 kg F = 1.5 x 9.81 N (force)
#define SPEED_FWD_LOW (1600)      // Forward LOW = 0.5 kg F = 0.5 x 9.91 N (force)


ESC ESC_Left  (ESC_L, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_Right (ESC_R, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_Depth (ESC_D, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)



// CREATE A ROS NODE HANDLE
ros::NodeHandle  nh;

// CALLBACK FUNCTION EXECUTES EACH TIME THE NANO_SUB SUBSCRIBER MSG IS RECEIVED
// Vector3 (x,y,z) x: thruster(left,right,depth) | y: thruster speed | z: duration in ms
void messageCb (const geometry_msgs::Vector3 &thruster_msg) {
  switch (int(thruster_msg.x)) {
    case ESC_L:
      ESC_Left.speed(int(thruster_msg.y));
      digitalWrite(LED_PIN, HIGH-digitalRead(13));   // blink the led
      break;
    
    case ESC_R:
      ESC_Right.speed(int(thruster_msg.y));
      break;
    
    case ESC_D:
      ESC_Depth.speed(int(thruster_msg.y));
      break;

    default:
      break;
  }
}

// CREATE THE SUBSCRIBER LISTENING ON TOPIC THRUSTER_COMMAND
ros::Subscriber<geometry_msgs::Vector3> nano_sub("thruster_command", messageCb);

// CREATE A PUBLISHER THAT BROADCASTS ON THE TOPIC NANO_PUB
std_msgs::String str_msg;
ros::Publisher nano_pub("thruster_pub", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  // ARM THE THRUSTER ESC'S TO ACCEPT THRUSTER COMMANDS
  ESC_Left.arm();   
  ESC_Right.arm();  
  ESC_Depth.arm();
  delay(1000); //delay 1 sec so that thrusters can complete arming

  // INITIALISE THE ROS NODE AND START THE PUBLISHER AND SUBSCRIBER
  nh.initNode();
  nh.advertise(nano_pub);
  nh.subscribe(nano_sub);

}

void loop()
{
  str_msg.data = hello;
  nano_pub.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}

