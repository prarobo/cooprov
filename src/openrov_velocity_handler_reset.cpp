#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include <stdio.h>

using namespace std;

#define STAR_REF_VEL 95
#define STAR_FORWARD_OFFSET 0
#define STAR_REVERSE_OFFSET 0
#define STAR_MAX_VEL 180
#define STAR_MIN_VEL 95

#define PORT_REF_VEL 95
#define PORT_FORWARD_OFFSET 0
#define PORT_REVERSE_OFFSET 0
#define PORT_MAX_VEL 180
#define PORT_MIN_VEL 95

#define TOP_REF_VEL 95
#define TOP_FORWARD_OFFSET 0
#define TOP_REVERSE_OFFSET 0
#define TOP_MAX_VEL 180
#define TOP_MIN_VEL 95

#define SERVO_REF_VEL 95
#define SERVO_FORWARD_OFFSET 0
#define SERVO_REVERSE_OFFSET 0
#define SERVO_MAX_VEL 180
#define SERVO_MIN_VEL 95

#define LED_REF 0
#define LED_MIN 0
#define LED_MAX 1
#define IMUR_REF 0
#define PRESR_REF 0

class vel_struct{
public:
  vel_struct();
  void setVel( int, int, int, int, int, int, int );
  int star;
  int port;
  int top;
  int servo;  
  int led;
  int imur;
  int presr;
};

vel_struct::vel_struct(){
  star = 0;
  port = 0;
  top = 0;
  servo = 0;
  led = 0;
  imur=0;
  presr=0;
}

void vel_struct :: setVel( int currStar, int currPort, int currTop, int currServo, int currLed, int currImur, int currPresr )	{
  star = currStar;
  port = currPort;
  top = currTop;
  servo = currServo;
  led = currLed;
  imur = currImur;
  presr = currPresr;
}

vel_struct curr_vel;
int prev_mode = 0;
std_msgs::Int16 vel;

ros::Subscriber direction_sub;
ros::Publisher velocity_pub;
void publish_vel( int );
void controlCallback(const std_msgs::Int8::ConstPtr&);

int main(int argc, char **argv)	{
  ros::init(argc, argv, "openrov_velocity_handler");
  ros::NodeHandle n;

  direction_sub = n.subscribe("openrov_dir", 10, controlCallback);
  velocity_pub = n.advertise<std_msgs::Int16>("openrov_vel", 10);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void publish_vel( int curr_mode )	{ 
  switch(curr_mode)	{
    case 0:
      vel.data = 0;
      break;
    case 1:
    case -1:
      vel.data = 1000+curr_vel.top;
      break;
    case 2:
    case -2:
      vel.data = 2000+curr_vel.star;
      break;
    case 3:
    case -3:
      vel.data = 3000+curr_vel.port;
      break;      
    case 4:
    case -4:
      vel.data = 4000+curr_vel.servo;
      break;
    // Pressure reset  
    case 7:
      vel.data = 7000;
      break; 
    // IMU reset  
    case 8:
      vel.data = 8000;
      break; 
    case 9:
    case -9:
      vel.data = 9000+curr_vel.led;
      break; 
  }
  //vel.data.push_back(curr_vel.star);
  //vel.data.push_back(curr_vel.port);
  //vel.data.push_back(curr_vel.top);
  //vel.data.push_back(curr_vel.servo);
  
  //cout<<vel<<endl;
  velocity_pub.publish( vel );
  //vel.data.resize(0);
}

void controlCallback(const std_msgs::Int8::ConstPtr& mode)
{
  int curr_mode = mode->data;
  switch(curr_mode)	{
    case 0:
      curr_vel.setVel( STAR_REF_VEL, PORT_REF_VEL, TOP_REF_VEL, SERVO_REF_VEL, LED_REF, IMUR_REF, PRESR_REF);
      break;
    case 1:
      if (prev_mode == curr_mode && curr_vel.top < TOP_MAX_VEL) curr_vel.top++;	
      break;
    case -1:
      if (prev_mode == curr_mode && curr_vel.top > TOP_MIN_VEL)	curr_vel.top--;
      break;
    case 2:
      if (prev_mode == curr_mode && curr_vel.star < STAR_MAX_VEL)	curr_vel.star++;
      break;
    case -2:
      if (prev_mode == curr_mode && curr_vel.star > STAR_MIN_VEL)	curr_vel.star--;
      break;
    case 3:
      if (prev_mode == curr_mode && curr_vel.port < PORT_MAX_VEL)	curr_vel.port++;
      break;
    case -3:
      if (prev_mode == curr_mode && curr_vel.port > PORT_MIN_VEL)	curr_vel.port--;
      break;      
    case 4:
      if (prev_mode == curr_mode && curr_vel.servo < SERVO_MAX_VEL)	curr_vel.servo++;
      break;
    case -4:
      if (prev_mode == curr_mode && curr_vel.servo > SERVO_MIN_VEL)	curr_vel.servo--;
      break;      
    case 7:
      curr_vel.presr=1;
      break;
    case 8:
      curr_vel.imur=1;
      break;
    case 9:
      curr_vel.led=1;
      break;
    case -9:
      curr_vel.led=0;
      break;      
  }
  prev_mode = curr_mode;
  publish_vel(curr_mode); 
}

