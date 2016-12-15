/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#define KEYCODE_RU 0x65
#define KEYCODE_RD 0x64 
#define KEYCODE_LU 0x72
#define KEYCODE_LD 0x66
#define KEYCODE_TU 0x77
#define KEYCODE_TD 0x73
#define KEYCODE_SU 0x74
#define KEYCODE_SD 0x67
#define KEYCODE_LEDU 0x79
#define KEYCODE_LEDD 0x68
#define KEYCODE_Q 0x71

class OpenrovTeleop
{
public:
  OpenrovTeleop();
  void keyLoop();
  void watchdog();

private:
  
  ros::NodeHandle nh_,ph_;
  int mode_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  ros::Publisher vel_pub_;
  void publish(int);
  boost::mutex publish_mutex_;

};

OpenrovTeleop::OpenrovTeleop():
  ph_("~"),
  mode_(0)
{
  vel_pub_ = nh_.advertise<std_msgs::Int8>("openrov_dir", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "openrov_teleop");
  OpenrovTeleop openrov_teleop;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&OpenrovTeleop::keyLoop, &openrov_teleop));
    
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&OpenrovTeleop::watchdog, &openrov_teleop));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}


void OpenrovTeleop::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
  {
    //publish(linear_, angular_);
  }
}

void OpenrovTeleop::keyLoop()
{
  char c;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the openrov.");

    
  std::cout<<"\nBoth\tStarboard\tPort\tServo\tLed\n";
  std::cout<<"w\te\tr\tt\tup\ty\t\n";
  std::cout<<"s\td\tf\tg\tdown\th\n";
  std::cout<<"q - stop\n";

  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    mode_=0;
    //ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_RU:
        ROS_DEBUG("STARBOARD UP");
	std::cout<<"Starboard up"<<std::endl;
        mode_ = 2.0;
        break;
      case KEYCODE_RD:
        ROS_DEBUG("STARBOARD DOWN");
        std::cout<<"Starboard down"<<std::endl;
	mode_ = -2.0;
        break;
      case KEYCODE_LU:
        ROS_DEBUG("PORT UP");
	std::cout<<"Port up"<<std::endl;
        mode_ = 3.0;
        break;
      case KEYCODE_LD:
        ROS_DEBUG("PORT DOWN");
	std::cout<<"Port down"<<std::endl;
        mode_ = -3.0;
        break;
      case KEYCODE_TU:
        ROS_DEBUG("TOP UP");
	std::cout<<"Top up"<<std::endl;
        mode_ = 1.0;
        break;
      case KEYCODE_TD:
        ROS_DEBUG("TOP DOWN");
	std::cout<<"Top down"<<std::endl;
        mode_ = -1.0;
        break;
      case KEYCODE_SU:
        ROS_DEBUG("SERVO UP");
	std::cout<<"Servo up"<<std::endl;
        mode_ = 4.0;
        break;
      case KEYCODE_SD:
        ROS_DEBUG("SERVO DOWN");
	std::cout<<"Servo down"<<std::endl;
        mode_ = -4.0;
        break;
      case KEYCODE_LEDU:
        ROS_DEBUG("LED UP");
	std::cout<<"Led up"<<std::endl;
        mode_ = 9.0;
        break;
      case KEYCODE_LEDD:
        ROS_DEBUG("LED DOWN");
	std::cout<<"Led down"<<std::endl;
        mode_ = -9.0;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("STOP");
	std::cout<<"Stop"<<std::endl;
        mode_ = 0;
        break;
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) { 
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish( mode_);
  }

  return;
}

void OpenrovTeleop::publish(int mode_)  
{
    std_msgs::Int8 mode;
    mode.data = mode_;
    
    vel_pub_.publish(mode);    

  return;
}



