#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void publishBaseTransform(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

void publishCameraTransform(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "minoru_left_link"));
}

void publishCameraTestTransform(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera"));
}

void publishARTagTransform(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ar_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "openrov_tf_broadcaster");
  ros::NodeHandle node;
  
  ros::Rate r(10);
  
  while (ros::ok())	{
    publishBaseTransform();
    publishCameraTransform();
    //publishCameraTestTransform();
    ros::spinOnce();
    r.sleep();
  }
}