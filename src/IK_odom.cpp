#include <ros/ros.h>
#include "xr1controllerpm.h"
#include "xr1define.h"
#include "std_msgs/Bool.h"
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "xr1controllerros/ChainModeChange.h"
#include <std_msgs/Float64.h>



// Because having a rand() function is too much for c++
double doubleRNG(){
  return (double)(rand()%100)/100.0 ;
}


// Every once in a while, command hands to go to weird places
void broadcastIKTargets( ros::Publisher & LeftElbowAngle, ros::Publisher & RightElbowAngle, tf::TransformBroadcaster& EFF_Broadcaster) {


  Eigen::Affine3d transform;
  Eigen::Vector3d translation;
  double elbow_angle;
  tf::StampedTransform target_transform;
  std_msgs::Float64 msg;



  // Making of a random transformation 
  translation << 0.3  , -0.3 , 0.3 + 0.1 ;
  //random Rotation
  transform =  AngleAxisd(-0.5*M_PI, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
  // randrom translation
  transform.translation() = translation;
  // A random elbow lift angle that is within range
  elbow_angle =  2.0 + 1.0;

  // Boardcast this  transformation
  transformEigenToTF(transform , target_transform);



  // std::cout << transform.matrix() <<std::endl;


  // Publish the left target
  msg.data = elbow_angle;
  LeftElbowAngle.publish (msg);
  EFF_Broadcaster.sendTransform(tf::StampedTransform(target_transform, ros::Time::now(), "/Odom", "/LeftEndEffectorTarget"));




  // do it again for the right arm
  translation <<  0.3 + 0.1 ,  0.3 , 0.3 + 0.1 ;
  transform =   AngleAxisd(-0.5*M_PI, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
  transform.translation() = translation;
  transformEigenToTF(transform , target_transform);

  // Flipt the angle for the right arm
  msg.data = - elbow_angle;
  RightElbowAngle.publish (msg);
  // Publish the right target
  EFF_Broadcaster.sendTransform(tf::StampedTransform(target_transform, ros::Time::now(), "/Odom", "/RightEndEffectorTarget"));

}





int main(int argc, char **argv) {

  ros::init(argc, argv, "ik_odom");

  ros::NodeHandle nh;

  ros::Publisher LeftElbowAngle = nh.advertise<std_msgs::Float64>("/LeftArm/ElbowAngle" , 1);
  ros::Publisher RightElbowAngle = nh.advertise<std_msgs::Float64>("/RightArm/ElbowAngle" , 1);
  tf::TransformBroadcaster EFF_Broadcaster;
  // Draw some random stuff every 10 seconds or so
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), boost::bind(broadcastIKTargets, LeftElbowAngle , RightElbowAngle , EFF_Broadcaster));






  ros::spin();
}