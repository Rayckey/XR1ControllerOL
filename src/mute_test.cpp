#include <ros/ros.h>
#include "xr1controllerpm.h"
#include "xr1define.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Twist.h>
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/ChainModeChange.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerutil.h"
#include <ros/package.h>

// Global Varibles
XR1ControllerPM * XR1_ptr;
std::vector<std::vector<double> > mute_cmd;
int cmd_idx;


tf::TransformBroadcaster * EFF_Broadcaster;
tf::TransformListener * EFF_Listener;
ros::Publisher * LeftArmPositionPublisher;
ros::Publisher * RightArmPositionPublisher;
ros::Publisher * MainBodyPositionPublisher;

// IGNORE THIS PART ==============================================================
// You know what this is --------------------------------------------------------
Eigen::VectorXd ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs& msg) {

  Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

  res << msg.Shoulder_X ,
      msg.Shoulder_Y,
      msg.Elbow_Z ,
      msg.Elbow_X ,
      msg.Wrist_Z ,
      msg.Wrist_X ,
      msg.Wrist_Y ;


  return res;
}

xr1controllerros::ArmMsgs ConvertArmMsgs(Eigen::VectorXd input) {
  xr1controllerros::ArmMsgs msg;

  msg.Shoulder_X = input(0);
  msg.Shoulder_Y = input(1);
  msg.Elbow_Z = input(2);
  msg.Elbow_X = input(3);
  msg.Wrist_Z = input(4);

  msg.Wrist_X = input(5);
  msg.Wrist_Y = input(6);

  return msg;
}


Eigen::VectorXd BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs& msg) {

  Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

  res << msg.Knee  ,
      msg.Back_Z,
      msg.Back_X,
      msg.Back_Y,
      msg.Neck_Z,
      msg.Neck_X,
      msg.Head;

  return res;
}

xr1controllerros::BodyMsgs ConvertBodyMsgs(Eigen::VectorXd input) {

  xr1controllerros::BodyMsgs msg;

  msg.Knee   = input(0);
  msg.Back_Z = input(1);
  msg.Back_X = input(2);
  msg.Back_Y = input(3);
  msg.Neck_Z = input(4);
  msg.Neck_X = input(5);
  msg.Head = input(6);

  return msg;
}



void broadcastTransform(const ros::TimerEvent& event) {

  std::vector<double> temp_cmd = XR1_ptr->getNextState();

  if (temp_cmd[0] < 0.5) {
    ROS_INFO("Moving into position");
  }

  else {
    XR1_ptr->setMutePosition(mute_cmd[cmd_idx]);

    cmd_idx++;

    if (cmd_idx >= mute_cmd.size())
      cmd_idx = 0;
  }

  LeftArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::LeftArm)));

  RightArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::RightArm)));

  MainBodyPositionPublisher->publish(ConvertBodyMsgs(XR1_ptr->getTargetPosition(XR1::MainBody)));


  for (int i = XR1::OmniWheels ; i < XR1::MainBody ; i ++) {
    XR1_ptr->updatingCallback(XR1_ptr->getTargetJointVelocity(i) , i , XR1::ActualVelocity);
  }


  Eigen::Affine3d itsafine;
  tf::StampedTransform transform;


  // This function triggers almost all the computation in the library
  XR1_ptr->triggerCalculation();

  // Publish the left one
  XR1_ptr->getEndEfftorTransformation(XR1::LeftArm , itsafine);
  tf::transformEigenToTF(itsafine , transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


  // Publish the right one
  XR1_ptr->getEndEfftorTransformation(XR1::RightArm , itsafine);
  // std::cout << itsafine.matrix() << std::endl;
  tf::transformEigenToTF(itsafine , transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));



  XR1_ptr->getEndEfftorTransformation(XR1::MainBody , itsafine);
  tf::transformEigenToTF(itsafine , transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/Head"));




  XR1_ptr->getBaseTransformation(XR1::OmniWheels , itsafine);
  tf::transformEigenToTF(itsafine, transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Odom", "/Base"));



  XR1_ptr->getBaseTransformation(XR1::MainBody , itsafine);
  tf::transformEigenToTF(itsafine, transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Base", "/Back_Y"));




}

// ----------------------------------------------------------------------------------
// ==================================================================================



// Look man you want the fk you gotta to feed me the angles
void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs & msg) {
  XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::LeftArm , XR1::ActualPosition);
}
void subscribeRightArmPosition(const xr1controllerros::ArmMsgs & msg) {
  XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::RightArm , XR1::ActualPosition);
}

void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs & msg) {
  XR1_ptr->updatingCallback(BodyMsgs2VectorXd(msg) , XR1::MainBody , XR1::ActualPosition);
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "mute_test");

  ros::NodeHandle nh;


  std::string path = ros::package::getPath("xr1controllerol");

  XR1_ptr = new XR1ControllerPM(path + "/fudge.xr1para");


  mute_cmd = CSVread(path + "/Barcelona.mute");
  cmd_idx = 0;



  EFF_Broadcaster = new tf::TransformBroadcaster();
  EFF_Listener = new tf::TransformListener();

  // Feed me infos
  ros::Subscriber LeftArmPositionSubscriber   = nh.subscribe("/LeftArm/Position" ,  3 , subscribeLeftArmPosition);

  // Feed me more
  ros::Subscriber RightArmPositionSubscriber  = nh.subscribe("/RightArm/Position" ,  3 , subscribeRightArmPosition);

  ros::Subscriber MainBodyPositionSubscriber = nh.subscribe("/MainBody/Position" , 3 , subscribeMainBodyPosition);



  // cough out the target position as the result of IK;
  LeftArmPositionPublisher = new ros::Publisher();
  RightArmPositionPublisher = new ros::Publisher();
  ros::Publisher LAPP  = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetPosition", 1);
  ros::Publisher RAPP  = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetPosition", 1);
    // Mute action sometimes have main body movements
  ros::Publisher MBPP  = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/TargetPosition", 1);
  LeftArmPositionPublisher    = &LAPP;
  RightArmPositionPublisher   = &RAPP;
  MainBodyPositionPublisher   = &MBPP;



  // Draw some random stuff every three seconds or so
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), broadcastTransform);

  ros::spin();


}