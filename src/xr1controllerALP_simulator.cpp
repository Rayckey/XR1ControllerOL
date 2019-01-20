#include <ros/ros.h>
#include "xr1controller.h"
#include "xr1controlleralp.h"
#include "xr1define.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/ChainModeChange.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerol/AnimationMsgs.h"
#include <ros/package.h>

// Global Varibles
XR1Controller * XR1_ptr;
XR1ControllerALP * XRA_ptr;

tf::TransformBroadcaster * EFF_Broadcaster;
tf::TransformListener * EFF_Listener;
ros::Publisher * LeftArmPositionPublisher;
ros::Publisher * RightArmPositionPublisher;
ros::Publisher * MainBodyPositionPublisher;

bool animation_switch;



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



void subscribeStartAnimation(const std_msgs::Bool& msg) {
  animation_switch = true;
}


void subscribeStopAnimation(const std_msgs::Bool& msg) {
  animation_switch = false;
}


void subscribeSetAnimation(const xr1controllerol::AnimationMsgs& msg) {
  XRA_ptr->setAnimation(msg.AnimationType , msg.AnimationID);
}

//-----------------------------------------------------------------------------

void broadcastTransform(const ros::TimerEvent& event) {

  Eigen::Affine3d itsafine;
  tf::StampedTransform transform;


  // This function triggers almost all the computation in the library
  XR1_ptr->triggerCalculation();

  // Publish the left one
  XR1_ptr->getEndEffectorTransformation(XR1::LeftArm , itsafine);
  tf::transformEigenToTF(itsafine , transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


  // Publish the right one
  XR1_ptr->getEndEffectorTransformation(XR1::RightArm , itsafine);
  // std::cout << itsafine.matrix() << std::endl;
  tf::transformEigenToTF(itsafine , transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));



  XR1_ptr->getEndEffectorTransformation(XR1::MainBody , itsafine);
  tf::transformEigenToTF(itsafine , transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/Head"));



  XR1_ptr->getBaseTransformation(XR1::OmniWheels , itsafine);
  tf::transformEigenToTF(itsafine, transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Odom", "/Base"));



  XR1_ptr->getBaseTransformation(XR1::MainBody , itsafine);
  tf::transformEigenToTF(itsafine, transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Base", "/Back_Y"));



  if (animation_switch) {
    std::vector<double> temp_cmd = XRA_ptr->getNextState();

    LeftArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::LeftArm, true)));
    RightArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::RightArm, true)));
    MainBodyPositionPublisher->publish(ConvertBodyMsgs(XR1_ptr->getTargetPosition(XR1::MainBody, true)));




    // trick ourselves into thinking we have velocites
    VectorXd omni_cmd = XR1_ptr->getTargetVelocity(XR1::OmniWheels , true);

    XR1_ptr->updatingCallback(XR1::OmniWheels , XR1::ActualVelocity , omni_cmd );



  }

}



// ----------------------------------------------------------------------------------
// ==================================================================================



// Look man you want the fk you gotta to feed me the angles
void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs& msg) {
  XR1_ptr->updatingCallback( XR1::LeftArm , XR1::ActualPosition , ArmMsgs2VectorXd(msg));
}
void subscribeRightArmPosition(const xr1controllerros::ArmMsgs& msg) {
  XR1_ptr->updatingCallback( XR1::RightArm , XR1::ActualPosition ,ArmMsgs2VectorXd(msg));
}

void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs& msg) {
  XR1_ptr->updatingCallback( XR1::MainBody , XR1::ActualPosition , BodyMsgs2VectorXd(msg));
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "IK_Simulator");

  ros::NodeHandle nh;

  animation_switch = false;


  std::string path = ros::package::getPath("xr1controllerol");

  XR1_ptr = new XR1Controller(path + "/two.xr1para");

  XRA_ptr = new XR1ControllerALP(path + "/ALP" , XR1_ptr, 130 , 10 , 1 , 1 );

  EFF_Broadcaster = new tf::TransformBroadcaster();


  // Feed me infos
  ros::Subscriber LeftArmPositionSubscriber   = nh.subscribe("/LeftArm/Position" ,  3 , subscribeLeftArmPosition);

  // Feed me more
  ros::Subscriber RightArmPositionSubscriber  = nh.subscribe("/RightArm/Position" ,  3 , subscribeRightArmPosition);

  ros::Subscriber MainBodyPositionSubscriber = nh.subscribe("/MainBody/Position" , 3 , subscribeMainBodyPosition);


  ros::Subscriber StartAnimationSubscriber = nh.subscribe("/startAnimation" , 3 , subscribeStartAnimation);
  ros::Subscriber StopAnimationSubscriber = nh.subscribe("/stopAnimation" , 3 , subscribeStopAnimation);

  ros::Subscriber SetAnimationSubscriber = nh.subscribe("/setAnimation" , 3 , subscribeSetAnimation);



  // cough out the target position as the result of ALP
  ros::Publisher LAPP  = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetPosition", 1);
  ros::Publisher RAPP  = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetPosition", 1);
  ros::Publisher MBPP  = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/TargetPosition", 1);

  LeftArmPositionPublisher    = &LAPP;
  RightArmPositionPublisher   = &RAPP;
  MainBodyPositionPublisher   = &MBPP;


  // Draw some random stuff every three seconds or so
  ros::Timer timer = nh.createTimer(ros::Duration(0.005), broadcastTransform);

  ros::spin();

}