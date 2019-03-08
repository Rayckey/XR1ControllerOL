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

#include "xr1controllerros/ChainModeChange.h"

#include "xr1controllerolmsgulit.h"
#include <ros/package.h>

// Global Varibles
XR1ControllerPM * XR1_ptr;
double RightElbowAngle;
double LeftElbowAngle;

tf::TransformBroadcaster * EFF_Broadcaster;
tf::TransformListener * EFF_Listener;
ros::Publisher * LeftArmPositionPublisher;
ros::Publisher * RightArmPositionPublisher;

VectorXd temp_vec5d;
VectorXd temp_vec7d;
xr1controllerros::ArmMsgs temp_armmsgs;
xr1controllerros::BodyMsgs temp_bodymsgs;


void lookupRightEFFTarget(tf::StampedTransform & transform,  Eigen::Affine3d & itsafine) {

  try {
    EFF_Listener->lookupTransform( "/Back_Y", "/RightEndEffectorTarget",
                                   ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    return;
  }

  transformTFToEigen(transform, itsafine);
  XR1_ptr->setEndEffectorPosition(XR1::RightArm, itsafine , RightElbowAngle);

  XR1_ptr->getTargetPosition(XR1::RightArm , temp_vec7d);
  ConvertArmMsgs(temp_vec7d , temp_armmsgs);

  RightArmPositionPublisher->publish(temp_armmsgs);
}

void lookupLeftEFFTarget(tf::StampedTransform & transform,   Eigen::Affine3d & itsafine) {
  try {
    EFF_Listener->lookupTransform( "/Back_Y", "/LeftEndEffectorTarget",
                                   ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    return;
  }

  transformTFToEigen(transform, itsafine);
  XR1_ptr->setEndEffectorPosition(XR1::LeftArm, itsafine , LeftElbowAngle);

  XR1_ptr->getTargetPosition(XR1::LeftArm , temp_vec7d);
  ConvertArmMsgs(temp_vec7d , temp_armmsgs);
  LeftArmPositionPublisher->publish(temp_vec7d);
}

void subscribeLeftElbowAngle(const std_msgs::Float64 & msg) {
  LeftElbowAngle = msg.data;
}
void subscribeRightElbowAngle(const std_msgs::Float64 & msg) {
  RightElbowAngle = msg.data;
}


void subscribeLeftArmMode(const xr1controllerros::ChainModeChange& msg) {
  XR1_ptr->setControlMode(XR1::LeftArm , msg.Mode);
}
void subscribeRightArmMode(const xr1controllerros::ChainModeChange& msg) {
  XR1_ptr->setControlMode(XR1::RightArm , msg.Mode);
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


  lookupRightEFFTarget(transform, itsafine);
  lookupLeftEFFTarget(transform, itsafine);



  XR1_ptr->getBaseTransformation(XR1::OmniWheels , itsafine);
  tf::transformEigenToTF(itsafine, transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Odom", "/Base"));



  XR1_ptr->getBaseTransformation(XR1::MainBody , itsafine);
  tf::transformEigenToTF(itsafine, transform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Base", "/Back_Y"));


}



void subscribeOmniCommand(const geometry_msgs::Twist & msg) {

  Vector3d temp_cmd;

  temp_cmd << msg.angular.z, msg.linear.y, msg.linear.x;

  XR1_ptr->SetOmniWheelsVelocity(temp_cmd);

  temp_cmd = XR1_ptr->getTargetVelocity(XR1::OmniWheels);

  for (int i = XR1::OmniWheels ; i < XR1::MainBody ; i++) {
    // simulation call -------------------------------------------------------------
    XR1_ptr->updatingCallback(temp_cmd(i - XR1::OmniWheels) , i ,XR1::ActualVelocity);
  }
}
// ----------------------------------------------------------------------------------
// ==================================================================================



// Look man you want the fk you gotta to feed me the angles
void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs & msg) {

  ArmMsgs2VectorXd(msg , temp_vec7d);
  XR1_ptr->updatingCallback(temp_vec7d , XR1::LeftArm , XR1::ActualPosition);

}
void subscribeRightArmPosition(const xr1controllerros::ArmMsgs & msg) {

  ArmMsgs2VectorXd(msg , temp_vec7d);
  XR1_ptr->updatingCallback(temp_vec7d , XR1::RightArm , XR1::ActualPosition);

}

void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs & msg) {

  BodyMsgs2VectorXd(msg , temp_vec7d);
  XR1_ptr->updatingCallback(temp_vec7d , XR1::MainBody , XR1::ActualPosition);

}




int main(int argc, char **argv) {
  ros::init(argc, argv, "IK_Simulator");

  ros::NodeHandle nh;

  temp_vec5d = VectorXd::Zero(5);
  temp_vec7d = VectorXd::Zero(7);

  std::string path = ros::package::getPath("xr1controllerol");

  XR1_ptr = new XR1ControllerPM(path + "/fudge.xr1para");

  EFF_Broadcaster = new tf::TransformBroadcaster();
  EFF_Listener = new tf::TransformListener();

  // Feed me infos
  ros::Subscriber LeftArmPositionSubscriber   = nh.subscribe("/LeftArm/Position" ,  3 , subscribeLeftArmPosition);

  // Feed me more
  ros::Subscriber RightArmPositionSubscriber  = nh.subscribe("/RightArm/Position" ,  3 , subscribeRightArmPosition);

  ros::Subscriber LeftHandPositionSubscriber = nh.subscribe("/MainBody/Position" , 3 , subscribeMainBodyPosition);

  // More!!
  ros::Subscriber LeftArmModeChangeSubscriber                 = nh.subscribe("/XR1/LeftArmChainModeChange" , 1, subscribeLeftArmMode);

  //MOREEEEEEEE!
  ros::Subscriber RightArmModeChangeSubscriber                = nh.subscribe("/XR1/RightArmChainModeChange" , 1, subscribeRightArmMode);

  // MMMMMMOOOOORRRRRREEEEEEEE!
  ros::Subscriber LeftElbowSubscriber                         = nh.subscribe("LeftArm/ElbowAngle" , 1, subscribeLeftElbowAngle);

  // MMMMMMMMMOOOOOOOOOOOOOOOOAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEERRRRRRRRRRRRRRRRRRRRR!!!!!!!!!!!!!!!
  ros::Subscriber RightElbowSubscriber                        = nh.subscribe("RightArm/ElbowAngle" , 1, subscribeRightElbowAngle );


  // Driving Commands
  ros::Subscriber OmniSubscriber                              = nh.subscribe("cmd_vel" , 1 , subscribeOmniCommand);


  // cough out the target position as the result of IK;
  LeftArmPositionPublisher = new ros::Publisher();
  RightArmPositionPublisher = new ros::Publisher();
  ros::Publisher LAPP  = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetPosition", 1);
  ros::Publisher RAPP  = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetPosition", 1);
  LeftArmPositionPublisher    = &LAPP;
  RightArmPositionPublisher   = &RAPP;



  LeftElbowAngle   = 2.5;
  RightElbowAngle  = -2.5;


  // Draw some random stuff every three seconds or so
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), broadcastTransform);

  ros::spin();


}