#include <ros/ros.h>
#include "xr1controllerpm.h"
#include "xr1define.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64.h>
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/ChainModeChange.h"
#include "xr1controllerros/BodyMsgs.h"
#include <ros/package.h>

// Global Varibles
XR1ControllerPM * XR1_ptr;
double RightElbowAngle;
double LeftElbowAngle;

tf::TransformBroadcaster * EFF_Broadcaster;
tf::TransformListener * EFF_Listener;
ros::Publisher * LeftArmPositionPublisher;
ros::Publisher * RightArmPositionPublisher;

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

  RightArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::RightArm)));
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

  LeftArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::LeftArm)));
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



}



// ----------------------------------------------------------------------------------
// ==================================================================================



// Look man you want the fk you gotta to feed me the angles
void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs& msg) {
  XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::LeftArm , XR1::ActualPosition);
}
void subscribeRightArmPosition(const xr1controllerros::ArmMsgs& msg) {
  XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::RightArm , XR1::ActualPosition);
}

void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs& msg) {
  XR1_ptr->updatingCallback(BodyMsgs2VectorXd(msg) , XR1::MainBody , XR1::ActualPosition);
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "IK_Simulator");

  ros::NodeHandle nh;


  std::string path = ros::package::getPath("xr1controllerol");

  XR1_ptr = new XR1ControllerPM(path + "/two.xr1para");

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
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), broadcastTransform);








    // Basic testing ---------------------------------------------------------------------



    VectorXd testy = VectorXd::Zero(7);
    VectorXd shity = VectorXd::Zero(7);

    testy << 0.805673833997187  ,  1.86216218006166  ,  0.795732612652139 ,   -0.172666403436580  ,  -0.410773351865951 ,   -2.11365969021934  ,  0.293479279663677;

    shity << M_PI / 2, M_PI / 2 -0.3491,M_PI / 2,0,0,-M_PI / 2,0;

    testy = testy + shity;

    XR1_ptr->updatingCallback(testy, XR1::LeftArm , XR1::ActualPosition);

    XR1_ptr->triggerCalculation();

    Affine3d transform;

    XR1_ptr->getEndEffectorTransformation(XR1::LeftArm , transform);

    Quaterniond testqua;
    testqua = transform.rotation();

    std::cout << testqua.w() <<" "<< testqua.x() <<" "<< testqua.y() <<" "<< testqua.z() << std::endl;

    std::cout << transform.translation().x() <<" "<<transform.translation().y() <<" "<<transform.translation().z() << std::endl;



    //    XR1_ptr->setControlMode(XR1::LeftArm , XR1::IKMode);
    //    Affine3d transform;

    //    transform.matrix() <<   0.172364,-0.0596322,  0.983226, 0.0195315,
    //                            0.379472,  0.925144, -0.010413,   0.11014,
    //                           -0.909005,  0.374902,   0.18209,    0.2695,
    //                                   0,         0,         0,         1;

    //    XR1_ptr->setEndEffectorPosition(XR1::LeftArm , transform , 3.0);

    // -----------------------------------------------------------------------------------













  ros::spin();


}