#include "xr1controllerol.h"
#include "xr1controllerolmsgulit.h"
#include "actuatorcontroller.h"
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/BodyMsgs.h"


XR1ControllerOL * XR1_ptr;

ros::Publisher *  MainBodyPositionPublisher;
ros::Publisher *  LeftArmPositionPublisher ;
ros::Publisher *  RightArmPositionPublisher;

VectorXd temp_vec5d;
VectorXd temp_vec7d;
VectorXd temp_vec3d;
xr1controllerros::HandMsgs temp_handmsgs;
xr1controllerros::ArmMsgs temp_armmsgs;
xr1controllerros::BodyMsgs temp_bodymsgs;

// Hey Gongbo you don't need this


void actuator_event_callback(const ros::TimerEvent& event)
{
  ActuatorController::getInstance()->processEvents();
}



void tiltBroadcast_callback(const ros::TimerEvent& event) {

  XR1_ptr->clearStates();

  XR1_ptr->getTargetPosition(XR1::MainBody , true);



  XR1_ptr->getTargetPosition(XR1::MainBody, temp_vec7d ,true);
  ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
  MainBodyPositionPublisher->publish(temp_bodymsgs);

  XR1_ptr->getTargetPosition(XR1::LeftArm, temp_vec7d , true);
  ConvertArmMsgs(temp_vec7d , temp_armmsgs);
  LeftArmPositionPublisher->publish(temp_armmsgs);

  XR1_ptr->getTargetPosition(XR1::RightArm, temp_vec7d, true);
  ConvertArmMsgs(temp_vec7d , temp_armmsgs);
  RightArmPositionPublisher->publish(temp_armmsgs);


}




int main(int argc, char **argv) {
  ros::init(argc, argv, "actuator_bridge");

  ros::NodeHandle nh;
  ROS_INFO("Started Controller");

  ActuatorController::initController();
  ActuatorController::getInstance()->autoRecoginze();

  temp_vec5d = VectorXd::Zero(5);
  temp_vec7d = VectorXd::Zero(7);
  temp_vec3d = VectorXd::Zero(3);

  XR1_ptr = new XR1ControllerOL();


  ros::Publisher temp_body               = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/TargetPosition" , 1);
  MainBodyPositionPublisher  = &temp_body;
  ros::Publisher temp_lear                = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetPosition" ,  1);
  LeftArmPositionPublisher  = &temp_lear;
  ros::Publisher temp_riar              = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetPosition" , 1);
  RightArmPositionPublisher  = &temp_riar;

  XR1_ptr->launchAllMotors(); // startSimulation()


  ros::Timer timer1 = nh.createTimer(ros::Duration(0.005), actuator_event_callback);

  ros::Timer timer2 = nh.createTimer(ros::Duration(0.01), &XR1ControllerOL::requestAcc , XR1_ptr);

  ros::Timer timer3 = nh.createTimer(ros::Duration(0.01), tiltBroadcast_callback);

  ros::spin();

  return 0;
}