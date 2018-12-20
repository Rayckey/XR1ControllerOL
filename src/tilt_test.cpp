#include "xr1controllerol.h"
#include "actuatorcontroller.h"
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/BodyMsgs.h"


XR1ControllerOL * XR1_ptr;

ros::Publisher *  MainBodyPositionPublisher;
ros::Publisher *  LeftArmPositionPublisher ;
ros::Publisher *  RightArmPositionPublisher;

// Hey Gongbo you don't need this


void actuator_event_callback(const ros::TimerEvent& event)
{
  ActuatorController::getInstance()->processEvents();
}



void tiltBroadcast_callback(const ros::TimerEvent& event) {

  XR1_ptr->clearStates();

  MainBodyPositionPublisher->publish(XR1_ptr->ConvertBodyMsgs(XR1_ptr->getTargetPosition(XR1::MainBody , true)));

  LeftArmPositionPublisher->publish(XR1_ptr->ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::LeftArm, true)));

  RightArmPositionPublisher->publish(XR1_ptr->ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::RightArm, true)));

}




int main(int argc, char **argv) {
  ros::init(argc, argv, "actuator_bridge");

  ros::NodeHandle nh;
  ROS_INFO("Started Controller");

  ActuatorController::initController();
  ActuatorController::getInstance()->autoRecoginze();


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