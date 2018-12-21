#include "xr1controllerol.h"
#include "actuatorcontroller.h"


XR1ControllerOL * XR1_ptr;




void actuator_event_callback(const ros::TimerEvent& event)
{
  ActuatorController::getInstance()->processEvents();
}





int main(int argc, char **argv) {
  ros::init(argc, argv, "actuator_bridge");

  ros::NodeHandle nh;
  ROS_INFO("Started Controller");

  ActuatorController::initController();
  ActuatorController::getInstance()->autoRecoginze();


  XR1_ptr = new XR1ControllerOL();





  XR1_ptr->launchAllMotors(); // startSimulation()


  ros::Timer timer1 = nh.createTimer(ros::Duration(0.005), actuator_event_callback);

  ros::Timer timer2 = nh.createTimer(ros::Duration(0.01), &XR1ControllerOL::readingCallback , XR1_ptr);

  ros::Timer timer3 = nh.createTimer(ros::Duration(0.01), &XR1ControllerOL::unleaseCallback , XR1_ptr);

  ros::Timer timer4 = nh.createTimer(ros::Duration(0.03), &XR1ControllerOL::requestQue , XR1_ptr);

  ros::Timer timer5 = nh.createTimer(ros::Duration(0.03), &XR1ControllerOL::MoCapCallback , XR1_ptr);


  ros::spin();

  return 0;
}