#include "xr1controllerol.h"
#include "actuatorcontroller.h"
#include <ros/package.h>

XR1ControllerOL * XR1_ptr;
std::vector<std::vector<double> > recorded_positions;
Affine3d itsafine;
Quaterniond itsqua;
Vector3d itsvec;


void actuator_event_callback(const ros::TimerEvent& event)
{
  ActuatorController::getInstance()->processEvents();
}



void subscribeRecordCommand(const std_msgs::Bool& msg) {


  std::vector<double> temp_data;

  XR1_ptr->getEndEffectorTransformation(XR1::LeftArm, itsafine);

  itsqua = itsafine.rotation();

  temp_data.push_back(itsqua.w());
  temp_data.push_back(itsqua.x());
  temp_data.push_back(itsqua.y());
  temp_data.push_back(itsqua.z());

  itsvec = itsafine.translation();
  temp_data.push_back(itsvec(0));
  temp_data.push_back(itsvec(1));
  temp_data.push_back(itsvec(2));

  temp_data.push_back(XR1_ptr->getElbowAngle(XR1::LeftArm));

  XR1_ptr->getEndEffectorTransformation(XR1::RightArm, itsafine);

  itsqua = itsafine.rotation();

  temp_data.push_back(itsqua.w());
  temp_data.push_back(itsqua.x());
  temp_data.push_back(itsqua.y());
  temp_data.push_back(itsqua.z());

  itsvec = itsafine.translation();
  temp_data.push_back(itsvec(0));
  temp_data.push_back(itsvec(1));
  temp_data.push_back(itsvec(2));

  temp_data.push_back(XR1_ptr->getElbowAngle(XR1::RightArm));


  recorded_positions.push_back(temp_data);

}


void subscribeWriteCommand(const std_msgs::Bool& msg) {



  std::string path = ros::package::getPath("xr1controllerol");


  std::ofstream myfile;
  myfile.open (path + "/teach.teach");


  if (recorded_positions.size()) {
    for (int i = 0; i < recorded_positions.size(); ++i)
    {
      for (int j = 0; j < recorded_positions[i].size() - 1; ++j)
      {
        myfile << recorded_positions[i][j] << ",";
      }

      myfile << recorded_positions[i][recorded_positions[i].size() - 1] << "\n";
    }
  }


  recorded_positions.clear();

  myfile.close();

}






int main(int argc, char **argv) {

  ros::init(argc, argv, "actuator_bridge");

  ros::NodeHandle nh;
  ROS_INFO("Started Actuator Bridge Ver. 3.1.0");

  ActuatorController::initController();
  ActuatorController::getInstance()->autoRecoginze();


  XR1_ptr = new XR1ControllerOL();





//  XR1_ptr->launchAllMotors(); // startSimulation()


  ros::Timer timer1 = nh.createTimer(ros::Duration(0.005), actuator_event_callback);

//  ros::Timer timer2 = nh.createTimer(ros::Duration(0.005), &XR1ControllerOL::readingCallback , XR1_ptr);

  ros::Timer timer3 = nh.createTimer(ros::Duration(0.005), &XR1ControllerOL::unleaseCallback , XR1_ptr);

//  ros::Timer timer4 = nh.createTimer(ros::Duration(0.03), &XR1ControllerOL::requestQue , XR1_ptr);

//  ros::Timer timer5 = nh.createTimer(ros::Duration(0.03), &XR1ControllerOL::MoCapCallback , XR1_ptr);


  ros::Subscriber RecordCommandSubscriber               = nh.subscribe("/XR1/RecordEFF" , 1, subscribeRecordCommand);

  ros::Subscriber WriteCommandSubscriber                = nh.subscribe("/XR1/WriteEFF" , 1, subscribeWriteCommand);




  ros::spin();

  return 0;
}