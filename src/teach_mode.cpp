#include "xr1controllerol.h"
#include "actuatorcontroller.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>

XR1ControllerOL * XR1_ptr;
std::vector<std::vector<double> > recorded_positions;
tf::TransformListener * EFF_Listener;
Affine3d itsafine;



void actuator_event_callback(const ros::TimerEvent& event)
{
	ActuatorController::getInstance()->processEvents();
}


void subscribeRecordCommand(const std_msgs::Bool& msg) {

	tf::StampedTransform transform;
	std::vector<double> temp_data;

	try {
		EFF_Listener->lookupTransform( "/Back_Y", "/LeftEndEffector",
		                              ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		return;
	}

	temp_data.push_back(transform.getRotation().w());
	temp_data.push_back(transform.getRotation().x());
	temp_data.push_back(transform.getRotation().y());
	temp_data.push_back(transform.getRotation().z());

	temp_data.push_back(transform.getOrigin().x());
	temp_data.push_back(transform.getOrigin().y());
	temp_data.push_back(transform.getOrigin().z());

	try {
		EFF_Listener->lookupTransform( "/Back_Y", "/RightEndEffector",
		                              ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		return;
	}


	temp_data.push_back(transform.getRotation().w());
	temp_data.push_back(transform.getRotation().x());
	temp_data.push_back(transform.getRotation().y());
	temp_data.push_back(transform.getRotation().z());

	temp_data.push_back(transform.getOrigin().x());
	temp_data.push_back(transform.getOrigin().y());
	temp_data.push_back(transform.getOrigin().z());


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


	myfile.close();

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "actuator_bridge");

	ros::NodeHandle nh;
	ROS_INFO("Started Controller");

	ActuatorController::initController();
	ActuatorController::getInstance()->autoRecoginze();


	EFF_Listener = new tf::TransformListener();

	XR1_ptr = new XR1ControllerOL();





	XR1_ptr->launchAllMotors(); // startSimulation()


	ros::Timer timer1 = nh.createTimer(ros::Duration(0.005), actuator_event_callback);

	ros::Timer timer2 = nh.createTimer(ros::Duration(0.01), &XR1ControllerOL::readingCallback , XR1_ptr);

	ros::Timer timer3 = nh.createTimer(ros::Duration(0.01), &XR1ControllerOL::unleaseCallback , XR1_ptr);



	ros::Subscriber RecordCommandSubscriber                 = nh.subscribe("/XR1/RecordEFF" , 1, subscribeRecordCommand);

	ros::Subscriber WriteCommandSubscriber                = nh.subscribe("/XR1/WriteEFF" , 1, subscribeWriteCommand);



	ros::spin();

	return 0;
}