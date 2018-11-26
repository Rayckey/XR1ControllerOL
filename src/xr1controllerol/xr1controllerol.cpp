#include "xr1controllerol.h"
#include <ros/package.h>
#include <iostream>



#define PI 3.141592654

XR1ControllerOL::XR1ControllerOL() :
	hand_command_switch(true)
{

	std::string path = ros::package::getPath("xr1controllerol");

	XR1_ptr = new XR1Controller(path + "/fudge.xr1para");

	m_pController = ActuatorController::getInstance();


	JointAttributePublisher 				= nh.advertise<xr1controllerros::JointAttributeMsgs>("XR1/JointAttribute", 1);

	ActuatorLaunchedPublisher				= nh.advertise<std_msgs::Bool>("XR1/LaunchedSignal", 1);

	LaunchSubscriber                		=  nh.subscribe("/startSimulation", 1 , &XR1ControllerOL::subscribeLaunch, this);
	ShutdownSubscriber                		=  nh.subscribe("/stopSimulation", 1 , &XR1ControllerOL::subscribeShutdown, this);

	MainBodyPositionSubscriber               = nh.subscribe("/MainBody/TargetPosition" , 100 , &XR1ControllerOL::subscribeMainBodyPosition, this);
	MainBodyCurrentSubscriber                = nh.subscribe("/MainBody/TargetCurrent" ,   100 , &XR1ControllerOL::subscribeMainBodyCurrent, this);

	LeftArmPositionSubscriber                = nh.subscribe("/LeftArm/TargetPosition" ,  100 , &XR1ControllerOL::subscribeLeftArmPosition, this);
	LeftArmVelocitySubscriber                = nh.subscribe("/LeftArm/TargetVelocity" ,  100 , &XR1ControllerOL::subscribeLeftArmVelocity, this);
	LeftArmCurrentSubscriber                 = nh.subscribe("/LeftArm/TargetCurrent" ,   100 , &XR1ControllerOL::subscribeLeftArmCurrent, this);

	RightArmPositionSubscriber               = nh.subscribe("/RightArm/TargetPosition" , 100 , &XR1ControllerOL::subscribeRightArmPosition, this);
	RightArmVelocitySubscriber               = nh.subscribe("/RightArm/TargetVelocity" , 100 , &XR1ControllerOL::subscribeRightArmVelocity, this);
	RightArmCurrentSubscriber                = nh.subscribe("/RightArm/TargetCurrent" ,  100 , &XR1ControllerOL::subscribeRightArmCurrent, this);


	MainBodyModeChangeSubscriber 			= nh.subscribe("/XR1/MainBodyChainModeChange" , 10,  &XR1ControllerOL::subscribeMainBodyMode, this);
	LeftArmModeChangeSubscriber  			= nh.subscribe("/XR1/LeftArmChainModeChange" , 10,  &XR1ControllerOL::subscribeLeftArmMode, this);
	RightArmModeChangeSubscriber 			= nh.subscribe("/XR1/RightArmChainModeChange" , 10,  &XR1ControllerOL::subscribeRightArmMode, this);
	LeftHandModeChangeSubscriber 			= nh.subscribe("/XR1/LeftHandChainModeChange" , 10,  &XR1ControllerOL::subscribeLeftHandMode, this);
	RightHandModeChangeSubscriber  			= nh.subscribe("/XR1/RightHandChainModeChange" , 10,  &XR1ControllerOL::subscribeRightHandMode, this);

	LeftElbowSubscriber                     = nh.subscribe("LeftArm/ElbowAngle" , 1, &XR1ControllerOL::subscribeLeftElbowAngle , this);
	RightElbowSubscriber                    = nh.subscribe("RightArm/ElbowAngle" , 1, &XR1ControllerOL::subscribeRightElbowAngle , this);

	MainBodyPositionPublisher               = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Position" , 1);
	MainBodyCurrentPublisher                = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Current" , 1);

	LeftArmPositionPublisher                = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Position" ,  1);
	LeftArmVelocityPublisher                = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Velocity" ,  1);
	LeftArmCurrentPublisher                 = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Current" ,   1);

	RightArmPositionPublisher               = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Position" , 1);
	RightArmVelocityPublisher               = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Velocity" , 1);
	RightArmCurrentPublisher                = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Current" ,  1);

	LeftHandPositionPublisher    			= nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/Position", 1);
	RightHandPositionPublisher   			= nh.advertise<xr1controllerros::HandMsgs>("/RightHand/Position", 1);


	ROS_INFO("Recognize_Finished");

	m_pController->m_sOperationFinished->connect_member(this, &XR1ControllerOL::actuatorOperation);


	// Getting all the group ID right ------------------------------------------------
	std::vector<uint8_t> temp_vector;

	temp_vector.clear();
	for (uint8_t i = XR1::MainBody ; i < XR1::LeftArm ; i++) {
		temp_vector.push_back(i);
	}
	control_group_map[XR1::MainBody] = temp_vector;

	temp_vector.clear();
	for (uint8_t i = XR1::LeftArm ; i < XR1::RightArm ; i++) {
		temp_vector.push_back(i);
	}
	control_group_map[XR1::LeftArm] = temp_vector;


	temp_vector.clear();
	for (uint8_t i = XR1::RightArm ; i < XR1::LeftHand ; i++) {
		temp_vector.push_back(i);
	}
	control_group_map[XR1::RightArm] = temp_vector;


	temp_vector.clear();
	for (uint8_t i = XR1::LeftHand ; i < XR1::RightHand ; i++) {
		temp_vector.push_back(i);
	}
	control_group_map[XR1::LeftHand] = temp_vector;


	temp_vector.clear();
	for (uint8_t i = XR1::RightHand ; i < XR1::Actuator_Total ; i++) {
		temp_vector.push_back(i);
	}
	control_group_map[XR1::RightHand] = temp_vector;
	// ------------------------------------------------


	attribute_map[Actuator::ACTUAL_POSITION] = XR1::ActualPosition;
	attribute_map[Actuator::ACTUAL_VELOCITY] = XR1::ActualVelocity;
	attribute_map[Actuator::ACTUAL_CURRENT] = XR1::ActualCurrent;



	mode_map[XR1::PositionMode] = Actuator::Mode_Profile_Pos;
	mode_map[XR1::VelocityMode] = Actuator::Mode_Profile_Vel;
	mode_map[XR1::ForceMode]    = Actuator::Mode_Cur;



	//Update Callback
	m_pController->m_sActuatorAttrChanged->connect_member(this, &XR1ControllerOL::updatingCallback);




	// Elbow lower anlges, measured from the top, by default:
	// For left arm, the desirable range is 1.0 to 3.0
	// For right arm, the desirable ranfge is -3.0 to -1.0
	LeftElbowAngle   = 2.5;
	RightElbowAngle  = -2.5;
	ROS_INFO("OL finished");

}

XR1ControllerOL::~XR1ControllerOL()
{
	// unregister all publishers here

	JointAttributePublisher.shutdown();
	ActuatorLaunchedPublisher.shutdown();
}


void XR1ControllerOL::launchAllMotors() {

	m_pController->launchAllActuators();
	if (allActuatorHasLaunched()) {
		std_msgs::Bool stuff;
		stuff.data = true;
		ActuatorLaunchedPublisher.publish(stuff);
	}

}

bool XR1ControllerOL::allActuatorHasLaunched()
{
	vector<uint8_t> idArray = m_pController->getActuatorIdArray();
	for (int i = 0; i < idArray.size(); ++i)
	{
		if (m_pController->getActuatorAttribute((uint8_t) idArray.at(i), Actuator::ACTUATOR_SWITCH) != Actuator::ACTUATOR_SWITCH_ON)
			return false;
	}
	return true;
}

void XR1ControllerOL::startSimulation() {
	launchAllMotors();
}

void XR1ControllerOL::stopAllMotors() {

	std::vector<uint8_t> IDArray = m_pController->getActuatorIdArray();

	for (int i = 0 ; i < IDArray.size() ; i++) {
		m_pController->closeActuator(IDArray.at(i));
	}

}

void XR1ControllerOL::stopSimulation() {
	stopAllMotors() ;
}


void XR1ControllerOL::setControlMode(uint8_t control_group , uint8_t option) {

	std::vector<uint8_t> temp_vector = control_group_map[control_group];


	for (uint8_t id : temp_vector)
		m_pController->activeActuatorMode(id, mode_map[option]);


}


void XR1ControllerOL::setJointPosition(uint8_t control_group , VectorXd JA) {

	std::vector<uint8_t> temp_vector = control_group_map[control_group];

	for (uint8_t id : temp_vector)
		m_pController->setPosition(id, JA(id - control_group));

}

void XR1ControllerOL::setJointVelocity(uint8_t control_group , VectorXd JV) {

	std::vector<uint8_t> temp_vector = control_group_map[control_group];

	for (uint8_t id : temp_vector)
		m_pController->setVelocity(id, JV(id - control_group));

}

void XR1ControllerOL::setJointCurrent(uint8_t control_group , VectorXd JC) {

	std::vector<uint8_t> temp_vector = control_group_map[control_group];

	for (uint8_t id : temp_vector)
		m_pController->setCurrent(id, JC(id - control_group));

}

void XR1ControllerOL::updatingCallback(uint8_t id, uint8_t attrId, double value) {

	// ROS_INFO("updatingCallback");
	// xr1controllerros::JointAttributeMsgs msg;

	// msg.JointID = id;

	// msg.AttributeID = attribute_map[attrId];

	// msg.Value = value;


	// JointAttributePublisher.publish(msg);

	if (attribute_map.find(attrId) != attribute_map.end())
		XR1_ptr->updatingCallback( id,  attribute_map[attrId],  value);
}


void XR1ControllerOL::subscribeLaunch(const std_msgs::Bool& msg) {
	launchAllMotors();
}

void XR1ControllerOL::subscribeShutdown(const std_msgs::Bool& msg) {
	stopAllMotors();
}

Eigen::VectorXd XR1ControllerOL::BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs& msg) {

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

Eigen::VectorXd XR1ControllerOL::ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs& msg) {

	Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

	res << msg.Shoulder_X ,
	    msg.Shoulder_Y,
	    msg.Elbow_Z ,
	    msg.Elbow_X ,
	    msg.Wrist_Z ,
	    msg.Wrist_Y ,
	    msg.Wrist_X ;

	return res;
}

Eigen::VectorXd XR1ControllerOL::HandsMsgs2VectorXd(const xr1controllerros::HandMsgs& msg) {

	Eigen::VectorXd res = Eigen::VectorXd::Zero(5);

	res << msg.Thumb  ,
	    msg.Index,
	    msg.Middle,
	    msg.Ring,
	    msg.Pinky;


	return res;
}


void XR1ControllerOL::subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->setJointVelocity(XR1::LeftArm , ArmMsgs2VectorXd(msg));
	setJointVelocity(XR1::LeftArm , XR1_ptr->getTargetVelocity(XR1::LeftArm));
}

void XR1ControllerOL::subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->setJointCurrent(XR1::LeftArm , ArmMsgs2VectorXd(msg));
	setJointCurrent(XR1::LeftArm , XR1_ptr->getTargetCurrent(XR1::LeftArm));
}

void XR1ControllerOL::subscribeRightArmVelocity(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->setJointVelocity(XR1::RightArm , ArmMsgs2VectorXd(msg));
	setJointVelocity(XR1::RightArm , XR1_ptr->getTargetVelocity(XR1::RightArm));
}

void XR1ControllerOL::subscribeRightArmCurrent(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->setJointCurrent(XR1::RightArm , ArmMsgs2VectorXd(msg));
	setJointCurrent(XR1::RightArm , XR1_ptr->getTargetCurrent(XR1::RightArm));
}

void XR1ControllerOL::subscribeLeftArmPosition(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->setJointPosition(XR1::LeftArm , ArmMsgs2VectorXd(msg));
	setJointPosition(XR1::LeftArm , XR1_ptr->getTargetPosition(XR1::LeftArm));
}

void XR1ControllerOL::subscribeRightArmPosition(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->setJointPosition(XR1::RightArm , ArmMsgs2VectorXd(msg));
	setJointPosition(XR1::RightArm , XR1_ptr->getTargetPosition(XR1::RightArm));
}




void XR1ControllerOL::subscribeMainBodyPosition(const xr1controllerros::BodyMsgs& msg) {
	XR1_ptr->setJointPosition(XR1::MainBody , BodyMsgs2VectorXd(msg));
	setJointPosition(XR1::MainBody , XR1_ptr->getTargetPosition(XR1::MainBody ));
}

void XR1ControllerOL::subscribeMainBodyCurrent(const xr1controllerros::BodyMsgs& msg) {
	XR1_ptr->setJointCurrent(XR1::MainBody , BodyMsgs2VectorXd(msg));
	setJointCurrent(XR1::MainBody , XR1_ptr->getTargetCurrent(XR1::MainBody));
}


void XR1ControllerOL::subscribeLeftHandPosition(const xr1controllerros::HandMsgs& msg) {
	XR1_ptr->setJointPosition(XR1::LeftHand , HandsMsgs2VectorXd(msg));
	setJointPosition(XR1::LeftHand , XR1_ptr->getTargetPosition(XR1::LeftHand));
}



void XR1ControllerOL::subscribeRightHandPosition(const xr1controllerros::HandMsgs& msg) {
	XR1_ptr->setJointPosition(XR1::RightHand , HandsMsgs2VectorXd(msg));
	setJointPosition(XR1::RightHand , XR1_ptr->getTargetPosition(XR1::RightHand));
}

void XR1ControllerOL::actuatorOperation(uint8_t nId, uint8_t nType)
{
	ROS_INFO("Getting launch finished");
	switch (nType) {
	case Actuator::Recognize_Finished:
		if (m_pController->hasAvailableActuator()) {

		}
		break;
	case Actuator::Launch_Finished:
		if (allActuatorHasLaunched())
		{
			std_msgs::Bool stuff;
			stuff.data = true;
			ActuatorLaunchedPublisher.publish(stuff);
		}

		break;
	default:
		break;
	}
}

void XR1ControllerOL::readingCallback(const ros::TimerEvent& this_event) {

	// ROS_INFO(" the current time is [%f]" , (float)this_event.current_real);

	for (uint8_t i = XR1::Left_Shoulder_X; i < XR1::LeftHand; ++i)
	{
		if ((int)m_pController->getActuatorAttribute(i, Actuator::ACTUATOR_SWITCH) == Actuator::ACTUATOR_SWITCH_ON)
		{
			m_pController->getCVPValue(i);
		}
	}



	for (uint8_t i = XR1::Back_Z; i <= XR1::Back_Y; ++i)
	{
		if ((int)m_pController->getActuatorAttribute(i, Actuator::ACTUATOR_SWITCH) == Actuator::ACTUATOR_SWITCH_ON)
		{
			m_pController->regainAttrbute(i, Actuator::ACTUAL_POSITION);
			m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
//            m_pController->regainAttrbute(i, Actuator::ACTUAL_VELOCITY);
		}
	}

	for (uint8_t i = XR1::Neck_Z; i < XR1::LeftArm; ++i)
	{
		if ((int)m_pController->getActuatorAttribute(i, Actuator::ACTUATOR_SWITCH) == Actuator::ACTUATOR_SWITCH_ON)
		{
			m_pController->regainAttrbute( i, Actuator::ACTUAL_POSITION);
		}
	}

	if (hand_command_switch) {
		for (uint8_t i = XR1::LeftHand; i < XR1::Actuator_Total; ++i)
		{
			if ((int)m_pController->getActuatorAttribute( i, Actuator::ACTUATOR_SWITCH) == Actuator::ACTUATOR_SWITCH_ON)
			{
				m_pController->regainAttrbute(i, Actuator::ACTUAL_POSITION);
				m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
			}
		}
	}



	if ((int)m_pController->getActuatorAttribute((uint8_t)XR1::Knee_X, Actuator::ACTUATOR_SWITCH) == Actuator::ACTUATOR_SWITCH_ON)
	{
		m_pController->regainAttrbute((uint8_t)XR1::Knee_X, Actuator::ACTUAL_POSITION);
	}

	hand_command_switch = !hand_command_switch;

}



void XR1ControllerOL::subscribeMainBodyMode(const xr1controllerros::ChainModeChange& msg) {
	setControlMode(XR1::MainBody , msg.Mode);
}
void XR1ControllerOL::subscribeLeftArmMode(const xr1controllerros::ChainModeChange& msg) {
	setControlMode(XR1::LeftArm , msg.Mode);
}
void XR1ControllerOL::subscribeRightArmMode(const xr1controllerros::ChainModeChange& msg) {
	setControlMode(XR1::RightArm , msg.Mode);
}
void XR1ControllerOL::subscribeLeftHandMode(const xr1controllerros::ChainModeChange& msg) {
	setControlMode(XR1::LeftHand , msg.Mode);
}
void XR1ControllerOL::subscribeRightHandMode(const xr1controllerros::ChainModeChange& msg) {
	setControlMode(XR1::RightHand , msg.Mode);
}


void XR1ControllerOL::unleaseCallback(const ros::TimerEvent&) {
	// ROS_INFO("Unleasing");
	MainBodyPositionPublisher.publish(ConvertBodyMsgs(XR1_ptr->getJointPositions(XR1::MainBody , true)));
	MainBodyCurrentPublisher.publish(ConvertBodyMsgs(XR1_ptr->getJointCurrents(XR1::MainBody , true)));

	LeftArmPositionPublisher.publish(ConvertArmMsgs(XR1_ptr->getJointPositions(XR1::LeftArm, true)));
	LeftArmVelocityPublisher.publish(ConvertArmMsgs(XR1_ptr->getJointVelocities(XR1::LeftArm, true)));
	LeftArmCurrentPublisher.publish(ConvertArmMsgs(XR1_ptr->getJointCurrents(XR1::LeftArm, true)));


	RightArmPositionPublisher.publish(ConvertArmMsgs(XR1_ptr->getJointPositions(XR1::RightArm , true)));
	RightArmVelocityPublisher.publish(ConvertArmMsgs(XR1_ptr->getJointVelocities(XR1::RightArm, true)));
	RightArmCurrentPublisher.publish(ConvertArmMsgs(XR1_ptr->getJointCurrents(XR1::RightArm , true)));

	LeftHandPositionPublisher.publish(ConvertHandMsgs(XR1_ptr->getJointCurrents(XR1::LeftHand , true)));
	RightHandPositionPublisher.publish(ConvertHandMsgs(XR1_ptr->getJointCurrents(XR1::RightHand , true)));



	broadcastTransform();
	// ROS_INFO("Unleased");
}




xr1controllerros::HandMsgs XR1ControllerOL::ConvertHandMsgs(Eigen::VectorXd HandPosition) {

	xr1controllerros::HandMsgs msg;
	msg.Thumb = HandPosition(0);
	msg.Index = HandPosition(1);
	msg.Middle = HandPosition(2);
	msg.Ring = HandPosition(3);
	msg.Pinky = HandPosition(4);

	return msg;
}

xr1controllerros::HandMsgs XR1ControllerOL::ConvertHandMsgs(std::vector<double> HandPosition) {
	xr1controllerros::HandMsgs msg;
	msg.Thumb = HandPosition[0];
	msg.Index = HandPosition[1];
	msg.Middle = HandPosition[2];
	msg.Ring = HandPosition[3];
	msg.Pinky = HandPosition[4];
	return msg;
}




xr1controllerros::ArmMsgs XR1ControllerOL::ConvertArmMsgs(std::vector<double> input) {

	xr1controllerros::ArmMsgs msg;

	msg.Shoulder_X = input[0];
	msg.Shoulder_Y = input[1];
	msg.Elbow_Z = input[2];
	msg.Elbow_X = input[3];
	msg.Wrist_Z = input[4];
	msg.Wrist_Y = input[5];
	msg.Wrist_X = input[6];

	return msg;
}

xr1controllerros::ArmMsgs XR1ControllerOL::ConvertArmMsgs(Eigen::VectorXd input) {
	xr1controllerros::ArmMsgs msg;

	msg.Shoulder_X = input(0);
	msg.Shoulder_Y = input(1);
	msg.Elbow_Z = input(2);
	msg.Elbow_X = input(3);
	msg.Wrist_Z = input(4);
	msg.Wrist_Y = input(5);
	msg.Wrist_X = input(6);

	return msg;
}

xr1controllerros::BodyMsgs XR1ControllerOL::ConvertBodyMsgs(std::vector<double> input) {

	xr1controllerros::BodyMsgs msg;

	msg.Knee   = input[0];
	msg.Back_Z = input[1];
	msg.Back_X = input[2];
	msg.Back_Y = input[3];
	msg.Neck_Z = input[4];
	msg.Neck_X = input[5];
	msg.Head = input[6];

	return msg;
}

xr1controllerros::BodyMsgs XR1ControllerOL::ConvertBodyMsgs(Eigen::VectorXd input) {

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

void XR1ControllerOL::broadcastTransform() {

	Eigen::Affine3d itsafine;
	tf::StampedTransform transform;

	// This function triggers almost all the computation in the library
	XR1_ptr->triggerCalculation();

	// Publish the left one
	XR1_ptr->getEndEfftorTransformation(XR1::LeftArm , itsafine);
	tf::transformEigenToTF(itsafine , transform);
	EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


	// Publish the right one
	XR1_ptr->getEndEfftorTransformation(XR1::RightArm , itsafine);
	// std::cout << itsafine.matrix() << std::endl;
	tf::transformEigenToTF(itsafine , transform);
	EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));

	lookupRightEFFTarget(transform, itsafine);
	lookupLeftEFFTarget(transform, itsafine);

}


void XR1ControllerOL::lookupRightEFFTarget(tf::StampedTransform & transform, 	Eigen::Affine3d & itsafine) {

	try {
		EFF_Listener.lookupTransform( "/Back_Y", "/RightEndEffectorTarget",
		                               ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		return;
	}

	transformTFToEigen(transform, itsafine);
	XR1_ptr->setEndEffectorPosition(XR1::RightArm, itsafine , RightElbowAngle);

	RightArmPositionPublisher.publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::RightArm)));
}

void XR1ControllerOL::lookupLeftEFFTarget(tf::StampedTransform & transform, 	Eigen::Affine3d & itsafine) {
	try {
		EFF_Listener.lookupTransform( "/Back_Y", "/LeftEndEffectorTarget",
		                               ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		return;
	}

	transformTFToEigen(transform, itsafine);
	XR1_ptr->setEndEffectorPosition(XR1::LeftArm, itsafine , LeftElbowAngle);

	LeftArmPositionPublisher.publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::LeftArm)));
}

void XR1ControllerOL::subscribeLeftElbowAngle(const std_msgs::Float64 & msg) {
	LeftElbowAngle = msg.data;
}
void XR1ControllerOL::subscribeRightElbowAngle(const std_msgs::Float64 & msg) {
	RightElbowAngle = msg.data;
}