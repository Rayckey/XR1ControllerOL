#ifndef XR1ControllerOL_H
#define XR1ControllerOL_H

#include "ros/ros.h"
#include "actuatorcontroller.h"
#include "xr1controller.h"
#include "xr1define.h"


// Messages for Communication
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerros/HandMsgs.h"
#include "xr1controllerros/JointAttributeMsgs.h"

#include "xr1controllerros/ChainModeChange.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "xr1controllerros/JointAttributeMsgs.h"
// Messages for Communication


#include "Eigen/Dense"


using namespace Eigen;


class XR1ControllerOL
{

public:
	XR1ControllerOL();

	~XR1ControllerOL();

	//Identical to startSimulation() in simulation, trigger sync mode and start the simulation
	//Used in the XR1Controller
	void launchAllMotors();//
	void startSimulation();

	//Identical to stopAllMotors() in simulation, stop the simulation
	void stopSimulation();//

	//Identical to stopSimulation() in simulation, stop the simulation
	//Used in the XR1Controller
	void stopAllMotors();//

	//Get the simulation time in double (s)
	//Used in Simulation
	double getSimulationTime();


	void updatingCallback(uint8_t id, uint8_t attrId, double value);

	//--------Joint Control----------------------------------

	//Set the Joint PID values for a joint
	//Used in the XR1Controller
	//Argu: Joint ID , Attribute ID , value
	//Reutrns : void , may add error message in the fulture
	void setJointAttribute(uint8_t joint_idx , uint8_t attribute_idx , double value);


	//Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID , Angles contained in Eigen::VectorXd
	//Reutrns : void , may add error message in the fulture
	void setJointPosition(uint8_t control_group , VectorXd JA);


	//Set the Target Joint Positions for a single joint, i.e. LeftShoulderX , RightWristZ
	//Used in the XR1Controller
	//Argu: Control Group ID , Angles contained in std::vector<double>
	//Reutrns : void , may add error message in the fulture
	void setJointPosition(uint8_t joint_idx ,   double JA);


	//Set the Target Joint Velocity for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID , Angular Velocity contained in Eigen::VectorXd
	//Reutrns : void , may add error message in the fulture
	void setJointVelocity(uint8_t control_group , VectorXd JV);


	//Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
	//Used in the XR1Controller
	//Argu: Control Group ID , Angular Velocity contained in std::vector<double>
	//Reutrns : void , may add error message in the fulture
	void setJointVelocity(uint8_t joint_idx ,   double JV);


	//Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID , Target Current
	//Reutrns : void , may add error message in the fulture
	void setJointCurrent(uint8_t control_group , VectorXd JC);


	//Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
	//Used in the XR1Controller
	//Argu: Control Group ID , Angular Velocity contained in std::vector<double>
	//Reutrns : void , may add error message in the fulture
	void setJointCurrent(uint8_t joint_idx ,   double JC);


	//Set the Control Method for an entire Control Group , i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID , Conrol Mode ID
	//Reutrns : void , may add error message in the fulture
	void setControlMode(uint8_t control_group , uint8_t option);



	std::map<uint8_t, std::vector<uint8_t> > control_group_map;

	std::map<uint8_t, uint8_t> attribute_map;

	std::map<uint8_t, Actuator::ActuatorMode> mode_map;




	// Convert Joint ROS Messages

	Eigen::VectorXd BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs& msg);

	Eigen::VectorXd ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs& msg);

	Eigen::VectorXd HandsMsgs2VectorXd(const xr1controllerros::HandMsgs& msg);




	// Things regarding actuator controller
	void readingCallback(const ros::TimerEvent&);

	void unleaseCallback(const ros::TimerEvent&);

	void actuatorOperation(uint8_t nId, uint8_t nType);

	bool allActuatorHasLaunched();
protected:

	void subscribeLaunch(const std_msgs::Bool& msg);

	void subscribeShutdown(const std_msgs::Bool& msg);

	void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs& msg);

	void subscribeMainBodyCurrent(const xr1controllerros::BodyMsgs& msg);

	void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs& msg);

	void subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs& msg);

	void subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs& msg);

	void subscribeRightArmPosition(const xr1controllerros::ArmMsgs& msg);

	void subscribeRightArmVelocity(const xr1controllerros::ArmMsgs& msg);

	void subscribeRightArmCurrent(const xr1controllerros::ArmMsgs& msg);

	void subscribeLeftHandPosition(const xr1controllerros::HandMsgs& msg);

	void subscribeRightHandPosition(const xr1controllerros::HandMsgs& msg);


	void subscribeMainBodyMode(const xr1controllerros::ChainModeChange& msg);
	void subscribeLeftArmMode(const xr1controllerros::ChainModeChange& msg);
	void subscribeRightArmMode(const xr1controllerros::ChainModeChange& msg);
	void subscribeLeftHandMode(const xr1controllerros::ChainModeChange& msg);
	void subscribeRightHandMode(const xr1controllerros::ChainModeChange& msg);



	void broadcastTransform();

	xr1controllerros::HandMsgs ConvertHandMsgs(Eigen::VectorXd HandPosition);

	xr1controllerros::HandMsgs ConvertHandMsgs(std::vector<double> HandPosition);

	xr1controllerros::ArmMsgs  ConvertArmMsgs(std::vector<double> input) ;

	xr1controllerros::ArmMsgs  ConvertArmMsgs(Eigen::VectorXd input) ;

	xr1controllerros::BodyMsgs ConvertBodyMsgs(std::vector<double> input);

	xr1controllerros::BodyMsgs ConvertBodyMsgs(Eigen::VectorXd input);

	void lookupRightEFFTarget(tf::StampedTransform & transform, 	Affine3d & itsafine);

	void lookupLeftEFFTarget(tf::StampedTransform & transform, 	Affine3d & itsafine);

	void subscribeLeftElbowAngle(const std_msgs::Float64 & msg);
	void subscribeRightElbowAngle(const std_msgs::Float64 & msg);
private:

	// Pay no Attention Here Plz
	ros::NodeHandle nh;
	ActuatorController * m_pController;

	XR1Controller * XR1_ptr;

	double LeftElbowAngle;
	double RightElbowAngle;
	Matrix4d temp_4d;

	tf::TransformBroadcaster EFF_Broadcaster;
	tf::TransformListener EFF_Listener;


	ros::Publisher ActuatorLaunchedPublisher;

	ros::Subscriber LaunchSubscriber;

	ros::Subscriber ShutdownSubscriber;

	ros::Subscriber MainBodyModeChangeSubscriber ;

	ros::Subscriber MainBodyCurrentSubscriber;

	ros::Subscriber LeftArmModeChangeSubscriber  ;

	ros::Subscriber RightArmModeChangeSubscriber ;

	ros::Subscriber LeftHandModeChangeSubscriber ;

	ros::Subscriber RightHandModeChangeSubscriber  ;

	ros::Subscriber LeftHandPositionSubscriber;

	ros::Subscriber RightHandPositionSubscriber;

	ros::Subscriber JointVisualizationSubscriber;

	ros::Subscriber LeftArmPositionSubscriber;

	ros::Subscriber RightArmPositionSubscriber;

	ros::Subscriber MainBodyPositionSubscriber;

	ros::Subscriber LeftArmVelocitySubscriber;

	ros::Subscriber RightArmVelocitySubscriber;

	ros::Subscriber LeftArmCurrentSubscriber;

	ros::Subscriber RightArmCurrentSubscriber;

	ros::Subscriber LeftHandCurrentSubscriber;

	ros::Subscriber RightHandCurrentSubscriber;

	ros::Subscriber LeftElbowSubscriber;
	ros::Subscriber RightElbowSubscriber;

	ros::Publisher JointAttributePublisher;

	ros::Publisher MainBodyPositionPublisher ;
	ros::Publisher MainBodyCurrentPublisher      ;
	ros::Publisher LeftArmPositionPublisher      ;
	ros::Publisher LeftArmVelocityPublisher      ;
	ros::Publisher LeftArmCurrentPublisher       ;
	ros::Publisher RightArmPositionPublisher     ;
	ros::Publisher RightArmVelocityPublisher     ;
	ros::Publisher RightArmCurrentPublisher      ;
	ros::Publisher LeftHandPositionPublisher   ;
	ros::Publisher RightHandPositionPublisher  ;





	// Very useless temp varibles
	Matrix4d temp_trans;
	bool hand_command_switch;


}; //class

#endif // my_namespace__my_plugin_H
