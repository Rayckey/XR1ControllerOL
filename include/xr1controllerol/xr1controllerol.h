#ifndef XR1ControllerOL_H
#define XR1ControllerOL_H

#include "ros/ros.h"
#include "actuatorcontroller.h"
#include "xr1controller.h"
#include "xr1controlleralp.h"
#include "xr1define.h"
#include "XR1IMUmethods.h"


// Messages for Communication
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerros/HandMsgs.h"
#include "xr1controllerros/JointAttributeMsgs.h"

#include "xr1controllerros/ChainModeChange.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

// The IK planner message service
#include "xr1controllerol/IKLinearService.h"
#include "xr1controllerol/IKTrackingService.h"

// The animation message type
#include "xr1controllerol/AnimationMsgs.h"


#include "Eigen/Dense"
#include "xr1controllerolmsgulit.h"


using namespace Eigen;



class XR1ControllerOL {

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


    void updatingCallback(uint8_t id, uint8_t attrId, double value);

    void QuaCallBack(uint64_t id, double w, double x, double y, double z);

    // void accCallBack(uint8_t id , double x , double y , double z , int pres);


    //--------Joint Control----------------------------------

    //Set the Joint PID values for a joint
    //Used in the XR1Controller
    //Argu: Joint ID , Attribute ID , value
    //Reutrns : void , may add error message in the fulture
    void setJointAttribute(uint8_t joint_idx, uint8_t attribute_idx, double value);


    //Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the future
    void setJointPosition(uint8_t control_group, VectorXd & JA);


    //Set the Target Joint Positions for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(uint8_t joint_idx, double JA);


    //Set the Target Joint Velocity for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(uint8_t control_group, VectorXd & JV);


    //Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(uint8_t joint_idx, double JV);


    //Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Target Current
    //Reutrns : void , may add error message in the fulture
    void setJointCurrent(uint8_t control_group, VectorXd & JC);


    //Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointCurrent(uint8_t joint_idx, double JC);


    // Get the target joint position, set vanilla to true to get simulation value
    double getTargetJointPosition(uint8_t joint_id, bool vanilla = false);

    //Get Target Position for Arms or Body
    VectorXd getTargetPosition(uint8_t control_group, bool vanilla = false);
    void getTargetPosition(uint8_t control_group, VectorXd  & output_ref , bool vanilla = false);


    void getEndEffectorTransformation(uint8_t control_group, Affine3d &TransformationReference);

    double getElbowAngle(uint8_t control_gourp);


    //Set the Control Method for an entire Control Group , i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Conrol Mode ID
    //Reutrns : void , may add error message in the fulture
    void setControlMode(uint8_t control_group, uint8_t option);


    //Set Control Mode for Entire XR1, which only has two mode: direct and drive
    //When in drive mode, it will take over
    void setMetaMode(const std_msgs::Int32 &msg);



    // Animation Library Stuff ------------------------------------

    // receive animation start signal
    void subscribeStartAnimation(const std_msgs::Bool& msg) ;


    // receive an animation order
    void subscribeSetAnimation(const xr1controllerol::AnimationMsgs& msg);


    // animation main loop
    void animationCallback();

    // change the mode of omniwheels
    void activateOmni();

    // apply velocities
    void Omni2Actuator();

    void subscribeSetCollisionDetection(const std_msgs::Bool & msg);

    void collisionDetectionCallback();

    // -------------------------------------------------------------




    // Things regarding actuator controller
    void readingCallback();

    void unleaseCallback(const ros::TimerEvent &);

    void unleaseJointInfo();

    void applyJointTarget();

    void gravityCallback();

    void requestAcc(const ros::TimerEvent &);

    void requestQue(const ros::TimerEvent &);

    void MoCapCallback(const ros::TimerEvent &);


    void actuatorOperation(uint8_t nId, uint8_t nType);

    bool allActuatorHasLaunched();

    void stateTransition();

    void switch2HighFrequency(bool option);
    void judgeControlGroupModes();
    void judgeActuatorModes(uint8_t control_group);





    // Some mumble jumble that no one clear about
    // Clear the path data
    void clearStates();


protected:

    void subscribeLaunch(const std_msgs::Bool &msg);

    void subscribeShutdown(const std_msgs::Bool &msg);

    void subscribeEStop(const std_msgs::Bool &msg);

    void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs &msg);

    void subscribeMainBodyCurrent(const xr1controllerros::BodyMsgs &msg);

    void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs &msg);

    void subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs &msg);

    void subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs &msg);

    void subscribeRightArmPosition(const xr1controllerros::ArmMsgs &msg);

    void subscribeRightArmVelocity(const xr1controllerros::ArmMsgs &msg);

    void subscribeRightArmCurrent(const xr1controllerros::ArmMsgs &msg);


    void subscribeMainBodyMode(const xr1controllerros::ChainModeChange &msg);

    void subscribeLeftArmMode(const xr1controllerros::ChainModeChange &msg);

    void subscribeRightArmMode(const xr1controllerros::ChainModeChange &msg);

    void subscribeLeftHandMode(const xr1controllerros::ChainModeChange &msg);

    void subscribeRightHandMode(const xr1controllerros::ChainModeChange &msg);

    void subscribeHeadBodyMode(const xr1controllerros::ChainModeChange &msg);

    void subscribeBackBodyMode(const xr1controllerros::ChainModeChange &msg);




    void subscribeLeftHandPosition(const xr1controllerros::HandMsgs &msg);

    void subscribeRightHandPosition(const xr1controllerros::HandMsgs &msg);

    void subscribeLeftHandCurrent(const xr1controllerros::HandMsgs &msg);

    void subscribeRightHandCurrent(const xr1controllerros::HandMsgs &msg);


    bool serviceIKPlanner(xr1controllerol::IKLinearServiceRequest & req ,
             xr1controllerol::IKLinearServiceResponse & res);
//    void subscribeIKLinearPlanner(const xr1controllerol::IKLinearTarget & msg);

    bool serviceIKTracking(xr1controllerol::IKTrackingServiceRequest & req ,
                           xr1controllerol::IKTrackingServiceResponse & res);

    void broadcastTransform();



    void subscribeLeftElbowAngle(const std_msgs::Float64 &msg);

    void subscribeRightElbowAngle(const std_msgs::Float64 &msg);



    void subscribetiltInit(const std_msgs::Bool &msg);

    void subscribeMoCapInit(const std_msgs::Bool &msg);





private:

    // Pay no Attention Here Plz
    ros::NodeHandle nh;
    ActuatorController *m_pController;

    XR1Controller *XR1_ptr;
    XR1ControllerALP * XRA_ptr;
    XR1IMUmethods *IMU_ptr;



    // Very important mode variables
    std::map<uint8_t, std::vector<uint8_t> > control_group_map;
    std::map<uint8_t, uint8_t> attribute_map;
    std::map<uint8_t, Actuator::ActuatorMode> mode_map;
    std::map<int , int> control_modes;
    bool high_frequency_switch;
    bool animation_switch;
    bool collision_detection_switch;
    bool previous_omni_state;




    double LeftElbowAngle;
    double RightElbowAngle;
    Matrix4d temp_4d;

    tf::TransformBroadcaster EFF_Broadcaster;
    tf::TransformListener EFF_Listener;


    ros::Publisher ActuatorLaunchedPublisher;

    ros::Subscriber LaunchSubscriber;

    ros::Subscriber ShutdownSubscriber;

    ros::Subscriber EStopSubscriber;

    ros::Subscriber MainBodyModeChangeSubscriber;

    ros::Subscriber MainBodyCurrentSubscriber;

    ros::Subscriber LeftArmModeChangeSubscriber;

    ros::Subscriber RightArmModeChangeSubscriber;

    ros::Subscriber LeftHandModeChangeSubscriber;

    ros::Subscriber RightHandModeChangeSubscriber;

    ros::Subscriber HeadBodyModeChangeSubscriber;

    ros::Subscriber BackBodyModeChangeSubscriber;

    ros::Subscriber JointVisualizationSubscriber;

    ros::Subscriber LeftArmPositionSubscriber;

    ros::Subscriber RightArmPositionSubscriber;

    ros::Subscriber MainBodyPositionSubscriber;

    ros::Subscriber LeftArmVelocitySubscriber;

    ros::Subscriber RightArmVelocitySubscriber;

    ros::Subscriber LeftArmCurrentSubscriber;

    ros::Subscriber RightArmCurrentSubscriber;

    ros::Subscriber LeftHandPositionSubscriber;

    ros::Subscriber RightHandPositionSubscriber;

    ros::Subscriber LeftHandCurrentSubscriber;

    ros::Subscriber RightHandCurrentSubscriber;

    ros::Subscriber LeftElbowSubscriber;
    ros::Subscriber RightElbowSubscriber;






    ros::Subscriber MetaModeSubscriber;

    ros::Subscriber tiltInitSubscriber;

    ros::Subscriber MoCapInitSubscriber;

    ros::ServiceServer IKPlannerService;

    ros::ServiceServer IKTrackingService;

    ros::Publisher JointAttributePublisher;

    ros::Publisher MainBodyPositionPublisher;
    ros::Publisher MainBodyCurrentPublisher;
    ros::Publisher LeftArmPositionPublisher;
    ros::Publisher LeftArmVelocityPublisher;
    ros::Publisher LeftArmCurrentPublisher;
    ros::Publisher RightArmPositionPublisher;
    ros::Publisher RightArmVelocityPublisher;
    ros::Publisher RightArmCurrentPublisher;
    ros::Publisher LeftHandPositionPublisher;
    ros::Publisher RightHandPositionPublisher;



    // Animation subscriber
    ros::Subscriber AnimationSwitchSubscriber;
    ros::Subscriber AnimationSetSubscriber;
    ros::Subscriber CollisionDetectionSubscriber;

    // Very useless temp varibles
    Matrix4d temp_trans;
    VectorXd temp_vec5d;
    VectorXd temp_vec7d;
    VectorXd temp_vec3d;
    xr1controllerros::HandMsgs temp_handmsgs;
    xr1controllerros::ArmMsgs temp_armmsgs;
    xr1controllerros::BodyMsgs temp_bodymsgs;
    bool hand_command_switch;
    int power_reading_counter;


    Eigen::Affine3d itsafine;
    tf::StampedTransform transform;
    geometry_msgs::Transform temp_geo_trans;
}; //class

#endif // my_namespace__my_plugin_H
