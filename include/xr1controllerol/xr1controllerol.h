#ifndef XR1ControllerOL_H
#define XR1ControllerOL_H

#include "ros/ros.h"
#include "actuatorcontroller.h"
#include "xr1controller.h"
#include "xr1controlleralp.h"
#include "xr1controllerblc.h"
#include "xr1define.h"
#include "XR1IMUmethods.h"


// Messages for Communication
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerros/HeadMsgs.h"
#include "xr1controllerros/HandMsgs.h"

#include "xr1controllerros/ChainModeChange.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

// Hopps Port messages
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>


// The IK planner message service
#include "xr1controllerol/IKLinearService.h"
#include "xr1controllerol/IKTrackingService.h"
#include "xr1controllerol/HandGripQuery.h"
#include "xr1controllerol/askReadiness.h"


// The animation message type
#include "xr1controllerol/AnimationMsgs.h"
#include "xr1controllerol/AnimationQuery.h"



// Ultility stuff
#include "Eigen/Dense"
#include "xr1controllerolmsgulit.h"


// Collision detection and Robot State Serive
#include "xr1controllerol/RobotStateQuery.h"

// std stuff
#include <map>
#include <string>
#include "std_srvs/SetBool.h"

using namespace Eigen;


class XR1ControllerOL {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    XR1ControllerOL();

    ~XR1ControllerOL();


    // Power Control Functions ------------------------------------------------------------------
    //Identical to startSimulation() in simulation, launch all the motors
    //Both does the same thing, I think one function calls the other, idk tho, too lazy to check
    void launchAllMotors();
    void startSimulation();

    //Identical to stopSimulation() in simulation, turn off all the motors
    //MAKE SURE THE ROBOT IS SECURED WHEN YOU DO THIS!
    //Both does the same thing, I think one function calls the other, idk tho, too lazy to check
    void stopSimulation();
    void stopAllMotors();
    // --------------------------------------------------------------------------------------------




    // Straight up Joint Control------------------------------------------------------------------

    //Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the future
    void setControlGroupTarget(uint8_t control_group);

    void setJointTarget(uint8_t joint_id);


    // ------------------------------------------------------------------------------------------


    // Retrieve joint values -------------------------------------------------------------------------
    // In some instances, we need to get values out without messages too

    // Get the target joint position, set vanilla to true to get simulation value
    double getTargetJointPosition(uint8_t joint_id, bool vanilla = false);

    //Get Target Position for Arms or Body
    void getTargetPosition(uint8_t control_group, VectorXd &output_ref, bool vanilla = false);

    // Get transformation information
    void getEndEffectorTransformation(uint8_t control_group, Affine3d &TransformationReference);

    // Get the current elbow anlge
    // THIS WILL TRIGGER AN INTERNAL CALCULATION, don't use it too much in a loop, I mean why would you ?
    double getElbowAngle(uint8_t control_gourp);
    // -------------------------------------------------------------------------------------------------


    // Control Mode Shenanigans --------------------------------------------------------------
    //Set the Control Method for an entire Control Group , i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Control Mode ID
    //Reutrns : void , may add error message in the fulture
    void setControlMode(uint8_t control_group, uint8_t option);


    //Set the Sub Control Method for an entire Control Group , i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Control Mode ID
    //Reutrns : void , may add error message in the fulture
    void setSubControlMode(uint8_t control_group, uint8_t option);

    // ----------------------------------------------------------------------------------



    // Animation Library Player Stuff ------------------------------------

    // animation main loop, does every thing there
    void animationCallback();

    // Clear the animation data an queue
    void clearStates();


    // -------------------------------------------------------------------




    // Animation Library Player Messages and Services -------------------

    // receive animation start signal
    // WILL ALSO CHANGE ALL THE SUB CONTROL MODES
    void subscribeStartAnimation(const std_msgs::Bool &msg);

    // receive an animation order
    // one animation coming right up , chump
    void subscribeSetAnimation(const xr1controllerol::AnimationMsgs &msg);


    // receive idle on/off optionIdle
    // will determine if an idle animation will be played when default runs out
    void subscribeSetIdle(const std_msgs::Bool &msg);

    // ------------------------------------------------------------------



    // Omni wheels control -------------------------------------------

    // change the mode of the OmniWheels
    void activateOmni();

    // apply velocity commands to Omniwheels
    void Omni2Actuator();
    // --------------------------------------------------------------


    // Collision Detection messages ---------------------------------------

    //Receive a message to start collision detection
    // THIS WILL OVERWRITE THE INVERSE DYNAMICS MODE
    // THE GRAVITY COMPENSATION SHOULD BE TURNED OFF
    void subscribeSetCollisionDetection(const std_msgs::Bool &msg);
    // --------------------------------------------------------------------


    // Collision Detection related function -------------------------------

    // Detect collision here, does all the stuff internally
    void collisionDetectionCallback();

    // --------------------------------------------------------------------


    // MoCap main loop ----------------------------------------------------
    // The targets are saved in target joint positions
    void MoCapCallback(const ros::TimerEvent &);
    // --------------------------------------------------------------------


    // Main High frequency control call back ------------------------------

    // Main Loop, trigger most of other callbacks
    void unleaseCallback(const ros::TimerEvent &);

    // Call back to publish all the dame joint information and tf
    void unleaseJointInfo();

    // FOR HIGH FREQUENCY COMMANDS
    // Apply the target joint angles, velocities, or currents to the robot
    // Not activated in low frequency mode
    void applyJointsTargets();

    // -------------------------------------------------------------------


    // Actuator Controller Shenanigans ------------------------------------

    // Request all the important joint information
    void readingCallback();

    // figure out the precise modes that each control modes should be in
    void judgeControlGroupModes();

    // figure out the precise modes that each actuators should be in
    void judgeActuatorModes(uint8_t control_group);

    // WHAT DO I DO ?!
    void actuatorOperation(uint8_t nId, uint8_t nType);

    // determine if all the actuators have been launched
    bool allActuatorHasLaunched();

    // ---------------------------------------------------------------------







protected:


    // Actuator Connection funcitons -------------------------------------------------
    //This function gets called whenever the actuator controller return a value
    //Not meant to be used outside
    void updatingCallback(uint8_t id, uint8_t attrId, double value);

    // --------------------------------------------------------------------------------


    // Power control Message Subscriber -------------------------------------------
    void subscribeLaunch(const std_msgs::Bool &msg);

    void subscribeShutdown(const std_msgs::Bool &msg);

    void subscribeEStop(const std_msgs::Bool &msg);
    // ----------------------------------------------------------------------------


    // Target State Control Message Subscriber -------------------------------------
    void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs &msg);

    void subscribeMainBodyCurrent(const xr1controllerros::BodyMsgs &msg);

    void subscribeHeadBodyPosition(const xr1controllerros::HeadMsgs &msg);

    void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs &msg);

    void subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs &msg);

    void subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs &msg);

    void subscribeRightArmPosition(const xr1controllerros::ArmMsgs &msg);

    void subscribeRightArmVelocity(const xr1controllerros::ArmMsgs &msg);

    void subscribeRightArmCurrent(const xr1controllerros::ArmMsgs &msg);

    void subscribeLeftHandPosition(const xr1controllerros::HandMsgs &msg);

    void subscribeRightHandPosition(const xr1controllerros::HandMsgs &msg);

    void subscribeLeftHandCurrent(const xr1controllerros::HandMsgs &msg);

    void subscribeRightHandCurrent(const xr1controllerros::HandMsgs &msg);
    // -----------------------------------------------------------------------------

    // Omni messages --------------------------------------------------------
    void subscribeOmniCommands(const geometry_msgs::Twist & msg);
    // ----------------------------------------------------------------------


    // Mode change Subscriber ------------------------------------------------------
    void subscribeRobotMode(const xr1controllerros::ChainModeChange &msg);
    // -----------------------------------------------------------------------------


    // IK related message and services ---------------------------------------------
    bool serviceIKPlanner(xr1controllerol::IKLinearServiceRequest &req,
                          xr1controllerol::IKLinearServiceResponse &res);

    bool serviceIKTracking(xr1controllerol::IKTrackingServiceRequest &req,
                           xr1controllerol::IKTrackingServiceResponse &res);

    bool serviceHandGrip(xr1controllerol::HandGripQueryRequest &req,
                         xr1controllerol::HandGripQueryResponse &res);

    bool serviceQueryAnimation(xr1controllerol::AnimationQueryRequest &req,
                               xr1controllerol::AnimationQueryResponse &res);
    // ------------------------------------------------------------------------------


    // Service that determine if the robot is ready ---------------
    bool serviceReady(xr1controllerol::askReadinessRequest & req,
            xr1controllerol::askReadinessResponse & res);
    // ------------------------------------------------------------


    // Service that report the robot state ------------------------
    bool serviceState(xr1controllerol::RobotStateQueryRequest & req,
            xr1controllerol::RobotStateQueryResponse & res);
    // ------------------------------------------------------------

    // FK related messages, tf ,and services -----------------------------------
    void broadcastTransform();
    // -------------------------------------------------------------------------



    // Hopps port -------------------------------------------------------

    void setupJointStateTable();

    void subscribeJointStates(const sensor_msgs::JointState & msg);

    void publishJointStates();

    void subscribeSpecial(const std_msgs::Int8 & msg);
    // ------------------------------------------------------------------


    // tilting and stuff -----------------------------------------------
    void requestQue();
    void subscribeTiltStart(const std_msgs::Bool &msg);
    void QuaCallBack(uint64_t id, double w, double x, double y, double z);
    void accCallBack(uint8_t id , double x , double y , double z , int pres);
    // -----------------------------------------------------------------

    // LEGACY ----------------------------------------------------------------

//    void subscribeMoCapInit(const std_msgs::Bool &msg);

//    void requestAcc(const ros::TimerEvent &);

//    void stateTransition();


    //MoCap Information we get from the actuator controller
    //Can be deactivated to save some resources

    // -----------------------------------------------------------------------





private:

    // Pay no Attention Here Plz ---------------------------
    ros::NodeHandle nh;
    ActuatorController *m_pController;

    XR1Controller *XR1_ptr;
    XR1ControllerALP *XRA_ptr;
    XR1ControllerBLC *XRB_ptr;
    XR1IMUmethods *IMU_ptr;
    std::vector<uint8_t> temp_ids;
    bool RecognizeFinished;
    int low_frequency_threshold;
    int low_frequency_counter;
    // -----------------------------------------------------


    // Very important mode variables -----------------------------------
    std::map<uint8_t, std::vector<uint8_t> > control_group_map;
    std::vector<uint8_t> control_group_flags;
    std::map<uint8_t, uint8_t> attribute_map;
    std::map<uint8_t, Actuator::ActuatorMode> mode_map;
    std::map<int, int> control_modes;
    bool collision_detection_switch;
    int previous_omni_state;
    int omni_cmd_expire_counter;
    // -----------------------------------------------------------------



    // Power Control messages ----------------------
    ros::Subscriber LaunchSubscriber;
    ros::Subscriber ShutdownSubscriber;
    ros::Subscriber EStopSubscriber;
    // ---------------------------------------------


    // Mode change Subscriber ----------------------
    ros::Subscriber ModeChangeSubscriber;
    // ---------------------------------------------


    // Body States Publishers ------------------------
    ros::Publisher HeadBodyPositionPublisher;
    ros::Publisher HeadBodyVelocityPublisher;
    ros::Publisher HeadBodyCurrentPublisher;
    ros::Publisher MainBodyPositionPublisher;
    ros::Publisher MainBodyVelocityPublisher;
    ros::Publisher MainBodyCurrentPublisher;
    ros::Publisher LeftArmPositionPublisher;
    ros::Publisher LeftArmVelocityPublisher;
    ros::Publisher LeftArmCurrentPublisher;
    ros::Publisher RightArmPositionPublisher;
    ros::Publisher RightArmVelocityPublisher;
    ros::Publisher RightArmCurrentPublisher;
    ros::Publisher LeftHandPositionPublisher;
    ros::Publisher RightHandPositionPublisher;
    ros::Publisher LeftHandCurrentPublisher;
    ros::Publisher RightHandCurrentPublisher;
    // ------------------------------------------------



    // Body Target State Subscriber -----------------------
    ros::Subscriber MainBodyPositionSubscriber;
    ros::Subscriber MainBodyCurrentSubscriber;
    ros::Subscriber HeadBodyPositionSubscriber;
    ros::Subscriber LeftArmPositionSubscriber;
    ros::Subscriber RightArmPositionSubscriber;
    ros::Subscriber LeftArmVelocitySubscriber;
    ros::Subscriber RightArmVelocitySubscriber;
    ros::Subscriber LeftArmCurrentSubscriber;
    ros::Subscriber RightArmCurrentSubscriber;
    ros::Subscriber LeftHandPositionSubscriber;
    ros::Subscriber RightHandPositionSubscriber;
    ros::Subscriber LeftHandCurrentSubscriber;
    ros::Subscriber RightHandCurrentSubscriber;
    // ---------------------------------------------------



    // IK related messages and services -------------
    ros::Subscriber LeftElbowSubscriber;
    ros::Subscriber RightElbowSubscriber;
    ros::ServiceServer IKPlannerService;
    ros::ServiceServer IKTrackingService;
    ros::ServiceServer HandGripService;
    ros::ServiceServer QueryAnimationService;
    // --------------------------------------------


    // FK related stuff ---------------------------
    tf::TransformBroadcaster EFF_Broadcaster;
    tf::TransformListener EFF_Listener;
    // --------------------------------------------



    // LEGACY-------------------------------------
    ros::Subscriber tiltInitSubscriber;
    // -------------------------------------------


    // Local MoCap system messages ---------------
    ros::Subscriber MoCapInitSubscriber;
    // -------------------------------------------


    // Check readness of the robot ---------------
    ros::ServiceServer ReadinessService;
    // -------------------------------------------


    // Check Robot State (Collision , calculation)-------
    ros::ServiceServer RobotStateService;
    // --------------------------------------------------


    // Animation subscriber -----------------------
    ros::Subscriber AnimationSwitchSubscriber;
    ros::Subscriber AnimationSetSubscriber;
    ros::Subscriber IdleSwitchSubscriber;

    // --------------------------------------------


    // Collision detection subscriber and publisher---------
    ros::Subscriber CollisionDetectionSubscriber;
    ros::Publisher CollisionEventPublisher;
    // -----------------------------------------------------


    // OmniWheels Information --------------------
    ros::Publisher OmniSpeedPublisher;
    ros::Subscriber OmniSpeedSubscriber;
    // -------------------------------------------



    // Hopps Port varibles, subscribers and publishers -----------------------

    sensor_msgs::JointState temp_jointstate;

    ros::Publisher m_joint_state_publisher;

    ros::Subscriber m_joint_state_subscriber;

    std::map<std::string , uint8_t> m_jointlookup;

    std::map<uint8_t , std::string> m_namelookup;

    std::map<uint8_t , uint8_t> m_control_group_lookup;

    // -------------------------------------------

    // tempeatury varibles, need to eliminate in the fulture ----------------

    ros::Subscriber m_special_subscriber;

    // ----------------------------------------------------------------------



    // Very useless temporary variables ----------------
    Matrix4d temp_trans;
    VectorXd temp_vec5d;
    VectorXd temp_vec7d;
    VectorXd temp_vec3d;
    VectorXd temp_vec4d;
    Vector3d temp_omni_cmd;
    Matrix4d temp_4d;
    xr1controllerros::HandMsgs temp_handmsgs;
    xr1controllerros::ArmMsgs temp_armmsgs;
    xr1controllerros::BodyMsgs temp_bodymsgs;
    xr1controllerros::HeadMsgs temp_headmsgs;
    bool hand_command_switch;
    int unlease_counter;
    int power_reading_counter;
    Affine3d itsafine;
    tf::StampedTransform transform;
    geometry_msgs::Transform temp_geo_trans;
    double temp_value;
    // ---------------------------------------------


}; //class

#endif // my_namespace__my_plugin_H
