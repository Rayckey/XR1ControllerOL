#ifndef XR1CONTROLLERPM_H
#define XR1CONTROLLERPM_H


#include "genericcontroller.h"
#include "Eigen/Dense"
#include "chaincontroller.h"
#include "headcontroller.h"
#include "backcontroller.h"
#include "handcontroller.h"
#include "omnicontroller.h"
#include "dynamicmethod.h"
#include "IKplanner.h"
#include <map>
#include <vector>
#include <deque>
#include <list>
#include "Eigen/Geometry"
#include "xr1define.h"
#include "xr1controllerutil.h"





class XR1ControllerPM
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    XR1ControllerPM(string parameters_path);



    // The E STOP command
    void employLockdown();


    //Simple Joint Controls--------------------------------------

    //Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(uint8_t control_group ,VectorXd & JA);

    //Set the Target Joint Velocity for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(uint8_t control_group ,VectorXd & JV);


    //Set the Target Joint Acceleration for entire control group, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointAcceleration(uint8_t control_group , VectorXd &JACC);

    //Set the Target Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Target Current
    //Reutrns : void , may add error message in the fulture
    void setJointCurrent(uint8_t control_group , VectorXd & JC);


    //Set the Target Joint Positions for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(uint8_t JointID ,double JA);

    //Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(uint8_t JointID ,double JV);


    //Set the Target Joint Acceleration for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointAcceleration(uint8_t JointID ,double JACC);

    //Set the Target Joint Currents for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointCurrent(uint8_t JointID ,double JC);

    //Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : JointAngles contained in Eigen::VectorXd
    VectorXd getJointPositions(uint8_t control_group);
    void getJointPositions(uint8_t control_group , VectorXd & output_ref);

    //Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : JointAngles contained in std::vector
    std::vector<double> getJointPositionsStd(uint8_t control_group);

    //Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Velocities contained in Eigen::VectorXd
    VectorXd getJointVelocities(uint8_t control_group);
    void getJointVelocities(uint8_t control_group , VectorXd & output_ref);

    //Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Velocites contained in std::vector
    std::vector<double> getJointVelocitiesStd(uint8_t control_group);

    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in Eigen::VectorXd
    VectorXd getJointCurrents(uint8_t control_group);
    void getJointCurrents(uint8_t control_group , VectorXd & output_ref);

    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in std::vector
    std::vector<double> getJointCurrentsStd(uint8_t control_group);


    //For each joint
    double getJointAngle(uint8_t joint_id);

    double getJointVelocity(uint8_t joint_id);

    double getJointCurrent(uint8_t joint_id);

    double getTargetJointPosition(uint8_t joint_id , uint8_t mode_fixed = 0);

    double getTargetJointVelocity(uint8_t joint_id);

    double getTargetJointCurrent(uint8_t joint_id);

    //---------------------------------------------------------------------------------
    //Get Target Position for Arms or Body
    VectorXd getTargetPosition(uint8_t control_group, uint8_t mode_fixed = 0);
    void getTargetPosition(uint8_t control_group, VectorXd & output_ref , uint8_t mode_fixed = 0);

    //Get Target velocity for Arms or Body
    VectorXd getTargetVelocity(uint8_t control_group);
    void getTargetVelocity(uint8_t control_group, VectorXd & output_ref);

    //Get Target Current for Arms or Body
    VectorXd getTargetCurrent(uint8_t control_group);
    void getTargetCurrent(uint8_t control_group, VectorXd & output_ref );

    //Zero all the values
    void Zero();

    //-----------------------------------------------------------------


    //Commnunications--------------------------------------------------
    //Set the Control Method for an entire Control Group , i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Conrol Mode ID
    //Reutrns : void , may add error message in the fulture
    void setControlMode(uint8_t control_group ,uint8_t option);
    uint8_t getControlMode(uint8_t control_group);
    std::vector<uint8_t> getControlGroupIDs(int control_group);
    void setSubControlMode(int sub_control_group , int option , int base_id = XR1::MainBody);
    int getSubControlMode(int sub_control_group);
    void switchIKMode(bool hand_tracking_switch = false); // switch to a state to IK Mode


    void errorHandle();


    //Update the actual values for controllers
    void updatingCallback(VectorXd & JointValue, uint8_t control_group , uint8_t values_type);
    void updatingCallback(double JointValue, uint8_t JointID , uint8_t values_type);


    // Trigger the Calcualtions
    //Subjected to change
    //Argu: N/A
    //Reutrns : void , may add error message in the fulture
    void triggerCalculation( bool dynamic_grav_switch = true );

    //--------------------------------------------------------------------------------

    //Toggle the dynamic modes for left and right arm, be very careful with the options
    void setInverseDynamicsOption( uint8_t option);

    // get that option, in case the varible is changed internally
    uint8_t getInverseDynamicsOption();
    //----------------------------------------------------------------------------------



    MatrixXd getJacobian(uint8_t control_group);

//    std::vector<MatrixXd> getJacobian(std::vector<uint8_t> joint_idx_list);


    // Brutal Straight Forward Controls----------------------------------------------

    bool setTrackingPosition(uint8_t control_group , Affine3d & TargetTransformation);

    bool solveBackApproach(const Vector3d & goal_position, Vector3d & back_angles);

    bool solveHeadTracking(const Vector3d & goal_position , Vector3d & head_angles);

    void setGrippingSwitch(uint8_t control_group, bool tof);

    bool setEndEffectorPosition(uint8_t control_group , const Affine3d & transformation, double elbow_angle, double period, uint8_t base_group = XR1::Back_Y);

    void stabilizeEndEffector(uint8_t control_group , uint8_t base_id);

    bool isIKPlannerActive(uint8_t control_group);

    void getEndEffectorTransformation(uint8_t control_group, Affine3d &TransformationReference);

    double getElbowAngle(uint8_t control_group);


    void setEndEffectorIncrement(uint8_t control_group ,const Vector3d& Linear , const Vector3d& Angular);
    void setLeftArmIncrement(const Vector3d& Linear , const Vector3d& Angular);
    void setRightArmIncrement(const Vector3d& Linear , const Vector3d& Angular);

    void setEndEffectorVelocity(uint8_t control_group ,const Vector3d& Linear , const Vector3d& Angular);
    void setLeftArmVelocity(const Vector3d& Linear , const Vector3d& Angular);
    void setRightArmVelocity(const Vector3d& Linear , const Vector3d& Angular);

    void setEndEffectorForce(uint8_t control_group , const Vector3d& Linear , const Vector3d& Torque);
    void setLeftArmForce(const Vector3d& Force , const Vector3d& Torque);
    void setRightArmForce(const Vector3d& Force , const Vector3d& Torque);

    void setLeftArmIncrement(const VectorXd& twist);
    void setRightArmIncrement(const VectorXd& twist);

    void setLeftArmVelocity(const VectorXd& twist);
    void setRightArmVelocity(const VectorXd& twist);

    void setLeftArmForce(const VectorXd& twist);
    void setRightArmForce(const VectorXd& twist);


    VectorXd getLeftArmForce();

    VectorXd getLeftArmVelocity();

    VectorXd getLeftArmPosition();

    MatrixXd getLeftArmPositionMatrix();

    VectorXd getRightArmForce();

    VectorXd getRightArmVelocity();

    VectorXd getRightArmPosition();

    MatrixXd getRightArmPositionMatrix();





    //Dynamics Controls-----------------------------------------------------------------
    void updateBaseTransformation();
    bool CollisionDetection(uint8_t control_group);
    void setPeriod(uint8_t control_group, double reading_interval_in_second);


    //OmniWheels Controls-----------------------------------------------------------------
    void SetOmniWheelsVelocity(Vector3d input);
    Vector3d getOmniWheelsVelocity();
    Vector3d getOmniWheelsPosition();
    void getOmniWheelsVelocity(Vector3d & ref);
    void getTargetOmniWheelsVelocity(Vector3d & ref);
    void getOmniWheelsPosition(Vector3d & ref);
    void getBaseTransformation(uint8_t control_group ,  Affine3d & output);
    void resetOdometry();


    // Direct inputs from animation and IMU
    // MoCapCommand
    void setMoCapPosition(std::vector<double> cmds);

    // Teach Commands
    void setTeachPosition(std::vector<double> TeachData);


    // Ports for animation library
    void setState(std::vector<double> goal_configuration , int period_in_ms, int control_rate = 200);
    void setState(int joint_id , double goal_position , int period_in_ms , int control_rate = 200);
    void insertNextState(std::vector<double> pos , std::vector<double>  vel, std::vector<double> acc);
    void insertNextState(int joint_id , double pos , double vel = 0, double acc =0);
    void setNextState(int joint_id , double pos , double vel = 0, double acc =0 );
    bool isStateActive();
    bool isStateActive(int joint_id);
    bool isStateReady(int joint_id);
    bool isReady4Animation(int joint_id);
    bool isReady4Teachmotion(int joint_id);
    bool inHighFrequencyControl(int joint_id);
    void setHighFrequencyControl(int joint_id , bool option);
    std::vector<double> getNextState();
    double getNextState(int joint_id);
    void trackBothHandsWithBack(VectorXd &output_ref);
    void trackBothHandsWithHead(VectorXd &output_ref);
    void clearState();



    // Options for post collision detection
    void liftLockdown();
    bool isXR1Okay();
    uint8_t getErrorCode();


    // Some lookup maps
    std::map<uint8_t , uint8_t> ControllerIDs;
    std::map<uint8_t , std::vector<uint8_t>> ControlGroupIDs;

private:

    // two functions to read parameters
    void readParameters(string parameters_path);
    std::vector<std::vector<double> > readParameter(std::string parameter_path);

    // internal functions
    bool setEndEffectorPosition(uint8_t control_group , const Matrix3d &rotation , const Vector3d &position , const double &elbow_lift_angle , uint8_t base_group = XR1::Back_Y);
    bool setEndEffectorPosition(uint8_t control_group , const Affine3d &transformation, double elbow_lift_angle , uint8_t base_group = XR1::Back_Y);


    // Pointers to all the controllers
    std::map<uint8_t ,GenericController *> ControllerMap;
    ChainController * LeftArm;
    ChainController * RightArm;
    BackController * MainBody;
    HeadController * HeadBody;
    HandController * LeftHand;
    HandController * RightHand;
    OmniController * OmniWheels;
    DynamicMethod * DungeonMaster;
    IKplanner * IKPlanner;



    // global configs
    std::map<uint8_t, uint8_t> ControlModes;
    std::map<uint8_t , uint8_t> SubControlModes;
    uint8_t SubControlModes2ControlModes(uint8_t sub_mode);
    bool SubControlModes2HighFrequency(uint8_t sub_mode);

    std::vector<uint8_t> ArrayIDHelper(uint8_t control_group);
    int num_joint_in_chain;
    int num_joint_in_main;
    int num_joint_in_head;
    int num_joint_in_hand;
    int num_joint_friction;
    int num_joint_parameters;




    //Private function called internally
    void getState();
    void getState(int joint_id);
    void calculateStates();
    void calculateStates(int joint_id);
    void assignState();
    void assignState(int joint_id);
    void moveIKQueue2States(int joint_id);
    void moveIKTracking2States(int joint_id);
//    void moveHandTracking2States(int joint_id);
    void moveHandTracking2MainBodyStates(int joint_id);
    void moveHandTracking2HeadBodyStates(int joint_id);
    void moveHandGripping2States(int joint_id);
    void moveStable2States(int joint_id);



    //private function all calculating states
    void tinyTriPos(double &value, double & qmin , double &pt_s, double &pt_e, int & pi, int & pn, double & pd, double & pps);
    void tinyTriVel(double &value, double & qmin , int &pi , int &pn , double &pd , double &pps);
    void tinyTriAcc(double &value, double & qmin , int &pi, int &pn, double &pd, double &pps);

    // private members for calcualting states
    uint8_t XR1_State;
    std::vector<double> Qmin;
    std::vector<std::vector<double> > GeneratedCommands;
    int PlaybackIndex;
    std::vector<double> start_state;
    std::vector<double> goal_state;
    std::vector<bool> highFrequencyState;
    std::vector<std::deque<double> > tri_states;
    std::vector<std::deque<double> > tri_vels;
    std::vector<std::deque<double> > tri_accs;
    std::vector<double> temp_state;
    std::vector<double> ready_state;
    std::vector<int> poly_period_ms;
    std::vector<double> poly_period_s;
    std::map<int,bool> StableBase;
    std::map<int, Eigen::Affine3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::Affine3d> > > StableAff;
    std::map<int,double> stableElbow;
    int poly_rate;
    int poly_index;
    std::vector<int> poly_num;
    double poly_double;
    double grip_current;

    // regarding collision detection
    void copyCurrent2Target();
    void passiveLockdown();
    double breakAcceleration(double velocity , double period);
    double breakMotion(double x0 , double v0 , double a, double t);
    bool GripDetection(uint8_t joint_id);
    bool CollisionThresholding(VectorXd && ActualCurrent , VectorXd && ExpectedCurrent, std::vector<double> Thresholds, std::vector<double> StaticThreshold);
    bool ReleaseThresholding(VectorXd ActualPosition, VectorXd TargetPosition, VectorXd Thresholds);
    bool CollisionThresholding(double ActualCurrent , double ExpectedCurrent, double Threshold);
    bool ReleaseThresholding(double ActualPosition, double TargetPosition, double Threshold);

    std::vector<double> LeftArmCollisionThreshold;
    std::vector<double> RightArmCollisionThreshold;
    std::vector<double> LeftArmStaticThreshold ;
    std::vector<double> RightArmStaticThreshold;
    double LeftArmCollisionCounter;
    double RightArmCollisionCounter;
    double LeftArmCollisionCounterLimit;
    double RightArmCollisionCounterLimit;
    double HandCollisionThreshold;





    void PlaybackCallback();



    // Mute Mode temp varibles
    Quaterniond mute_qua;
    Matrix3d mute_rot;
    Vector3d mute_vec;
    Affine3d Odom2Base;
    Affine3d Base2Back;
    Affine3d Odom2Target;
    Affine3d Back2Target;


    // Body and Head Tracking Stuff
    std::map<uint8_t , Vector3d > TrackingPositionMap;
    std::map<uint8_t , Matrix3d > TrackingOrientationMap;


    // IK extra stuff
    Affine3d temp_aff3d;
    Affine3d temp_aff3d2;
    Matrix3d temp_mat3d;
    Vector3d temp_vec3d;
    Vector3d temp_vec3d2;


    // temp varibles that save time and money
    VectorXd temp_vec5d;
    VectorXd temp_vec7d;
    VectorXd temp_vec4d;
    VectorXd temp_vec3d3;

};

#endif // XR1CONTROLLERPM_H
