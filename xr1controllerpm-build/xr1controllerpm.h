#ifndef XR1CONTROLLERPM_H
#define XR1CONTROLLERPM_H


#include "genericcontroller.h"
#include "Eigen/Dense"
#include "chaincontroller.h"
#include "handcontroller.h"
#include "omnicontroller.h"
#include "dynamicmethod.h"
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


    // MoCapCommand
    void setMoCapPosition(std::vector<double> cmds);


    // Mute Commands
    void setMutePosition(std::vector<double> MuteData);



    //Simple Joint Controls--------------------------------------

    //Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(uint8_t control_group ,VectorXd JA);

    //Set the Target Joint Velocity for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(uint8_t control_group ,VectorXd JV);


    //Set the Target Joint Acceleration for entire control group, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointAcceleration(uint8_t control_group ,VectorXd JACC);

    //Set the Target Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Target Current
    //Reutrns : void , may add error message in the fulture
    void setJointCurrent(uint8_t control_group , VectorXd JC);


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

    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in std::vector
    std::vector<double> getJointCurrentsStd(uint8_t control_group);


    //For each joint
    double getJointAngle(uint8_t joint_id);

    double getJointVelocity(uint8_t joint_id);

    double getJointCurrent(uint8_t joint_id);

    //---------------------------------------------------------------------------------
    //Get Target Position for Arms or Body
    VectorXd getTargetPosition(uint8_t control_group);

    //Get Target velocity for Arms or Body
    VectorXd getTargetVelocity(uint8_t control_group);

    //Get Target Current for Arms or Body
    VectorXd getTargetCurrent(uint8_t control_group);

    double getTargetJointPosition(uint8_t joint_id);

    double getTargetJointVelocity(uint8_t joint_id);

    double getTargetJointCurrent(uint8_t joint_id);


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



    //Set Control Mode for Entire XR1, which only has two mode: direct and drive
    //When in drive mode, it will take over
    void setMetaMode(uint8_t option);
    uint8_t getMetaMode();


    void errorHandle();


    //Update the actual values for controllers
    void updatingCallback(VectorXd JointValue, uint8_t control_group , uint8_t values_type);
    void updatingCallback(double JointValue, uint8_t JointID , uint8_t values_type);


    // Trigger the Calcualtions
    //Subjected to change
    //Argu: N/A
    //Reutrns : void , may add error message in the fulture
    void triggerCalculation();

    //--------------------------------------------------------------------------------

    //Toggle the dynamic modes for left and right arm, be very careful with the options
    void setInverseDynamicsOption( uint8_t option);
    //----------------------------------------------------------------------------------



    MatrixXd getJacobian(uint8_t control_group);

//    std::vector<MatrixXd> getJacobian(std::vector<uint8_t> joint_idx_list);


    // Brutal Straight Forward Controls----------------------------------------------


    bool setEndEffectorPosition(uint8_t control_group , const Matrix3d &rotation , const Vector3d &position , const double &elbow_lift_angle);

    bool setEndEffectorPosition(uint8_t control_group , const Affine3d &transformation, double elbow_lift_angle);

    void getEndEfftorTransformation(uint8_t control_group, Affine3d &TransformationReference, bool IK = true);


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



    //Tilt Control----------------------------------------------------------------

    void TiltCompensation();

//    Vector3d TiltCompensation(Quaterniond BaseRotation , Vector3d BaseAcceleration);

    void tiltCallback(double x , double y , double z);

    void tiltCallback(double w , double x , double y , double z);

    void tiltInit();

    Vector3d getBaseAcc();

    //Dynamics Controls-----------------------------------------------------------------

    void updateBaseTransformation();

    bool CollisionDetection(uint8_t control_group);

    void setPeriod(uint8_t control_group, double reading_interval_in_second);


    //OmniWheels Controls-----------------------------------------------------------------
    void SetOmniWheelsVelocity(Vector3d input);
    Vector3d getOmniWheelsVelocity();
    Vector3d getOmniWheelsPosition();
    void getBaseTransformation(uint8_t control_group ,  Affine3d & output);
    void resetOdometry();





    // Ports for animation library
    void setState(std::vector<double> goal_configuration , int period_in_ms, int control_rate = 200);
    std::vector<double> getNextState();
    void clearState();







    // Options for post collision detection
    void liftLockdown();
    bool isXR1Okay();
    uint8_t getErrorCode();




    // Options for EFF position planning
    void setEFFgoal(uint8_t control_group, Vector3d eff_position , Matrix3d eff_orientation , int period_in_ms , int control_rate = 200);
    void getNextArmState();

private:


    // Pointers to all the controllers
    std::map<uint8_t ,GenericController *> ControllerMap;
    std::map<uint8_t , uint8_t> ControllerIDs;
    ChainController * LeftArm;
    ChainController * RightArm;
    ChainController * MainBody;
    HandController * LeftHand;
    HandController * RightHand;
    OmniController * OmniWheels;
    DynamicMethod * DungeonMaster;


    // global configs
    std::map<uint8_t, uint8_t> ControlModes;
    uint8_t MetaMode;
    std::vector<uint8_t> ArrayIDHelper(uint8_t control_group);
    int num_joint_in_chain;
    int num_joint_in_hand;
    int num_joint_friction;
    int num_joint_parameters;




    //Private function called internally
    void getState();
    void calculateStates();
    void assignState();
    void readParameters(string parameters_path);
    std::vector<std::vector<double> > readParameter(std::string parameter_path);


    //private function all calculating states
    void tinyTriPos(double &value, double & qmin , double &pt_s, double &pt_e);
    void tinyTriVel(double &value, double & qmin );
    void tinyTriAcc(double &value, double & qmin );
    void solveTri(double & qmin , double & pt_s, double & pt_e, double &  period);

    // private members for calcualting states
    uint8_t XR1_State;
    std::vector<double> Qmin;
    std::vector<std::vector<double> > GeneratedCommands;
    int PlaybackIndex;
    std::vector<double> start_state;
    std::vector<double> goal_state;
    std::deque<std::vector<double> > tri_states;
    std::deque<std::vector<double> > tri_vels;
    std::deque<std::vector<double> > tri_accs;
    std::vector<double> temp_state;
    std::vector<double> ready_state;
    int poly_period_ms;
    double poly_period_s;
    int poly_rate;
    int poly_index;
    int poly_num;
    double poly_double;
    double grip_current;

    // regarding collision detection
    void employLockdown();
    void copyCurrent2Target();
    void passiveLockdown();
    double breakAcceleration(double velocity , double period);
    double breakMotion(double x0 , double v0 , double a, double t);
    bool GripDetection(uint8_t joint_id);
    bool CollisionThresholding(VectorXd ActualCurrent , VectorXd ExpectedCurrent, std::vector<double> Thresholds, std::vector<double> StaticThreshold);
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


    // tilt control members
    std::vector<double> driveCmd;
    std::vector<Vector3d> rawAccVec;
    Quaterniond rawQua;
    Quaterniond initQua;
    Vector3d initAcc;
    Vector3d tmpAcc;
    Vector3d rawAcc;
    Vector3d currAcc;
    Matrix3d rawTilt;
    Matrix3d initTilt;
    Matrix3d currTilt;
    Matrix3d tempTilt;
    Vector3d actuAcc;
    Vector3d actuEul;
    Vector3d hatAcc;
    Vector3d innAcc;
    Vector3d noiAcc;
    Vector3d no2Acc;
    Vector3d kvcAcc;
    Vector3d ganAcc;
    Vector3d cvcAcc;
    void accKalman(double x , double y , double z);
    void TiltCalcualtion(Matrix3d & rotation_of_acc);
    void assignAcc2Joint();



    void PlaybackCallback();



    // Mute Mode temp varibles
    Quaterniond mute_qua;
    Matrix3d mute_rot;
    Vector3d mute_vec;
    Affine3d Odom2Base;
    Affine3d Base2Back;
    Affine3d Odom2Target;
    Affine3d Back2Target;


    // ik planning stuff
    Vector3d EFFgoalPosition;
    Matrix3d EFFgoalOrientation;


};

#endif // XR1CONTROLLERPM_H
