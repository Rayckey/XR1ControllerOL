#ifndef XR1CONTROLLER_H
#define XR1CONTROLLER_H

#include "Eigen/Dense"
#include <vector>
#include "xr1define.h"
#include "xr1controllerpm.h"
#include <map>

class XR1Controller
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    XR1Controller(std::string path);
    bool isXR1Okay(); //Error Checks
    uint8_t getErrorCode();


    //-----------------------------------------------------------------


    //--------Joint Control----------------------------------


    //Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(uint8_t control_group , VectorXd & JA);


    //Set the Target Joint Positions for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(uint8_t joint_idx ,   double JA);


    //Set the Target Joint Velocity for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(uint8_t control_group , VectorXd & JV);


    //Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(uint8_t joint_idx ,   double JV);



    //Set the Target Joint Accelration for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Target Current
    //Reutrns : void , may add error message in the fulture
    void setJointAcceleration(uint8_t control_group , VectorXd &JA);


    //Set the Target Joint Accelration for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointAcceleration(uint8_t joint_idx ,   double JA);



    //Set the Target Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Target Current
    //Reutrns : void , may add error message in the fulture
    void setJointEffort(uint8_t control_group , VectorXd & JC);


    //Set the Target Joint Currents for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointEffort(uint8_t joint_idx ,   double JC);


    //Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : JointAngles contained in Eigen::VectorXd
    VectorXd getJointPositions(uint8_t control_group , bool vanilla = false);
    void getJointPositions(uint8_t control_group , VectorXd &output_ref , bool vanilla = false);


    //Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : JointAngles contained in std::vector
    std::vector<double> getJointPositionsStd(uint8_t control_group);


    //Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Velocities contained in Eigen::VectorXd
    VectorXd getJointVelocities(uint8_t control_group , bool vanilla = false);
    void getJointVelocities(uint8_t control_group , VectorXd & output_ref , bool vanilla = false);


    //Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Velocites contained in std::vector
    std::vector<double> getJointVelocitiesStd(uint8_t control_group);


    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in Eigen::VectorXd
    VectorXd getJointEfforts(uint8_t control_group , bool vanilla = false);
    void getJointEfforts(uint8_t control_group , VectorXd & output_ref , bool vanilla = false);


    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in std::vector
    std::vector<double> getJointEffortsStd(uint8_t control_group);



    double getTargetJointPosition(uint8_t joint_id , bool vanilla = false);

    double getTargetJointVelocity(uint8_t joint_id , bool vanilla = false);

    double getTargetJointEffort(uint8_t joint_id , bool vanilla = false);

    double getTargetJointAcceleration(uint8_t joint_id);

    double getJointPosition(uint8_t joint_id , bool vanilla = false);

    double getJointVelocity(uint8_t joint_id , bool vanilla = false);

    double getJointEffort(uint8_t joint_id , bool vanilla = false);

    double getJointAcceleration(uint8_t joint_id);

    //Set the Control Method for an entire Control Group , i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Conrol Mode ID
    //Reutrns : void , may add error message in the fulture
    void setControlMode(uint8_t control_group , uint8_t option);
    uint8_t getControlMode(uint8_t control_group);
    std::vector<uint8_t> getControlGroupIDs(int control_group);
    void setSubControlMode(int sub_control_group , int option , int base_id = XR1::MainBody);
    int getSubControlMode(int sub_control_group);


    //Get the last calculated Jacobian from XR1Controller
    //Used in XR1Controller
    //Argu: Joint ID (Only From LeftShoulderX to RightWristX)
    //Reutrns : A 6x7 Eigen::MatrixXd of Jacobians or A 6x7 Eigen::MatrixXd of zeros should an error occur
    MatrixXd getJacobian(uint8_t joint_idx);


    //Get the last calculated Jacobian from XR1Controller for several joints
    //Used in XR1Controller
    //Argu: Vector of Joint ID (Only From LeftShoulderX to RightWristX)
    //Reutrns : A 6x7 Eigen::MatrixXd of Jacobians or A 6x7 Eigen::MatrixXd of zeros should an error occur
//    std::vector<MatrixXd> getJacobian(std::vector<uint8_t> joint_idx_list);


    //-------------------End Effector (Wrist) Control ---------------------------



    bool setTrackingPosition(uint8_t control_group , Affine3d & TargetTransformation);

    void setGrippingSwitch(uint8_t control_group, bool tof);  

    bool setEndEffectorTransformation(uint8_t control_group , const Affine3d & transformation, double elbow_angle, double period , uint8_t base_group = XR1::Back_Y);

    bool setEndEffectorTransformation(uint8_t control_group , const Vector3d & position_1 , const Vector3d & position_2 , const Affine3d &transformation_target, double optional_1 = 777 , double optional_2 = 1);

    void stabilizeEndEffector(uint8_t control_group , uint8_t base_id );

    bool setGrippingSwitch();

    bool isIKPlannerActive(uint8_t control_group);

    void getEndEffectorTransformation(uint8_t control_group, Affine3d & TransformationReference);

    double getElbowAngle(uint8_t control_group);

    void setEndEffectorIncrement(uint8_t control_group ,const Vector3d& Linear , const Vector3d& Angular);

    //Move the End Effector of LeftArm for a SMALL distance
    //Used in XR1Controller
    //Argu: Linear increment contained in Eigen::Vector3d , Angular increment contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setLeftArmIncrement(const Vector3d& Linear , const Vector3d& Angular);
    void setLeftArmIncrement(const VectorXd& twist);

    //Move the End Effector of RightArm for a SMALL distance
    //Used in XR1Controller
    //Argu: Linear increment contained in Eigen::Vector3d , Angular increment contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setRightArmIncrement(const Vector3d& Linear , const Vector3d& Angular);
    void setRightArmIncrement(const VectorXd& twist);



    void setEndEffectorVelocity(uint8_t control_group ,const Vector3d& Linear , const Vector3d& Angular);
    //Set the velocity of the End Effector of LeftArm,
    //Used in XR1Controller , Needs to be refreshed every simulation step
    //Argu: Linear velocity contained in Eigen::Vector3d , Angular velocity (Euler Angles in the order XYZ) contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setLeftArmVelocity(const Vector3d& Linear , const Vector3d& Angular);
    void setLeftArmVelocity(const VectorXd& twist);

    //Set the velocity of the End Effector of RightArm,
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: Linear velocity contained in Eigen::Vector3d , Angular velocity (Euler Angles in the order XYZ) contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setRightArmVelocity(const Vector3d& Linear , const Vector3d& Angular);
    void setRightArmVelocity(const VectorXd& twist);

    //Set the velocity of the End Effector of Arms , with the addition of dynamic compensation
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: Force contained in Eigen::Vector3d , Torque contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setEndEffectorForce(uint8_t control_group , const Vector3d& Linear , const Vector3d& Torque);


    //get the net force on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 forces and 3 torques , may add error message in the fulture
    VectorXd getLeftArmForce();

    //get the velocity on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 linear velocity and 3 angular velocity , may add error message in the fulture
    VectorXd getLeftArmVelocity() ;

    //get the position on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 linear position and 3 angular position , may add error message in the fulture
    VectorXd getLeftArmPosition() ;

    //get the Homogenous transformation End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : A homogenous transformation (4x4)
    MatrixXd getLeftArmPositionMatrix() ;

    //get the net force on End Effector of the right arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 forces and 3 torques , may add error message in the fulture
    VectorXd getRightArmForce() ;

    //get the velocity on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 linear velocity and 3 angular velocity , may add error message in the fulture
    VectorXd getRightArmVelocity() ;

    //get the position on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 linear position and 3 angular position , may add error message in the fulture
    VectorXd getRightArmPosition() ;

    //get the Homogenous transformation End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : A homogenous transformation (4x4)
    MatrixXd getRightArmPositionMatrix() ;



    //Convert output from XR1IMU to Motor Joint angles
    void setMoCapPosition(std::vector<double> IMU_output);


    //Convert teach data into action
    void setTeachPosition(std::vector<double> TeachData);


    //Convert output from Action files to Motor Joint Angles;
    void setActionPosition(std::vector<double> Action_output);


    //Convert output from Data files to Motor Joint Angles;
    void setState(std::vector<double> goal_configuration , int period_in_ms, int control_rate = 200);
    void setAnimationState(std::vector<double> goal_configuration , int period_in_ms, int control_rate = 200);
    std::vector<double> getNextState(bool vanilla = false);
    bool isStateActive();
    void state2Actuator(std::vector<double> & some_state);
    void setState(int joint_id , double goal_position , int period_in_ms , int control_rate = 200);
    double getNextState(int joint_id);
    bool isStateActive(int joint_id);
    bool isReady4Animation(int joint_id);
    bool isReady4Teachmotion(int joint_id);
    void insertNextState(std::vector<double> pos , std::vector<double>  vel, std::vector<double> acc);
    void insertNextState(int joint_id , double pos , double vel = 0 , double acc = 0);
    void setNextState(int joint_id , double pos , double vel = 0, double acc = 0 , int mode_fix = XR1::AnimationMode);
    bool inHighFrequencyControl(int joint_id);
    void setHighFrequencyControl(int joint_id , bool option);


    //Trun ON/OFF the dynamic compensation of UpperBody, only affects Current Mode
    //Used in XR1Controller.
    //Argu: option true/false,
    //Reutrns : void , may add error message in the fulture
    void setInverseDynamicsOption(uint8_t option);


    // get that same option in case it was changed internally
    uint8_t getInverseDynamicsOption();


    //Get Target Position for Arms or Body
    VectorXd getTargetPosition(uint8_t control_group , bool vanilla = false);
    void getTargetPosition(uint8_t control_group , VectorXd & output_ref , bool vanilla = false);

    //Get Target velocity for Arms or Body
    VectorXd getTargetVelocity(uint8_t control_group , bool vanilla = false);
    void getTargetVelocity(uint8_t control_group , VectorXd & output_ref, bool vanilla = false);

    //Get Target Current for Arms or Body
    VectorXd getTargetEffort(uint8_t control_group , bool vanilla = false);
    void getTargetEffort(uint8_t control_group , VectorXd & output_ref , bool vanilla = false);


    // Trigger the Calcualtions
    //Subjected to change
    //Argu: N/A
    //Reutrns : void , may add error message in the fulture
    void triggerCalculation(bool dynamic_grav_switch = true);

    void updatingCallback(uint8_t id , uint8_t attrId , double value);

    void updatingCallback(uint8_t control_group, uint8_t attrId , VectorXd grouped_value);

    double getActuatorRatio(uint8_t id);


    // Check Collision Detections
    bool checkCollision();
    bool checkCollision(uint8_t control_group);



    void SetOmniWheelsVelocity(Vector3d input);
    Vector3d getOmniWheelsVelocity();
    Vector3d getOmniWheelsPosition();

    void getOmniWheelsTargetVelocity(Vector3d & ref);
    void getOmniWheelsVelocity(Vector3d & ref);
    void getOmniWheelsPosition(Vector3d & ref);


    void getBaseTransformation(uint8_t control_group , Affine3d & output);
    void resetOdometry();

    void setPeriod(uint8_t control_group , double period);

    void tiltCallback(double x , double y , double z);

    void tiltCallback(double w , double x , double y , double z);

    void tiltInit();

    Vector3d getBaseAcc();

    void liftLockdown();

    void employLockdown();

    void clearStates();

    void switchIKMode(bool hand_tracking_switch = false);

private:

    XR1ControllerPM * XR1_ptr;

    double Position2Actuator(uint8_t id , double value);

    double Velocity2Actuator(uint8_t id , double value);

    double Current2Actuator(uint8_t id , double value);

    double Current2Efforts(uint8_t id ,double value);

    double Efforts2Currents(uint8_t id ,double value);


    double Actuator2Position(uint8_t id , double value);

    double Actuator2Velocity(uint8_t id , double value);

//    double Actuator2Current(uint8_t id , double value);



    // wrist stuff ---------------------------------------------------------------------------
    void WristReadingHelper(uint8_t mode);

    void WristCommandHelper(uint8_t mode ,  VectorXd& input);

    void WristCommandHelper(uint8_t mode ,  double & wrist_x_input , double & wrist_y_input);

    void Wrist2Virtual(double & wrist_x_input , double & wrist_y_input);

    void Virtual2Wrist(double & wrist_x_input , double & wrist_y_input);

    void Wrist2Virtual(VectorXd& input);

    void Virtual2Wrist(VectorXd& input);
    // ---------------------------------------------------------------------------------------

    std::map<uint8_t , double> WristPositions;

    std::map<uint8_t , double> WristVelocities;

    std::map<uint8_t , double> WristCurrents;

    std::map<uint8_t , bool> WristPositionSwitch;

    std::map<uint8_t , bool> WristVelocitySwitch;

    std::map<uint8_t , bool> WristCurrentSwitch;

    std::map<uint8_t , bool> ActuatorType;



    // quick control group lookup
    std::map<uint8_t, std::vector<uint8_t> > control_group_map;
    std::vector<uint8_t> control_group_flags;
    std::map<uint8_t , uint8_t> m_control_group_lookup;

    // actuator current to toruqe variables
    std::vector<double> P1s;
    std::vector<double> P2s;
    std::vector<double> P3s;

//    std::vector<double> driveCmd;



    int num_joint_in_chain;

    int PositioningInterval;


    std::vector<double> sitting_position;



    // temp values
    VectorXd temp_vec3d;
    VectorXd temp_vec7d;
    VectorXd temp_vec5d;



    // Legacy

//    bool setEndEffectorPosition(uint8_t control_group , const Affine3d &transformation, double elbow_lift_angle = 8 , uint8_t base_group = XR1::Back_Y);
//    bool setEndEffectorPosition(uint8_t control_group , const Matrix3d &rotation , const Vector3d &position , double elbow_lift_angle = 8 , uint8_t base_group = XR1::Back_Y);


};

#endif // XR1CONTROLLER_H
