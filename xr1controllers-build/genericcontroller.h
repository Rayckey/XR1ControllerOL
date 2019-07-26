#ifndef GENERICCONTROLLER_H
#define GENERICCONTROLLER_H
#include "Eigen/Dense"
#include <vector>
#include "xr1define.h"
#include "xr1controllerutil.h"
#include "xr1controllercommon.h"
#include <iostream>

using namespace Eigen;
class GenericController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GenericController(uint8_t id , int num_joint);



    // Update the current values
    virtual void updateValue(VectorXd & JointValue , uint8_t value_type);
    virtual void updateValue(double JointValue, uint8_t JointID , uint8_t value_type);


    //Retrive Buffered Values from each group
    virtual VectorXd getJointPositions();
    virtual void getJointPositions(VectorXd & output_ref);
    virtual std::vector<double> getJointPositionsStd();

    virtual VectorXd getJointVelocities();
    virtual void getJointVelocities(VectorXd & output_ref);
    virtual std::vector<double> getJointVelocitiesStd();

    virtual VectorXd getJointEfforts();
    virtual void getJointEfforts(VectorXd & output_ref);
    virtual std::vector<double> getJointEffortsStd();



    //Retrive Target Values from each group
    virtual VectorXd getTargetJointPositions();
    virtual void getTargetJointPositions(VectorXd & output_ref);
    virtual std::vector<double> getTargetJointPositionsStd();

    virtual VectorXd getTargetJointVelocities();
    virtual void getTargetJointVelocities(VectorXd & output_ref);
    virtual std::vector<double> getTargetJointVelocitiesStd();

    virtual VectorXd getTargetJointAccelerations();
    virtual void getTargetJointAccelerations(VectorXd & output_ref);
    virtual std::vector<double> getTargetJointAccelerationsStd();

    virtual VectorXd getTargetJointEfforts();
    virtual void getTargetJointEfforts(VectorXd & output_ref);
    virtual std::vector<double> getTargetJointEffortsStd();

    // IK excluesive ports
    virtual VectorXd getIKJointPositions();
    virtual void getIKJointPositions(VectorXd & output_ref);
    virtual void    overwriteIKJointPositions();



    //Retrive buffered values for each joint
    virtual double getJointPosition(uint8_t joint_id);
    virtual double getJointVelocity(uint8_t joint_id);
    virtual double getJointAcceleration(uint8_t joint_id);
    virtual double getJointEffort(uint8_t joint_id);


    //Get Target Values for each joint
    virtual double getTargetJointPosition(uint8_t joint_id);
    virtual double getTargetJointVelocity(uint8_t joint_id);
    virtual double getTargetJointAcceleration(uint8_t joint_id);
    virtual double getTargetJointEffort(uint8_t joint_id);

    // Again, IK is seperated
    virtual double getIKJointPosition(uint8_t joint_id);


    // Set the update interval
    virtual void setPeriod(double reading_interval_in_second);


		

    //get End Effector information
    virtual VectorXd getEndEffectorVelocity(uint8_t frame_reference);

    virtual void getEndEffectorVelocity(VectorXd &output_ref , uint8_t frame_reference);

    virtual void getKinematics(uint8_t joint_id , Affine3d & output_ref);

    virtual void getTransformation(uint8_t joint_id , Affine3d & output_ref);

    virtual Affine3d getTransformation(uint8_t JointID);

    virtual void getEndEffectorTransformation(Affine3d & transformationReference);

    virtual double getElbowAngle();

    virtual double getTargetElbowAngle();


    // Returns the last calculated Jacobian matrix
    virtual MatrixXd getJacobian(uint8_t id, uint8_t reference_frame);

    virtual void getJacobian(uint8_t id, MatrixXd & output_ref , uint8_t reference_frame = XR1::SpatialFrame);


    // get end effector target, will over write previous target
    virtual void setEndEffectorIncrement(const Vector3d& Linear , const Vector3d& Angular , uint8_t frame_reference = XR1::SpatialFrame);

    virtual void setEndEffectorVelocity(const Vector3d& Linear , const Vector3d& Angular , uint8_t frame_reference = XR1::SpatialFrame);

    virtual void setEndEffectorEffort(const Vector3d& Force , const Vector3d& Torque , uint8_t frame_reference = XR1::SpatialFrame);

    virtual void setEndEffectorIncrement(const VectorXd& twist , uint8_t frame_reference = XR1::SpatialFrame);

    virtual void setEndEffectorVelocity(const VectorXd& twist , uint8_t frame_reference = XR1::SpatialFrame);

    virtual void setEndEffectorEffort(const VectorXd& twist, uint8_t frame_reference = XR1::SpatialFrame);

    // End effector target but with target

    virtual bool setEndEffectorTransformation(const Affine3d &transformation, double optional_1 );


    // Sometimes the base will have transformation as well
//    virtual void initializeBaseTransformation();

    virtual void setBaseTransformation(Affine3d & transformationReference);

    virtual void getBaseTransformation(Affine3d & transformationReference);

    //trigger during mode change
    virtual void modeChange();

    // Buffered values
    VectorXd Joint_Positions;
    VectorXd Joint_Velocities;
    VectorXd Joint_Acceleration;
    VectorXd Joint_Efforts;
    VectorXd Target_Joint_Positions;
    VectorXd Target_Joint_Velocities;
    VectorXd Target_Joint_Acceleration;
    VectorXd Target_Joint_Efforts;

    // IK values are seperated
    VectorXd IK_Joint_Positions;
    VectorXd IK_Joint_Velocities;
    VectorXd IK_Joint_Acceleration;

    uint8_t DynamicsOption;


    //Kinematics stuff
    std::vector<Affine3d> m_Kinematics;
    std::vector<Affine3d> m_Transformations;
    Affine3d BaseTransformation;
    Affine3d ToolTransformation;
    Affine3d TargetTransformation;


    // temp variables
    Matrix3d temp_rot;
    Vector3d temp_vec;
    Matrix3d temp_hat;
    Affine3d temp_afn;


protected:

    // important control group infomrations
    uint8_t Begin_ID;
    int NUM_OF_JOINTS;
    double period;


};

#endif // GENERICCONTROLLER_H
