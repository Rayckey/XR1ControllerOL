#ifndef GENERICCONTROLLER_H
#define GENERICCONTROLLER_H
#include "Eigen/Dense"
#include <vector>
#include "xr1define.h"
#include "xr1controllerutil.h"
#include <iostream>

using namespace Eigen;
class GenericController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GenericController(uint8_t id , int num_joint);

    virtual void updateValue(VectorXd JointValue , uint8_t value_type);

    virtual void updateValue(double JointValue, uint8_t JointID , uint8_t value_type);

    //Retrive Current Values from each group
    virtual VectorXd getJointAngles();

    virtual std::vector<double> getJointAnglesStd();

    virtual VectorXd getJointVelocities();

    virtual std::vector<double> getJointVelocitiesStd();

    virtual VectorXd getTargetJointAccelerations();

    virtual std::vector<double> getTargetJointAccelerationsStd();

    virtual VectorXd getJointCurrents();

    virtual std::vector<double> getJointCurrentsStd();



    //Retrive Target Values from each group
    virtual VectorXd getTargetJointAngles();

    virtual VectorXd getIKJointAngles();

    virtual void    overwriteIKJointAngles();

    virtual std::vector<double> getTargetJointAnglesStd();

    virtual VectorXd getTargetJointVelocities();

    virtual std::vector<double> getTargetJointVelocitiesStd();

    virtual VectorXd getTargetJointCurrents();

    virtual std::vector<double> getTargetJointCurrentsStd();



    //For each finger
    virtual double getJointAngle(uint8_t joint_id);

    virtual double getJointVelocity(uint8_t joint_id);

    virtual double getJointAcceleration(uint8_t joint_id);

    virtual double getJointCurrent(uint8_t joint_id);


    //Get Target Values
    virtual double getTargetJointAngle(uint8_t joint_id);

    virtual double getIKJointAngle(uint8_t joint_id);


    virtual double getTargetJointVelocity(uint8_t joint_id);

    virtual double getTargetJointCurrent(uint8_t joint_id);


    //Transform 3d vector into Skew Matrix



    virtual void setPeriod(double reading_interval_in_second);


		



    //Dumb empty virtual functions
    virtual VectorXd getEFFVelocity();

    virtual VectorXd getEFFPosition();

    virtual MatrixXd getEFFPositionMatrix();

    virtual void getEndEffectorTransformation(Affine3d & transformationReference);

    virtual double getElbowAngle();

//    virtual void getBaseTransformation(Affine3d & input);

    // Update the the base
//    void updateBaseTransformation(Matrix3d BaseT);

    // Returns the last calculated Jacobian matrix
    virtual MatrixXd getJacobian(uint8_t id);

    // Return the last calculated end effector Transformation
    virtual MatrixXd getTransformation(uint8_t JointID);


    virtual void setEFFIncrement(const Vector3d& Linear , const Vector3d& Angular);

    virtual void setEFFVelocity(const Vector3d& Linear , const Vector3d& Angular);

    virtual void setEFFCurrent(const Vector3d& Force , const Vector3d& Torque);

    virtual void setEFFIncrement(const VectorXd& twist);

    virtual void setEFFVelocity(const VectorXd& twist);

    virtual void setEFFCurrent(const VectorXd& twist);

//    virtual bool setEFFPosition(const VectorXd& twist , const double &elbow_lift_angle);

    virtual bool setEFFPosition(const Matrix3d &rotation , const Vector3d &position , const double elbow_lift_angle);

    virtual bool setEFFPosition(const Affine3d &transformation, double elbow_lift_angle);


    virtual void modeChange();

    // Free for grabbing values
    VectorXd Joint_Angles;
    VectorXd Joint_Velocities;
    VectorXd Joint_Acceleration;
    VectorXd Joint_Currents;
    VectorXd Target_Joint_Angles;
    VectorXd Target_Joint_Velocities;
    VectorXd Target_Joint_Acceleration;

    VectorXd IK_Joint_Angles;
    VectorXd IK_Joint_Velocities;
    VectorXd IK_Joint_Acceleration;

    uint8_t DynamicsOption;
    double ElbowAngle;

protected:

    uint8_t Begin_ID;
    int NUM_OF_JOINTS;
    double period;

    VectorXd Target_Joint_Currents;




};

#endif // GENERICCONTROLLER_H
