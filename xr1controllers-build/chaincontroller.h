#ifndef CHAINCONTROLLER_H
#define CHAINCONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>
#include "wrappedarmiksolver.h"
#include <iostream>
#include "dlib/optimization.h"

using namespace Eigen;

class ChainController: public GenericController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ChainController(MatrixXd mdh_input, uint8_t group_id, uint8_t num_joint,  double a1 , double a2 , double a3, double arm_angle, double upper_limit, double lower_limit);

    typedef dlib::matrix<double, 0, 1> column_vector;



    void triggerCalculation();



    // End Effector Controls
    void setEndEffectorIncrement(const Vector3d& Linear , const Vector3d& Angular , uint8_t frame_reference = XR1::SpatialFrame);

    void setEndEffectorVelocity(const Vector3d& Linear , const Vector3d& Angular , uint8_t frame_reference = XR1::SpatialFrame);

    void setEndEffectorEffort(const Vector3d& Force , const Vector3d& Torque , uint8_t frame_reference = XR1::SpatialFrame);

    void setEndEffectorIncrement(const VectorXd& twist , uint8_t frame_reference = XR1::SpatialFrame);

    void setEndEffectorVelocity(const VectorXd& twist , uint8_t frame_reference = XR1::SpatialFrame);

    void setEndEffectorEffort(const VectorXd& twist, uint8_t frame_reference = XR1::SpatialFrame);


    double getElbowAngle();
    double getTargetElbowAngle();

    VectorXd getEndEffectorVelocity(uint8_t frame_reference = XR1::SpatialFrame);

    void getEndEffectorVelocity(VectorXd & output_ref , uint8_t frame_reference = XR1::SpatialFrame);


    // Use this for points that are VERY CLOSE! it sets the target position straight up;
    bool setEndEffectorTransformation(const Affine3d &target_transformation, double target_elbow_angle = 777 );

    void correctIKJointAngles();

    double  computeElbowCost(const column_vector &target_elbow_angle);

    // Update the the base
//    void updateBaseTransformation(Matrix3d BaseT);

    // Returns the last calculated Jacobian matrix
    MatrixXd getJacobian(uint8_t id , uint8_t reference_frame = XR1::SpatialFrame);
    void getJacobian(uint8_t id , MatrixXd & jac , uint8_t reference_frame = XR1::SpatialFrame);


    double ElbowAngle;
    double TargetElbowAngle;



private:


    //pointer to ik solver
    WrappedArmIKSolver * WrappedIKSolver_ptr;


    MatrixXd m_MDHParameters;

    VectorXd m_gamma ;
    VectorXd m_b     ;
    VectorXd m_alpha ;
    VectorXd m_d     ;
    VectorXd m_r     ;

    //Buffer values
    std::vector<MatrixXd> m_BodyJacobians;
    std::vector<MatrixXd> m_SpatialJacobians;


    column_vector UpperLimit;
    column_vector LowerLimit;


    // Temp values
    Affine3d ArmPit;
    Affine3d tempAffineA;
    Affine3d tempAffineB;



};

#endif // CHAINCONTROLLER_H
