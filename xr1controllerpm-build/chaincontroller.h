#ifndef CHAINCONTROLLER_H
#define CHAINCONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>
#include "IKsolver.h"
#include <iostream>

using namespace Eigen;

class ChainController: public GenericController
{
public:
    ChainController(MatrixXd DH_input, uint8_t id , int num_joint);

    VectorXd getTargetJointCurrents();

    std::vector<double> getTargetJointCurrentsStd();

    //For each joint

    double getTargetJointCurrent(uint8_t joint_id);



    VectorXd getEFFVelocity();

    VectorXd getEFFPosition();

    MatrixXd getEFFPositionMatrix();

    void getEndEfftorTransformation(Affine3d & transformationReference);

    Vector3d Matrix2XYZ(Matrix3d BaseRotation);

    // Update the the base
//    void updateBaseTransformation(Matrix3d BaseT);

    // Returns the last calculated Jacobian matrix
    MatrixXd getJacobian(uint8_t id);

    // Return the last calculated end effector Transformation
    MatrixXd getTransformation(uint8_t JointID);



    void triggerCalculationPass();

    void setEFFIncrement(const Vector3d& Linear , const Vector3d& Angular);

    void setEFFVelocity(const Vector3d& Linear , const Vector3d& Angular);

    void setEFFCurrent(const Vector3d& Force , const Vector3d& Torque);

    void setEFFIncrement(const VectorXd& twist);

    void setEFFVelocity(const VectorXd& twist);

    void setEFFCurrent(const VectorXd& twist);

//    bool setEFFPosition(const VectorXd& twist , const double &elbow_lift_angle);


    // Use this for points that are VERY CLOSE! it sets the target position straight up;
    bool setEFFPosition(const Matrix3d &rotation , const Vector3d &position , double elbow_lift_angle);

    bool setEFFPosition(const Affine3d &transformation, double elbow_lift_angle);



    void T_DH(double d , double offset , double alpha , double ad , double theta);

    void T_MDH(MatrixXd &temp_trans, double alpha , double ad , double d , double offset , double theta);

    MatrixXd Adjoint(MatrixXd & T);

    MatrixXd invAdjoint(MatrixXd & T);

    MatrixXd EFF2BaseForceAdjoint(MatrixXd & T);


    void EulerXYZ(double x , double y , double z , Matrix3d &input) ;

    void EulerZYX(double x , double y , double z , Matrix3d &input) ;

    Matrix3d EulerXYZ(double x , double y , double z ) ;

    Matrix3d EulerZYX(double x , double y , double z ) ;

    MatrixXd pinv(MatrixXd input);

    MatrixXd twist2trans(VectorXd twist);

    void clearResults();


    VectorXd Dynamic_Compensation;


    uint8_t Begin_ID;

private:

    IKsolver * scott_the_solver;

    double shoulder_angle_offset;

    double la1;
    double la2;
    double la3;
    double la4;
    double la5;


    std::vector<MatrixXd> Jacobians;

    std::vector<MatrixXd> m_T_array;
    std::vector<MatrixXd> m_Ti_array;


    MatrixXd DH_parameters;

    VectorXd d ;
    VectorXd ad ;
    VectorXd alpha ;
    VectorXd offset;


    // Saves Jacobian matrix as a the member variable
    void Jacobeans(VectorXd Joint_Angles);


    // Saves individual transformation in the transformation collection
    void Transformation(VectorXd Joint_Angles);





    Matrix3d BaseRotationTransposed;


    int NUM_PARA;

    Vector3d grav;

    double g;
    double PI;



protected:


    // For Transforms
    double costheta;
    double sintheta;
    double sinalpha;
    double cosalpha;

    // Jacobeans
    Vector3d v;
    Vector3d w_j;
    Vector3d z;

    Vector3d temp_v_1;
    Vector3d temp_v_2;


    //Transformation

    MatrixXd Trans;

    MatrixXd Temp_Trans;

};

#endif // CHAINCONTROLLER_H
