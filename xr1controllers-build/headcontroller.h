#ifndef HEADCONTROLLER_H
#define HEADCONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>
#include <iostream>

using namespace Eigen;

class HeadController: public GenericController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    HeadController(MatrixXd DH_input, uint8_t id , int num_joint);

    void triggerCalculationPass();

    //Over load some functions
    VectorXd getTargetJointCurrents();
    void getTargetJointCurrents(VectorXd & output_ref);
    std::vector<double> getTargetJointCurrentsStd();
    double getTargetJointCurrent(uint8_t joint_id);


    // End Effector Controls
    void getEndEffectorTransformation(Affine3d & transformationReference);
    void getBaseTransformation(Affine3d &transformationReference);


    // MDH methods
    void T_MDH(MatrixXd &temp_trans, double alpha , double ad , double d , double offset , double theta);


    void clearResults();

    VectorXd Dynamic_Compensation;
    uint8_t Begin_ID;


private:

    // Saves individual transformation in the transformation collection
    void Transformation();


    // DH paramters

    MatrixXd DH_parameters;
    VectorXd d ;
    VectorXd ad ;
    VectorXd alpha ;
    VectorXd offset;

    //Buffer values
    std::vector<MatrixXd> m_T_array;
    std::vector<MatrixXd> m_Ti_array;


    Affine3d BaseTransformation;
    Affine3d NeckTransformation; // I hate DH



    //regular consts
    Vector3d grav;
    double g;


protected:


    // Temp values

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

    Affine3d ArmPit;
    Affine3d TransferedGoal;
    Affine3d tempAffine;

    //Adjoint
    Matrix3d temp_rot;
    Vector3d temp_vec;
    Matrix3d temp_hat;
    Affine3d temp_afn;
};

#endif // HeadController_H
