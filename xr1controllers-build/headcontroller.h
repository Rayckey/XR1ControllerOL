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
    HeadController(MatrixXd mdh_input, uint8_t id , int num_joint , Affine3d & eye_transform);

    void triggerCalculation();


    // End Effector Controls
    bool setEndEffectorTransformation(Affine3d &transformationReference , double optional_1);


    uint8_t Begin_ID;


private:


    // MDH paramters
    MatrixXd m_MDHParameters;
    VectorXd m_gamma ;
    VectorXd m_b     ;
    VectorXd m_alpha ;
    VectorXd m_d     ;
    VectorXd m_r     ;

    //Buffer values
    std::vector<Affine3d> m_IKinematics;

protected:


    // Temp values


};

#endif // HeadController_H
