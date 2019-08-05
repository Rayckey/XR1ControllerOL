#ifndef BACKCONTROLLER_H
#define BACKCONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>
#include <iostream>

using namespace Eigen;

class BackController: public GenericController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackController(MatrixXd mdh_input, uint8_t group_id  , uint8_t num_joint);

    void triggerCalculation();




private:

    // Saves individual transformation in the transformation collection
    void Transformation();


    // DH paramters
    MatrixXd m_MDHParameters;
    VectorXd m_gamma ;
    VectorXd m_b     ;
    VectorXd m_alpha ;
    VectorXd m_d     ;
    VectorXd m_r     ;





protected:


    // Temp values


};

#endif // CHAINCONTROLLER_H
