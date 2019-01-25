#ifndef OMNICONTROLLER_H
#define OMNICONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"
#include "xr1controllerutil.h"

using namespace Eigen;
class OmniController : public GenericController
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OmniController(uint8_t id, uint8_t num_joint);

    void setVelocity(Vector3d input);

    Vector3d getVelocity();

    Vector3d getPosition();

    void getBaseTransformation(Affine3d & input);

    void calculateSpeed();

    void triggerCalculation();

    void resetOdometry();


private:
    double WheelRadius;

    double OmniRadius;

    double left_front_angle;

    double right_front_angle;

    double back_angle;


    Vector3d OmniVelocity;

    Vector3d TargetOmniVelocity;

    Vector3d OmniPosition;

    MatrixXd OmniTransformation;

    void statePropagation();


    double max_vel;
    double max_ang;


    Matrix3d MappingMatrix;
    Matrix3d invMappingMatrix;


    void inverseSpeed();



};

#endif // OMNICONTROLLER_H
