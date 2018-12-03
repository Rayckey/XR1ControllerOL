#ifndef OMNICONTROLLER_H
#define OMNICONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"

using namespace Eigen;
class OmniController : public GenericController
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OmniController(uint8_t id, uint8_t num_joint);

    void setVelocity(Vector3d input);

    void calculateSpeed();


private:
    double WheelRadius;

    double OmniRadius;

    double left_front_angle;

    double right_front_angle;

    double back_angle;


    Vector3d OmniVelocity;

    Vector3d TargetOmniVelocity;


    double max_vel;
    double max_ang;

    double PI;

    Matrix3d MappingMatrix;


    void inverseSpeed(Vector3d Velocity);



};

#endif // OMNICONTROLLER_H
