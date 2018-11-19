#ifndef HANDCONTROLLER_H
#define HANDCONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>
using namespace Eigen;
class HandController : public GenericController
{
public:
    HandController(uint8_t id , int num_joint);

};

#endif // HANDCONTROLLER_H
