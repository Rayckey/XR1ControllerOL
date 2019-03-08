//
// Created by rocky on 19-3-7.
//

#ifndef PROJECT_XR1CONTROLLEROLMSGULIT_H

#include "Eigen/Dense"

#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerros/HandMsgs.h"
#include "xr1controllerros/JointAttributeMsgs.h"

#include <eigen_conversions/eigen_msg.h>



using namespace Eigen;

// Convert Joint ROS Messages

Eigen::VectorXd BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs &msg);

Eigen::VectorXd ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs &msg);

Eigen::VectorXd HandsMsgs2VectorXd(const xr1controllerros::HandMsgs &msg);

void BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs &msg   , VectorXd & output_ref);

void ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs &msg     , VectorXd & output_ref);

void HandsMsgs2VectorXd(const xr1controllerros::HandMsgs &msg  , VectorXd & output_ref);



xr1controllerros::HandMsgs ConvertHandMsgs(Eigen::VectorXd & HandPosition);

xr1controllerros::HandMsgs ConvertHandMsgs(std::vector<double> HandPosition);

xr1controllerros::ArmMsgs ConvertArmMsgs(std::vector<double> input);

xr1controllerros::ArmMsgs ConvertArmMsgs(Eigen::VectorXd & input);

xr1controllerros::BodyMsgs ConvertBodyMsgs(std::vector<double> input);

xr1controllerros::BodyMsgs ConvertBodyMsgs(Eigen::VectorXd & input);



void ConvertHandMsgs(Eigen::VectorXd & HandPosition , xr1controllerros::HandMsgs & msg);

void ConvertHandMsgs(std::vector<double> HandPosition , xr1controllerros::HandMsgs & msg);

void ConvertArmMsgs(std::vector<double> input , xr1controllerros::ArmMsgs & msg);

void ConvertArmMsgs(Eigen::VectorXd & input , xr1controllerros::ArmMsgs & msg);

void  ConvertBodyMsgs(std::vector<double> input , xr1controllerros::BodyMsgs & msg);

void  ConvertBodyMsgs(Eigen::VectorXd & input , xr1controllerros::BodyMsgs & msg);



#define PROJECT_XR1CONTROLLEROLMSGULIT_H

#endif //PROJECT_XR1CONTROLLEROLMSGULIT_H
