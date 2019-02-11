//
// Created by rocky on 19-2-11.
//
#include "../../include/xr1controllerol/xr1controllerol.h"

Eigen::VectorXd XR1ControllerOL::BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

    res << msg.Knee,
            msg.Back_Z,
            msg.Back_X,
            msg.Back_Y,
            msg.Neck_Z,
            msg.Neck_X,
            msg.Head;

    return res;
}

Eigen::VectorXd XR1ControllerOL::ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

    res << msg.Shoulder_X,
            msg.Shoulder_Y,
            msg.Elbow_Z,
            msg.Elbow_X,
            msg.Wrist_Z,
            msg.Wrist_X,
            msg.Wrist_Y;


    return res;
}

Eigen::VectorXd XR1ControllerOL::HandsMsgs2VectorXd(const xr1controllerros::HandMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(5);

    res << msg.Thumb,
            msg.Index,
            msg.Middle,
            msg.Ring,
            msg.Pinky;


    return res;
}


void XR1ControllerOL::subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointVelocity(XR1::LeftArm, ArmMsgs2VectorXd(msg));
        setJointVelocity(XR1::LeftArm, XR1_ptr->getTargetVelocity(XR1::LeftArm));
    }
}

void XR1ControllerOL::subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointCurrent(XR1::LeftArm, ArmMsgs2VectorXd(msg));
        setJointCurrent(XR1::LeftArm, XR1_ptr->getTargetCurrent(XR1::LeftArm));
    }
}

void XR1ControllerOL::subscribeRightArmVelocity(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointVelocity(XR1::RightArm, ArmMsgs2VectorXd(msg));
        setJointVelocity(XR1::RightArm, XR1_ptr->getTargetVelocity(XR1::RightArm));
    }
}

void XR1ControllerOL::subscribeRightArmCurrent(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointCurrent(XR1::RightArm, ArmMsgs2VectorXd(msg));
        setJointCurrent(XR1::RightArm, XR1_ptr->getTargetCurrent(XR1::RightArm));
    }
}

void XR1ControllerOL::subscribeLeftArmPosition(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointPosition(XR1::LeftArm, ArmMsgs2VectorXd(msg));
        setJointPosition(XR1::LeftArm, XR1_ptr->getTargetPosition(XR1::LeftArm));
    }
}


VectorXd XR1ControllerOL::getTargetPosition(uint8_t control_group, bool vanilla) {
    return XR1_ptr->getTargetPosition(control_group, vanilla);
}


void XR1ControllerOL::subscribeRightArmPosition(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointPosition(XR1::RightArm, ArmMsgs2VectorXd(msg));
        setJointPosition(XR1::RightArm, XR1_ptr->getTargetPosition(XR1::RightArm));
    }
}


void XR1ControllerOL::subscribeMainBodyPosition(const xr1controllerros::BodyMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointPosition(XR1::MainBody, BodyMsgs2VectorXd(msg));
        setJointPosition(XR1::MainBody, XR1_ptr->getTargetPosition(XR1::MainBody));
    }
}

void XR1ControllerOL::subscribeMainBodyCurrent(const xr1controllerros::BodyMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointCurrent(XR1::MainBody, BodyMsgs2VectorXd(msg));
        setJointCurrent(XR1::MainBody, XR1_ptr->getTargetCurrent(XR1::MainBody));
    }
}


void XR1ControllerOL::subscribeLeftHandPosition(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointPosition(XR1::LeftHand, HandsMsgs2VectorXd(msg));
        setJointPosition(XR1::LeftHand, XR1_ptr->getTargetPosition(XR1::LeftHand));
    }
}


void XR1ControllerOL::subscribeRightHandPosition(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointPosition(XR1::RightHand, HandsMsgs2VectorXd(msg));
        setJointPosition(XR1::RightHand, XR1_ptr->getTargetPosition(XR1::RightHand));
    }
}

void XR1ControllerOL::subscribeLeftHandCurrent(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointCurrent(XR1::LeftHand, HandsMsgs2VectorXd(msg));
        setJointCurrent(XR1::LeftHand, XR1_ptr->getTargetCurrent(XR1::LeftHand));
    }
}

void XR1ControllerOL::subscribeRightHandCurrent(const xr1controllerros::HandMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {
        XR1_ptr->setJointCurrent(XR1::RightHand, HandsMsgs2VectorXd(msg));
        setJointCurrent(XR1::RightHand, XR1_ptr->getTargetCurrent(XR1::RightHand));
    }
}


void XR1ControllerOL::subscribeMainBodyMode(const xr1controllerros::ChainModeChange &msg) {
    setControlMode(XR1::MainBody, msg.Mode);
}

void XR1ControllerOL::subscribeLeftArmMode(const xr1controllerros::ChainModeChange &msg) {
    setControlMode(XR1::LeftArm, msg.Mode);
}

void XR1ControllerOL::subscribeRightArmMode(const xr1controllerros::ChainModeChange &msg) {
    setControlMode(XR1::RightArm, msg.Mode);
}

void XR1ControllerOL::subscribeLeftHandMode(const xr1controllerros::ChainModeChange &msg) {
    setControlMode(XR1::LeftHand, msg.Mode);
}

void XR1ControllerOL::subscribeRightHandMode(const xr1controllerros::ChainModeChange &msg) {
    setControlMode(XR1::RightHand, msg.Mode);
}



xr1controllerros::HandMsgs XR1ControllerOL::ConvertHandMsgs(Eigen::VectorXd HandPosition) {

    xr1controllerros::HandMsgs msg;
    msg.Thumb = HandPosition(0);
    msg.Index = HandPosition(1);
    msg.Middle = HandPosition(2);
    msg.Ring = HandPosition(3);
    msg.Pinky = HandPosition(4);

    return msg;
}

xr1controllerros::HandMsgs XR1ControllerOL::ConvertHandMsgs(std::vector<double> HandPosition) {
    xr1controllerros::HandMsgs msg;
    msg.Thumb = HandPosition[0];
    msg.Index = HandPosition[1];
    msg.Middle = HandPosition[2];
    msg.Ring = HandPosition[3];
    msg.Pinky = HandPosition[4];
    return msg;
}


xr1controllerros::ArmMsgs XR1ControllerOL::ConvertArmMsgs(std::vector<double> input) {

    xr1controllerros::ArmMsgs msg;

    msg.Shoulder_X = input[0];
    msg.Shoulder_Y = input[1];
    msg.Elbow_Z = input[2];
    msg.Elbow_X = input[3];
    msg.Wrist_Z = input[4];
    msg.Wrist_X = input[5];
    msg.Wrist_Y = input[6];


    return msg;
}

xr1controllerros::ArmMsgs XR1ControllerOL::ConvertArmMsgs(Eigen::VectorXd input) {
    xr1controllerros::ArmMsgs msg;

    msg.Shoulder_X = input(0);
    msg.Shoulder_Y = input(1);
    msg.Elbow_Z = input(2);
    msg.Elbow_X = input(3);
    msg.Wrist_Z = input(4);
    msg.Wrist_X = input(5);
    msg.Wrist_Y = input(6);

    return msg;
}

xr1controllerros::BodyMsgs XR1ControllerOL::ConvertBodyMsgs(std::vector<double> input) {

    xr1controllerros::BodyMsgs msg;

    msg.Knee = input[0];
    msg.Back_Z = input[1];
    msg.Back_X = input[2];
    msg.Back_Y = input[3];
    msg.Neck_Z = input[4];
    msg.Neck_X = input[5];
    msg.Head = input[6];

    return msg;
}

xr1controllerros::BodyMsgs XR1ControllerOL::ConvertBodyMsgs(Eigen::VectorXd input) {

    xr1controllerros::BodyMsgs msg;

    msg.Knee = input(0);
    msg.Back_Z = input(1);
    msg.Back_X = input(2);
    msg.Back_Y = input(3);
    msg.Neck_Z = input(4);
    msg.Neck_X = input(5);
    msg.Head = input(6);

    return msg;
}


void XR1ControllerOL::subscribeLeftElbowAngle(const std_msgs::Float64 &msg) {
    LeftElbowAngle = msg.data;
}

void XR1ControllerOL::subscribeRightElbowAngle(const std_msgs::Float64 &msg) {
    RightElbowAngle = msg.data;
}



void XR1ControllerOL::subscribetiltInit(const std_msgs::Bool &msg) {
    XR1_ptr->tiltInit();
}

void XR1ControllerOL::subscribeMoCapInit(const std_msgs::Bool &msg) {
    ROS_INFO("Here be initialized");
//    XR1_ptr->setMetaMode(XR1::MoCapMode);
    IMU_ptr->Initialize();
}