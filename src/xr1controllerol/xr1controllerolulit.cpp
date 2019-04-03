//
// Created by rocky on 19-2-11.
//
#include "../../include/xr1controllerol/xr1controllerol.h"



void XR1ControllerOL::subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointVelocity(XR1::LeftArm, temp_vec7d);
        XR1_ptr->getJointVelocities(XR1::LeftArm , temp_vec7d);
        setJointVelocity(XR1::LeftArm, temp_vec7d);
    }
}

void XR1ControllerOL::subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {

        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointCurrent(XR1::LeftArm, temp_vec7d);
        XR1_ptr->getTargetCurrent(XR1::LeftArm , temp_vec7d);
        setJointCurrent(XR1::LeftArm, temp_vec7d);
    }
}

void XR1ControllerOL::subscribeRightArmVelocity(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointVelocity(XR1::RightArm, temp_vec7d);
        XR1_ptr->getTargetVelocity(XR1::RightArm , temp_vec7d);
        setJointVelocity(XR1::RightArm, temp_vec7d);
    }
}

void XR1ControllerOL::subscribeRightArmCurrent(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointCurrent(XR1::RightArm, temp_vec7d);
        XR1_ptr->getTargetCurrent(XR1::RightArm , temp_vec7d);
        setJointCurrent(XR1::RightArm, temp_vec7d);
    }
}

void XR1ControllerOL::subscribeLeftArmPosition(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){
       ROS_INFO("It is active right now , can't move ");
    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointPosition(XR1::LeftArm, temp_vec7d);
        XR1_ptr->getTargetPosition(XR1::LeftArm , temp_vec7d);
        setJointPosition(XR1::LeftArm, temp_vec7d);
    }
}


VectorXd XR1ControllerOL::getTargetPosition(uint8_t control_group, bool vanilla) {
    return XR1_ptr->getTargetPosition(control_group, vanilla);
}

void XR1ControllerOL::getTargetPosition(uint8_t control_group, VectorXd  & output_ref , bool vanilla){
    XR1_ptr->getTargetPosition(control_group , output_ref , vanilla);
}

void XR1ControllerOL::subscribeRightArmPosition(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointPosition(XR1::RightArm, temp_vec7d);
        XR1_ptr->getTargetPosition(XR1::RightArm, temp_vec7d);
        setJointPosition(XR1::RightArm, temp_vec7d);
    }
}


void XR1ControllerOL::subscribeMainBodyPosition(const xr1controllerros::BodyMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        BodyMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointPosition(XR1::MainBody, temp_vec7d);
        XR1_ptr->getTargetPosition(XR1::MainBody,temp_vec7d);
        setJointPosition(XR1::MainBody, temp_vec7d);
    }
}


void XR1ControllerOL::subscribeHeadBodyPosition(const xr1controllerros::HeadMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        HeadMsgs2VectorXd(msg , temp_vec3d);
        XR1_ptr->setJointPosition(XR1::HeadBody, temp_vec3d);
        XR1_ptr->getTargetPosition(XR1::HeadBody,temp_vec3d);
        setJointPosition(XR1::HeadBody, temp_vec3d);
    }
}

void XR1ControllerOL::subscribeMainBodyCurrent(const xr1controllerros::BodyMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        BodyMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointCurrent(XR1::MainBody, temp_vec7d);
        XR1_ptr->getTargetCurrent(XR1::MainBody , temp_vec7d);
        setJointCurrent(XR1::MainBody, temp_vec7d);
    }
}


void XR1ControllerOL::subscribeLeftHandPosition(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        HandsMsgs2VectorXd(msg , temp_vec5d);
        XR1_ptr->setJointPosition(XR1::LeftHand, temp_vec5d);
        XR1_ptr->getTargetPosition(XR1::LeftHand , temp_vec5d);
        setJointPosition(XR1::LeftHand, temp_vec5d);
    }
}


void XR1ControllerOL::subscribeRightHandPosition(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        HandsMsgs2VectorXd(msg , temp_vec5d);
        XR1_ptr->setJointPosition(XR1::RightHand, temp_vec5d);
        XR1_ptr->getTargetPosition(XR1::RightHand , temp_vec5d);
        setJointPosition(XR1::RightHand, temp_vec5d);
    }
}

void XR1ControllerOL::subscribeLeftHandCurrent(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        HandsMsgs2VectorXd(msg , temp_vec5d);
        XR1_ptr->setJointCurrent(XR1::LeftHand, temp_vec5d);
        XR1_ptr->getTargetCurrent(XR1::LeftHand , temp_vec5d);
        setJointCurrent(XR1::LeftHand, temp_vec5d);
    }
}

void XR1ControllerOL::subscribeRightHandCurrent(const xr1controllerros::HandMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {
        HandsMsgs2VectorXd(msg , temp_vec5d);
        XR1_ptr->setJointCurrent(XR1::RightHand, temp_vec5d);
        XR1_ptr->getTargetCurrent(XR1::RightHand , temp_vec5d);
        setJointCurrent(XR1::RightHand, temp_vec5d);
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


void XR1ControllerOL::subscribeHeadBodyMode(const xr1controllerros::ChainModeChange &msg){
    setControlMode(XR1::HeadBody , msg.Mode);
}

void XR1ControllerOL::subscribeBackBodyMode(const xr1controllerros::ChainModeChange &msg){
    setControlMode(XR1::MainBody , msg.Mode);
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
