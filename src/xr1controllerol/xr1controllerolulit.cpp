//
// Created by rocky on 19-2-11.
// Just a ton of ultility functions
//
#include "../../include/xr1controllerol/xr1controllerol.h"


void XR1ControllerOL::subscribeLeftArmPosition(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){
        ROS_INFO("It is active right now , can't move ");
    }

    else if (XR1_ptr->getSubControlMode(XR1::LeftArm) == XR1::DirectMode){
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointPosition(XR1::LeftArm, temp_vec7d);
        setControlGroupTarget(XR1::LeftArm);
    }
}

void XR1ControllerOL::subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointVelocity(XR1::LeftArm, temp_vec7d);
        setControlGroupTarget(XR1::LeftArm);
    }
}

void XR1ControllerOL::subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointCurrent(XR1::LeftArm, temp_vec7d);
        setControlGroupTarget(XR1::LeftArm);
    }
}



void XR1ControllerOL::subscribeRightArmPosition(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else if (XR1_ptr->getSubControlMode(XR1::RightArm) == XR1::DirectMode){
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointPosition(XR1::RightArm, temp_vec7d);
        setControlGroupTarget(XR1::RightArm);
    }
}

void XR1ControllerOL::subscribeRightArmVelocity(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointVelocity(XR1::RightArm, temp_vec7d);
        setControlGroupTarget(XR1::RightArm);
    }
}

void XR1ControllerOL::subscribeRightArmCurrent(const xr1controllerros::ArmMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        ArmMsgs2VectorXd(msg , temp_vec7d);
        XR1_ptr->setJointCurrent(XR1::RightArm, temp_vec7d);
        setControlGroupTarget(XR1::RightArm);
    }
}



void XR1ControllerOL::subscribeMainBodyPosition(const xr1controllerros::BodyMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else if (XR1_ptr->getSubControlMode(XR1::MainBody) == XR1::DirectMode){
        BodyMsgs2VectorXd(msg , temp_vec4d);
        XR1_ptr->setJointPosition(XR1::MainBody, temp_vec4d);
        setControlGroupTarget(XR1::MainBody);
    }
}

void XR1ControllerOL::subscribeMainBodyCurrent(const xr1controllerros::BodyMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        BodyMsgs2VectorXd(msg , temp_vec4d);
        XR1_ptr->setJointCurrent(XR1::MainBody, temp_vec4d);
        setControlGroupTarget(XR1::MainBody);
    }
}


void XR1ControllerOL::subscribeHeadBodyPosition(const xr1controllerros::HeadMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else if (XR1_ptr->getSubControlMode(XR1::HeadBody) == XR1::DirectMode){
        HeadMsgs2VectorXd(msg , temp_vec3d);
        XR1_ptr->setJointPosition(XR1::HeadBody, temp_vec3d);
        setControlGroupTarget(XR1::HeadBody);
    }
}



void XR1ControllerOL::subscribeLeftHandPosition(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else if (XR1_ptr->getSubControlMode(XR1::LeftHand) == XR1::DirectMode) {
        HandsMsgs2VectorXd(msg , temp_vec5d);
        XR1_ptr->setJointPosition(XR1::LeftHand, temp_vec5d);
        setControlGroupTarget(XR1::LeftHand);
    }
}


void XR1ControllerOL::subscribeLeftHandCurrent(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else {
        HandsMsgs2VectorXd(msg , temp_vec5d);
        XR1_ptr->setJointCurrent(XR1::LeftHand, temp_vec5d);
        setControlGroupTarget(XR1::LeftHand);
    }
}

void XR1ControllerOL::subscribeRightHandPosition(const xr1controllerros::HandMsgs &msg) {
    if (XR1_ptr->isStateActive()){

    }

    else if (XR1_ptr->getSubControlMode(XR1::RightHand) == XR1::DirectMode){
        HandsMsgs2VectorXd(msg , temp_vec5d);
        XR1_ptr->setJointPosition(XR1::RightHand, temp_vec5d);
        setControlGroupTarget(XR1::RightHand);
    }
}

void XR1ControllerOL::subscribeRightHandCurrent(const xr1controllerros::HandMsgs &msg) {

    if (XR1_ptr->isStateActive()){

    }

    else {
        HandsMsgs2VectorXd(msg , temp_vec5d);
        XR1_ptr->setJointCurrent(XR1::RightHand, temp_vec5d);
        setControlGroupTarget(XR1::RightHand);
    }
}



void XR1ControllerOL::getTargetPosition(uint8_t control_group, VectorXd  & output_ref , bool vanilla){
    XR1_ptr->getTargetPosition(control_group , output_ref , vanilla);
}
