#include <ros/ros.h>
#include "xr1controller.h"
#include "xr1controlleralp.h"
#include "xr1controllerolmsgulit.h"
#include "xr1define.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/ChainModeChange.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerros/HandMsgs.h"
#include "xr1controllerol/AnimationMsgs.h"
#include <ros/package.h>

// Global Varibles
XR1Controller *XR1_ptr;
XR1ControllerALP *XRA_ptr;

tf::TransformBroadcaster *EFF_Broadcaster;
tf::TransformListener *EFF_Listener;
ros::Publisher *LeftArmPositionPublisher;
ros::Publisher *RightArmPositionPublisher;
ros::Publisher *MainBodyPositionPublisher;
ros::Publisher *LeftHandPositionPublisher;
ros::Publisher *RightHandPositionPublisher;
ros::Publisher *HeadBodyPositionPublisher;
VectorXd temp_vec5d;
VectorXd temp_vec7d;
VectorXd temp_vec4d;
VectorXd temp_vec3d;
xr1controllerros::HandMsgs temp_handmsgs;
xr1controllerros::ArmMsgs temp_armmsgs;
xr1controllerros::BodyMsgs temp_bodymsgs;
xr1controllerros::HeadMsgs temp_headmsgs;


bool animation_switch;


void subscribeStartAnimation(const std_msgs::Bool &msg) {
    animation_switch = msg.data;

    if (animation_switch){
        XR1_ptr->setSubControlMode(XR1::MainBody, XR1::AnimationMode);
        XR1_ptr->setSubControlMode(XR1::HeadBody, XR1::AnimationMode);
        XR1_ptr->setSubControlMode(XR1::LeftArm, XR1::AnimationMode);
        XR1_ptr->setSubControlMode(XR1::RightArm, XR1::AnimationMode);
        XR1_ptr->setSubControlMode(XR1::LeftHand, XR1::AnimationMode);
        XR1_ptr->setSubControlMode(XR1::RightHand, XR1::AnimationMode);
    }

}

void subscribeSubControlMode(const xr1controllerros::ChainModeChange & msg){

    XR1_ptr->setSubControlMode(msg.ChainID,msg.Mode);

}


void subscribeSetAnimation(const xr1controllerol::AnimationMsgs &msg) {
    XRA_ptr->setAnimation(msg.AnimationType, msg.AnimationID);
}


void subscribeSetIdle(const std_msgs::Bool &msg) {
    XRA_ptr->setIdleOption(msg.data);
}


//-----------------------------------------------------------------------------

void broadcastTransform(const ros::TimerEvent &event) {

    Eigen::Affine3d itsafine;
    tf::StampedTransform transform;


    // This function triggers almost all the computation in the library
    XR1_ptr->triggerCalculation();

    // Publish the left one
    XR1_ptr->getEndEffectorTransformation(XR1::LeftArm, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


    // Publish the right one
    XR1_ptr->getEndEffectorTransformation(XR1::RightArm, itsafine);
    // std::cout << itsafine.matrix() << std::endl;
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));


    XR1_ptr->getEndEffectorTransformation(XR1::HeadBody, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/Head"));


    XR1_ptr->getBaseTransformation(XR1::OmniWheels, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Odom", "/Base"));


    XR1_ptr->getBaseTransformation(XR1::MainBody, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Base", "/Back_Y"));


    if (animation_switch) {
        XRA_ptr->getNextState();

        if (XR1_ptr->inHighFrequencyControl(XR1::MainBody)) {
            XR1_ptr->getTargetPosition(XR1::MainBody, temp_vec7d, true);
            ConvertBodyMsgs(temp_vec7d, temp_bodymsgs);
            MainBodyPositionPublisher->publish(temp_bodymsgs);
        }

        if (XR1_ptr->inHighFrequencyControl(XR1::HeadBody)) {
            XR1_ptr->getTargetPosition(XR1::HeadBody, temp_vec7d, true);
            ConvertHeadMsgs(temp_vec7d, temp_headmsgs);
            HeadBodyPositionPublisher->publish(temp_headmsgs);
        }

        if (XR1_ptr->inHighFrequencyControl(XR1::LeftArm)) {
            XR1_ptr->getTargetPosition(XR1::LeftArm, temp_vec7d, true);
            ConvertArmMsgs(temp_vec7d, temp_armmsgs);
            LeftArmPositionPublisher->publish(temp_armmsgs);
        }


        if (XR1_ptr->inHighFrequencyControl(XR1::RightArm)) {
            XR1_ptr->getTargetPosition(XR1::RightArm, temp_vec7d, true);
            ConvertArmMsgs(temp_vec7d, temp_armmsgs);
            RightArmPositionPublisher->publish(temp_armmsgs);
        }

        if (XR1_ptr->inHighFrequencyControl(XR1::LeftHand)) {
            XR1_ptr->getTargetPosition(XR1::LeftHand, temp_vec7d, true);
            ConvertHandMsgs(temp_vec7d, temp_handmsgs);
            LeftHandPositionPublisher->publish(temp_handmsgs);
        }

        if (XR1_ptr->inHighFrequencyControl(XR1::RightHand)) {
            XR1_ptr->getTargetPosition(XR1::RightHand, temp_vec7d, true);
            ConvertHandMsgs(temp_vec7d, temp_handmsgs);
            RightHandPositionPublisher->publish(temp_handmsgs);
        }


        // trick ourselves into thinking we have velocites
        VectorXd omni_cmd = XR1_ptr->getTargetVelocity(XR1::OmniWheels, true);

        XR1_ptr->updatingCallback(XR1::OmniWheels, XR1::ActualVelocity, omni_cmd);

//    ROS_INFO("The state is wheels are [%d] " , XRA_ptr->isOmniWheelsMoving());

    }

}



// ----------------------------------------------------------------------------------
// ==================================================================================



// Look man you want the fk you gotta to feed me the angles
void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs &msg) {

    ArmMsgs2VectorXd(msg, temp_vec7d);
    XR1_ptr->updatingCallback(XR1::LeftArm, XR1::ActualPosition, temp_vec7d);

}

void subscribeRightArmPosition(const xr1controllerros::ArmMsgs &msg) {

    ArmMsgs2VectorXd(msg, temp_vec7d);
    XR1_ptr->updatingCallback(XR1::RightArm, XR1::ActualPosition, temp_vec7d);

}

void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs &msg) {

    BodyMsgs2VectorXd(msg, temp_vec7d);
    XR1_ptr->updatingCallback(XR1::MainBody, XR1::ActualPosition, temp_vec7d);

}

void subscribeHeadBodyPosition(const xr1controllerros::HeadMsgs &msg) {

    HeadMsgs2VectorXd(msg, temp_vec7d);
    XR1_ptr->updatingCallback(XR1::HeadBody, XR1::ActualPosition, temp_vec7d);

}


void subscribeLeftHandPosition(const xr1controllerros::HandMsgs &msg) {
    HandsMsgs2VectorXd(msg, temp_vec7d);
    XR1_ptr->updatingCallback(XR1::LeftHand, XR1::ActualPosition, temp_vec7d);
}

void subscribeRightHandPosition(const xr1controllerros::HandMsgs &msg) {
    HandsMsgs2VectorXd(msg, temp_vec7d);
    XR1_ptr->updatingCallback(XR1::RightHand, XR1::ActualPosition, temp_vec7d);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "IK_Simulator");

    ros::NodeHandle nh;

    animation_switch = false;


    temp_vec5d = VectorXd::Zero(5);
    temp_vec7d = VectorXd::Zero(7);
    temp_vec3d = VectorXd::Zero(3);
    temp_vec4d = VectorXd::Zero(4);

    std::string path = ros::package::getPath("xr1controllerol");


    std::vector<double> sit_pos;

    while (sit_pos.size() < XR1::Actuator_Total)
        sit_pos.push_back(0);

    XR1_ptr = new XR1Controller(path + "/strawberry.xr1para", sit_pos);

    XRA_ptr = new XR1ControllerALP(path + "/ALP", XR1_ptr, 169, 10, 1);

    EFF_Broadcaster = new tf::TransformBroadcaster();


    // Feed me infos
    ros::Subscriber LeftArmPositionSubscriber = nh.subscribe("/LeftArm/Position", 3, subscribeLeftArmPosition);

    // Feed me more
    ros::Subscriber RightArmPositionSubscriber = nh.subscribe("/RightArm/Position", 3, subscribeRightArmPosition);

    ros::Subscriber MainBodyPositionSubscriber = nh.subscribe("/MainBody/Position", 3, subscribeMainBodyPosition);

    ros::Subscriber HeadBodyPositionSubscriber = nh.subscribe("/HeadBody/Position", 3, subscribeHeadBodyPosition);


    ros::Subscriber LeftHandPositionSubscriber = nh.subscribe("/LeftHand/Position", 3, subscribeLeftHandPosition);

    ros::Subscriber RightHandPositionSubscriber = nh.subscribe("/RightHand/Position", 3, subscribeRightHandPosition);


    ros::Subscriber StartAnimationSubscriber = nh.subscribe("/startAnimation", 3, subscribeStartAnimation);

    ros::Subscriber SetAnimationSubscriber = nh.subscribe("/setAnimation", 3, subscribeSetAnimation);

    ros::Subscriber setSubControlModeSubscriber = nh.subscribe("/setSubControlMode" , 3 , subscribeSubControlMode);

    ros::Subscriber SetIdleSubscriber = nh.subscribe("setIdle" , 1 , subscribeSetIdle);

    // cough out the target position as the result of ALP
    ros::Publisher LAPP = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetPosition", 1);
    ros::Publisher RAPP = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetPosition", 1);
    ros::Publisher MBPP = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/TargetPosition", 1);
    ros::Publisher HBPP = nh.advertise<xr1controllerros::HeadMsgs>("/HeadBody/TargetPosition", 1);
    ros::Publisher LHPP = nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/TargetPosition", 1);
    ros::Publisher RHPP = nh.advertise<xr1controllerros::HandMsgs>("/RightHand/TargetPosition", 1);

    LeftArmPositionPublisher = &LAPP;
    RightArmPositionPublisher = &RAPP;
    MainBodyPositionPublisher = &MBPP;
    LeftHandPositionPublisher = &LHPP;
    RightHandPositionPublisher = &RHPP;
    HeadBodyPositionPublisher = &HBPP;


    // Draw some random stuff every three seconds or so
    ros::Timer timer = nh.createTimer(ros::Duration(0.005), broadcastTransform);

    ros::spin();

}