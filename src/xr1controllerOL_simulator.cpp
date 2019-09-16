#include <ros/ros.h>
#include "xr1controller.h"
#include "xr1controlleralp.h"
#include "xr1controllerblc.h"
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


#include <map>
#include <vector>

#include "xr1controllerol/IKPlannerService.h"
#include "xr1controllerol/IKLinearService.h"
#include "xr1controllerol/HandGripQuery.h"
#include "xr1controllerol/RobotStateQuery.h"
#include "xr1controllerol/askReadiness.h"
#include "xr1controllerol/AnimationQuery.h"


// Global Varibles
XR1Controller *XR1_ptr;
XR1ControllerALP *XRA_ptr;
XR1ControllerBLC *XRB_ptr;
std::vector<uint8_t> temp_ids;

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
Vector3d temp_vec3d_fix;
xr1controllerros::HandMsgs temp_handmsgs;
xr1controllerros::ArmMsgs temp_armmsgs;
xr1controllerros::BodyMsgs temp_bodymsgs;
xr1controllerros::HeadMsgs temp_headmsgs;
geometry_msgs::Transform temp_geo_trans;


std::vector<uint8_t> control_group_flags;

ros::ServiceServer QueryAnimationService;

Eigen::Affine3d itsafine;
tf::StampedTransform temp_transform;

// Common functions ---------------------------------------------------------------------------

void setControlGroupTarget(uint8_t control_group){

    XR1_ptr->getTargetPosition(control_group , temp_vec7d , true);

    switch (control_group){

        case XR1::MainBody:{

            ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
            MainBodyPositionPublisher->publish(temp_bodymsgs);

        }

            break;


        case XR1::HeadBody:{

            ConvertHeadMsgs(temp_vec7d , temp_headmsgs);
            HeadBodyPositionPublisher->publish(temp_headmsgs);

        }

            break;


        case XR1::LeftArm:{

            ConvertArmMsgs(temp_vec7d , temp_armmsgs);
            LeftArmPositionPublisher->publish(temp_armmsgs);

        }

            break;


        case XR1::RightArm:{

            ConvertArmMsgs(temp_vec7d , temp_armmsgs);
            RightArmPositionPublisher->publish(temp_armmsgs);

        }

            break;


        case XR1::LeftHand:{

            ConvertHandMsgs(temp_vec7d , temp_handmsgs);
            LeftHandPositionPublisher->publish(temp_handmsgs);

        }

            break;

        case XR1::RightHand:{

            ConvertHandMsgs(temp_vec7d , temp_handmsgs);
            RightHandPositionPublisher->publish(temp_handmsgs);

        }

            break;


        default:
            break;



    }
}

void subscribeSubControlMode(const xr1controllerros::ChainModeChange & msg){


    ROS_INFO("Setting Control Group [%d] to Mode [%d]", msg.ChainID, msg.Mode);
    XR1_ptr->setSubControlMode(msg.ChainID,msg.Mode);

}


void clearStates() {
    XRA_ptr->clearAll();
    XR1_ptr->clearStates();
}




bool serviceQueryAnimation(xr1controllerol::AnimationQueryRequest &req,
                                            xr1controllerol::AnimationQueryResponse &res){

    int type_id , ani_id, pro_id;

    bool hasIdle, hasDefault;

    XRA_ptr->checkOptions(hasIdle , hasDefault);

    res.hasIdle = hasIdle;

    res.hasDefault = hasDefault;

    res.inAnimationMode = false;
    for (uint8_t control_group : control_group_flags){
        if (XR1_ptr->getSubControlMode(control_group) == XR1::AnimationMode){
            res.inAnimationMode = true;
        }
    }

    if (XRA_ptr->checkProgress(type_id,ani_id , pro_id) ){
        res.isPlaying = true;
        res.AnimationType = type_id;
        res.AnimationID = ani_id;
        res.AnimationProgress = pro_id;
    }

    else {
        res.isPlaying = false;
        res.AnimationType = 0;
        res.AnimationID = 0;
        res.AnimationProgress = 0;
    }


    return true;
}



bool serviceState(xr1controllerol::RobotStateQueryRequest & req,
                                   xr1controllerol::RobotStateQueryResponse & res){

    res.isOkay = false;
    res.RobotState = false;
    res.CollisionSwitch = false;
    res.HeadBodyMode = 0;
    res.MainBodyMode = 0;
    res.LeftArmMode = 0;
    res.RightArmMode = 0;
    res.LeftHandMode = 0;
    res.RightHandMode = 0;



    if (req.isQuery){

        res.isOkay = XR1_ptr->isXR1Okay();

        res.RobotState = XR1_ptr->getErrorCode();

        res.HeadBodyMode = XR1_ptr->getSubControlMode(XR1::HeadBody);
        res.MainBodyMode = XR1_ptr->getSubControlMode(XR1::MainBody);
        res.LeftArmMode  = XR1_ptr->getSubControlMode(XR1::LeftArm);
        res.RightArmMode = XR1_ptr->getSubControlMode(XR1::RightArm);
        res.LeftHandMode = XR1_ptr->getSubControlMode(XR1::LeftHand);
        res.RightHandMode= XR1_ptr->getSubControlMode(XR1::RightHand);

    }



    return true;
}
// --------------------------------------------------------------------------------------



// IK simulation --------------------------------------------------------------------------------------
bool serviceHandGrip(xr1controllerol::HandGripQueryRequest & req,
                     xr1controllerol::HandGripQueryResponse & res){

    res.inProgress = true;
    res.isGripped = false;

    int root_control_group = (req.ControlGroup == XR1::LeftHand) ? XR1::LeftArm : XR1::RightArm ;

    if (XR1_ptr->isIKPlannerActive(root_control_group)){

        res.inProgress = true;
        return true;

    }else {

        res.inProgress = false;
        res.isGripped = true;
    }


    return true;

}


// this is when we process the next state of the robot
void stateTransition() {

    XRA_ptr->getNextState();

    double temp_value;

    for (uint8_t control_group : control_group_flags){

//        ROS_INFO("Group [%d] is going " , control_group);

        if (XR1_ptr->inHighFrequencyControl(control_group) && XR1_ptr->isXR1Okay()){

            temp_ids = XR1_ptr->getControlGroupIDs(control_group);
            setControlGroupTarget(control_group);
        }
    }

}


// Is XR1 ready? the answer is always yes
bool serviceReady(xr1controllerol::askReadinessRequest & req,
                  xr1controllerol::askReadinessResponse & res){

    res.isReady = true;

    return true;
}

// Fake IK stuff
bool serviceIKPlanner(xr1controllerol::IKPlannerServiceRequest &req,
                      xr1controllerol::IKPlannerServiceResponse &res) {



    temp_geo_trans = req.TargetTransform;

    tf::transformMsgToEigen(temp_geo_trans, itsafine);

    uint8_t control_group = req.ControlGroup;




    // The default response
    res.inProgress = true;
    res.isReachable = false;
    res.isAccepted = false;

    if (XR1_ptr->isIKPlannerActive(control_group)) {
        res.inProgress = true;
    } else {
        if (req.NewTarget) {
//            ROS_INFO("getting request with time [%f]" ,req.Period);
            XR1_ptr->setSubControlMode(control_group, XR1::IKMode);


            if (control_group == XR1::LeftArm)
                XR1_ptr->setSubControlMode(XR1::LeftHand, XR1::IKMode);
            else if (control_group == XR1::RightArm)
                XR1_ptr->setSubControlMode(XR1::RightHand, XR1::IKMode);

            res.inProgress = false;
            std::cout << itsafine.matrix() << std::endl;
            if (XR1_ptr->setEndEffectorTransformation(control_group, itsafine, req.TargetElbowAngle, req.Period)) {
                ROS_INFO("doing request" );

                res.isReachable = true;
                res.isAccepted = true;

                XR1_ptr->setGrippingSwitch(control_group, req.Grip);
            }
        }
        res.inProgress = false;

    }

    return true;

}
// ---------------------------------------------------------------------------------

bool serviceIKLinearPlanner(xr1controllerol::IKLinearServiceRequest &req,
                      xr1controllerol::IKLinearServiceResponse &res) {



    temp_geo_trans = req.TargetTransform;

    tf::transformMsgToEigen(temp_geo_trans, itsafine);

    uint8_t control_group = req.ControlGroup;



    // The default response
    res.inProgress = true;
    res.isReachable = false;
    res.isAccepted = false;

    if (XR1_ptr->isIKPlannerActive(control_group)) {
        res.inProgress = true;
    } else {
        if (req.NewTarget) {
//            ROS_INFO("getting request with time [%f]" ,req.Period);
            XR1_ptr->setSubControlMode(control_group, XR1::IKMode);


            if (control_group == XR1::LeftArm)
                XR1_ptr->setSubControlMode(XR1::LeftHand, XR1::IKMode);
            else if (control_group == XR1::RightArm)
                XR1_ptr->setSubControlMode(XR1::RightHand, XR1::IKMode);

            res.inProgress = false;
            std::cout << itsafine.matrix() << std::endl;
            if (XR1_ptr->setEndEffectorTransformation(control_group, itsafine, req.TargetElbowAngle, req.Period)) {
                ROS_INFO("doing request" );

                res.isReachable = true;
                res.isAccepted = true;

                XR1_ptr->setGrippingSwitch(control_group, req.Grip);
            }
        }
        res.inProgress = false;

    }

    return true;

}



// set Animation mode --------------------------------------------------------------
void subscribeStartAnimation(const std_msgs::Bool &msg) {

    // if true, set any avialable control groups to animation
    if (msg.data){

        for (uint8_t control_group : control_group_flags){

            if (XR1_ptr->getSubControlMode(control_group) <= XR1::MoCapMode){

                XR1_ptr->setSubControlMode(control_group , XR1::AnimationMode);

                ROS_INFO("Animation for Control Group [%d] , is now ON",  (int)control_group );

            }

        }

    }

    // if false, set any avialable control groups to animation
    else {

        for (uint8_t control_group : control_group_flags){

            if (XR1_ptr->getSubControlMode(control_group) == XR1::AnimationMode){

                XR1_ptr->setSubControlMode(control_group , XR1::DirectMode);

                ROS_INFO("Animation for Control Group [%d] , is now OFF",  (int)control_group );

            }

        }

//        XRA_ptr->setSingleTransitionPeriod(3);

        clearStates();
    }

}
// -----------------------------------------------------------------------------------

void subscribeSetAnimation(const xr1controllerol::AnimationMsgs &msg) {
    XRA_ptr->setAnimation(msg.AnimationType, msg.AnimationID);
}


void subscribeSetIdle(const std_msgs::Bool &msg) {
    XRA_ptr->setIdleOption(msg.data);
}


void subscribeSetDefault(const std_msgs::Bool &msg){
    XRA_ptr->setDefaultOption(msg.data);
}

//-----------------------------------------------------------------------------

void broadcastTransform(const ros::TimerEvent &event) {

    // This function triggers almost all the computation in the library
    XR1_ptr->triggerCalculation(true);

    // Publish the left one
    XR1_ptr->getEndEffectorTransformation(XR1::LeftArm, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, temp_transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(temp_transform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


    // Publish the right one
    XR1_ptr->getEndEffectorTransformation(XR1::RightArm, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    // std::cout << itsafine.matrix() << std::endl;
    tf::transformEigenToTF(itsafine, temp_transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(temp_transform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));


    XR1_ptr->getEndEffectorTransformation(XR1::HeadBody, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, temp_transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(temp_transform, ros::Time::now(), "/Back_Y", "/Head"));


    XR1_ptr->getBaseTransformation(XR1::OmniWheels, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, temp_transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(temp_transform, ros::Time::now(), "/Odom", "/Base"));


    XR1_ptr->getBaseTransformation(XR1::MainBody, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, temp_transform);
    EFF_Broadcaster->sendTransform(tf::StampedTransform(temp_transform, ros::Time::now(), "/Base", "/Back_Y"));


    stateTransition();




    if (XR1_ptr->getSubControlMode(XR1::OmniWheels) == XR1::RoamMode){

        XR1_ptr->getTargetVelocity(XR1::OmniWheels , temp_vec3d);

        for (uint8_t i = XR1::OmniWheels ; i < XR1::MainBody ; i++) {
            ROS_INFO("LF: [%f] , RF: [%f] , BK: [%f] " , temp_vec3d(0) ,temp_vec3d(1) ,temp_vec3d(2)) ;
            // simulation call -------------------------------------------------------------
            XR1_ptr->updatingCallback(i , XR1::ActualVelocity,temp_vec3d(i-XR1::OmniWheels));
            // -----------------------------------------------------------------------------
        }

    }




}



// ----------------------------------------------------------------------------------



// fukcing wheels lmao--------------------------------------------------------------------------
void subscribeOmniCommands(const geometry_msgs::Twist & msg){

    if (XR1_ptr->getSubControlMode(XR1::OmniWheels) == XR1::RoamMode){
        temp_vec3d_fix << msg.angular.x , msg.linear.y , msg.linear.z ;


        ROS_INFO(" New Omni Command Received [%f] [%f] [%f]" , temp_vec3d_fix(0) ,temp_vec3d_fix(1) ,temp_vec3d_fix(2)) ;


        XRA_ptr->setTargetOmniCmd(temp_vec3d_fix);
    }

}



// ----------------------------------------------------------------------------------

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
    ros::init(argc, argv, "OL_Simulator");

    ros::NodeHandle nh;


    temp_vec5d = VectorXd::Zero(5);
    temp_vec7d = VectorXd::Zero(7);
    temp_vec3d = VectorXd::Zero(3);
    temp_vec4d = VectorXd::Zero(4);


    // Getting all the group ID right ------------------------------------------------
    // control_group_flags.push_back(XR1::OmniWheels);
    control_group_flags.push_back(XR1::MainBody);
    control_group_flags.push_back(XR1::HeadBody);
    control_group_flags.push_back(XR1::LeftArm);
    control_group_flags.push_back(XR1::RightArm);
    control_group_flags.push_back(XR1::LeftHand);
    control_group_flags.push_back(XR1::RightHand);
    // -----------------------------------------------------------------------------


    ROS_INFO("Stuff" );
    std::string path = ros::package::getPath("xr1controllerol");


    XR1_ptr = new XR1Controller(path + "/xr1paras/CrimsonCurrant.xr1para");

    XRB_ptr = new XR1ControllerBLC(path + "/BLC" ,path + "/ALP" );

    XRA_ptr = new XR1ControllerALP(path + "/ALP", XR1_ptr, XRB_ptr);

    EFF_Broadcaster = new tf::TransformBroadcaster();

//    ROS_INFO("Stuff" );
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

    ros::Subscriber setSubControlModeSubscriber = nh.subscribe("/XR1/ChainModeChange" , 3 , subscribeSubControlMode);

    ros::Subscriber SetIdleSubscriber = nh.subscribe("setIdleAnimations" , 1 , subscribeSetIdle);

    ros::Subscriber SetDefaultSubscriber = nh.subscribe("setDefaultAnimation" , 1 , subscribeSetDefault);

    ros::Subscriber OmniSpeedSubscriber = nh.subscribe("/XR1/cmd_vel" , 10 , subscribeOmniCommands);


//    ROS_INFO("Stuff" );
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



    QueryAnimationService = nh.advertiseService("/queryAnimation", serviceQueryAnimation);

    ros::ServiceServer RobotStateService = nh.advertiseService("XR1/State" , serviceState);

    ros::ServiceServer IKPlannerService = nh.advertiseService("XR1/IKPlanner", serviceIKPlanner);

    ros::ServiceServer IKLinearPlannerService = nh.advertiseService("XR1/IKLPT", serviceIKLinearPlanner);

    ros::ServiceServer HandGripService = nh.advertiseService("XR1/HGQ", serviceHandGrip);

    ros::ServiceServer readyService = nh.advertiseService("XR1/Ready", serviceReady);


    ROS_INFO("Stuff" );
    // Draw some random stuff every three seconds or so
    ros::Timer timer = nh.createTimer(ros::Duration(0.005), broadcastTransform);

    ros::spin();

}