#include <ros/ros.h>
#include "xr1controllerpm.h"
#include "xr1define.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64.h>
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/HandMsgs.h"
#include "xr1controllerros/ChainModeChange.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerol/IKLinearService.h"
#include <ros/package.h>

// Global Varibles
XR1ControllerPM * XR1_ptr;
double RightElbowAngle;
double LeftElbowAngle;
Eigen::Affine3d itsafine;
tf::StampedTransform tform;
geometry_msgs::Transform temp_geo_trans;
tf::TransformBroadcaster * EFF_Broadcaster;
tf::TransformListener * EFF_Listener;

ros::Publisher * LeftArmPositionPublisher;
ros::Publisher * RightArmPositionPublisher;
ros::Publisher * LeftHandPositionPublisher;
ros::Publisher * RightHandPositionPublisher;
ros::Publisher * MainBodyPositionPublisher;

// IGNORE THIS PART ==============================================================
// You know what this is --------------------------------------------------------
Eigen::VectorXd ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs& msg) {

  Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

  res << msg.Shoulder_X ,
      msg.Shoulder_Y,
      msg.Elbow_Z ,
      msg.Elbow_X ,
      msg.Wrist_Z ,
      msg.Wrist_X ,
      msg.Wrist_Y ;


  return res;
}

xr1controllerros::ArmMsgs ConvertArmMsgs(Eigen::VectorXd input) {
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


Eigen::VectorXd BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs& msg) {

  Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

  res << msg.Knee  ,
      msg.Back_Z,
      msg.Back_X,
      msg.Back_Y,
      msg.Neck_Z,
      msg.Neck_X,
      msg.Head;

  return res;
}


xr1controllerros::BodyMsgs ConvertBodyMsgs(Eigen::VectorXd input) {

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


Eigen::VectorXd HandsMsgs2VectorXd(const xr1controllerros::HandMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(5);

    res << msg.Thumb,
            msg.Index,
            msg.Middle,
            msg.Ring,
            msg.Pinky;


    return res;
}


xr1controllerros::HandMsgs ConvertHandMsgs(Eigen::VectorXd HandPosition) {

    xr1controllerros::HandMsgs msg;
    msg.Thumb = HandPosition(0);
    msg.Index = HandPosition(1);
    msg.Middle = HandPosition(2);
    msg.Ring = HandPosition(3);
    msg.Pinky = HandPosition(4);

    return msg;
}



void lookupRightEFFTarget(tf::StampedTransform & transform,  Eigen::Affine3d & itsafine) {

  try {
    EFF_Listener->lookupTransform( "/Back_Y", "/RightEndEffectorTarget",
                                   ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    return;
  }

  transformTFToEigen(transform, itsafine);
  XR1_ptr->setEndEffectorPosition(XR1::RightArm, itsafine , RightElbowAngle);

  RightArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::RightArm)));
}

void lookupLeftEFFTarget(tf::StampedTransform & transform,   Eigen::Affine3d & itsafine) {
  try {
    EFF_Listener->lookupTransform( "/Back_Y", "/LeftEndEffectorTarget",
                                   ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    return;
  }

  transformTFToEigen(transform, itsafine);
  XR1_ptr->setEndEffectorPosition(XR1::LeftArm, itsafine , LeftElbowAngle);

  LeftArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::LeftArm)));
}

void subscribeLeftElbowAngle(const std_msgs::Float64 & msg) {
  LeftElbowAngle = msg.data;
}
void subscribeRightElbowAngle(const std_msgs::Float64 & msg) {
  RightElbowAngle = msg.data;
}


void subscribeLeftArmMode(const xr1controllerros::ChainModeChange& msg) {
  XR1_ptr->setControlMode(XR1::LeftArm , msg.Mode);
}
void subscribeRightArmMode(const xr1controllerros::ChainModeChange& msg) {
  XR1_ptr->setControlMode(XR1::RightArm , msg.Mode);
}


void stateTransition(){

    std::vector<double> state_cmd = XR1_ptr->getNextState();

    if (state_cmd[0] < 0.5){

        RightArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::RightArm)));

        LeftArmPositionPublisher->publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::LeftArm)));

        MainBodyPositionPublisher->publish(ConvertBodyMsgs(XR1_ptr->getTargetPosition(XR1::MainBody)));

        LeftHandPositionPublisher->publish(ConvertHandMsgs(XR1_ptr->getTargetPosition(XR1::LeftHand)));

        RightHandPositionPublisher->publish(ConvertHandMsgs(XR1_ptr->getTargetPosition(XR1::RightHand)));
    }

}


bool serviceIKPlanner(xr1controllerol::IKLinearServiceRequest & req ,
                                       xr1controllerol::IKLinearServiceResponse & res){

    temp_geo_trans = req.TargetTransform;

    tf::transformMsgToEigen(temp_geo_trans , itsafine);

    uint8_t control_group = req.ControlGroup;

    XR1_ptr->setControlMode(control_group , XR1::IKMode);
    // The default response
    res.inProgress = true;
    res.isReachable = false;
    res.isAccepted = false;

    if (XR1_ptr->isIKPlannerActive(control_group))
    {
        res.inProgress = true;
    }

    else {
        if (req.NewTarget){
            res.inProgress = false;
            if (XR1_ptr->setEndEffectorPosition(control_group , itsafine , req.TargetElbowAngle , req.Period)){
                res.isReachable = true;
                res.isAccepted = true;

                XR1_ptr->setGrippingSwitch( control_group , req.Grip);
            }
        }

    }

    return true;

}

//-----------------------------------------------------------------------------

void broadcastTransform(const ros::TimerEvent& event) {



  // This function triggers almost all the computation in the library
  XR1_ptr->triggerCalculation();

  // Publish the left one
  XR1_ptr->getEndEffectorTransformation(XR1::LeftArm , itsafine);
  tf::transformEigenToTF(itsafine , tform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(tform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


  // Publish the right one
  XR1_ptr->getEndEffectorTransformation(XR1::RightArm , itsafine);
  // std::cout << itsafine.matrix() << std::endl;
  tf::transformEigenToTF(itsafine , tform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(tform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));



  XR1_ptr->getEndEffectorTransformation(XR1::MainBody , itsafine);
  tf::transformEigenToTF(itsafine , tform);
  EFF_Broadcaster->sendTransform(tf::StampedTransform(tform, ros::Time::now(), "/Back_Y", "/Head"));


  if (!(XR1_ptr->isIKPlannerActive(XR1::RightArm)))
      lookupRightEFFTarget(tform, itsafine);

  if (!(XR1_ptr->isIKPlannerActive(XR1::LeftArm)))
      lookupLeftEFFTarget(tform, itsafine);


  stateTransition();

}



// ----------------------------------------------------------------------------------
// ==================================================================================



// Look man you want the fk you gotta to feed me the angles
void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs& msg) {
  XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::LeftArm , XR1::ActualPosition);
}
void subscribeRightArmPosition(const xr1controllerros::ArmMsgs& msg) {
  XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::RightArm , XR1::ActualPosition);
}

void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs& msg) {
  XR1_ptr->updatingCallback(BodyMsgs2VectorXd(msg) , XR1::MainBody , XR1::ActualPosition);
}

void subscribeLeftHandPosition(const xr1controllerros::HandMsgs& msg) {
    XR1_ptr->updatingCallback(HandsMsgs2VectorXd (msg) , XR1::LeftHand , XR1::ActualPosition);
}
void subscribeRightHandPosition(const xr1controllerros::HandMsgs& msg) {
    XR1_ptr->updatingCallback(HandsMsgs2VectorXd(msg) , XR1::RightHand , XR1::ActualPosition);
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "IK_Simulator");

  ros::NodeHandle nh;


  std::string path = ros::package::getPath("xr1controllerol");

  XR1_ptr = new XR1ControllerPM(path + "/Turnips.xr1para");

  EFF_Broadcaster = new tf::TransformBroadcaster();
  EFF_Listener = new tf::TransformListener();

  // Feed me infos
  ros::Subscriber LeftArmPositionSubscriber   = nh.subscribe("/LeftArm/Position" ,  3 , subscribeLeftArmPosition);

  // Feed me more
  ros::Subscriber RightArmPositionSubscriber  = nh.subscribe("/RightArm/Position" ,  3 , subscribeRightArmPosition);


    // Feed me infos
    ros::Subscriber LeftHandPositionSubscriber   = nh.subscribe("/LeftHand/Position" ,  3 , subscribeLeftHandPosition);

    // Feed me more
    ros::Subscriber RightHandPositionSubscriber  = nh.subscribe("/RightHand/Position" ,  3 , subscribeRightHandPosition);


  ros::Subscriber MainBodyPositionSubscriber = nh.subscribe("/MainBody/Position" , 3 , subscribeMainBodyPosition);

  // More!!
  ros::Subscriber LeftArmModeChangeSubscriber                 = nh.subscribe("/XR1/LeftArmChainModeChange" , 1, subscribeLeftArmMode);

  //MOREEEEEEEE!
  ros::Subscriber RightArmModeChangeSubscriber                = nh.subscribe("/XR1/RightArmChainModeChange" , 1, subscribeRightArmMode);

  // MMMMMMOOOOORRRRRREEEEEEEE!
  ros::Subscriber LeftElbowSubscriber                         = nh.subscribe("LeftArm/ElbowAngle" , 1, subscribeLeftElbowAngle);

  // MMMMMMMMMOOOOOOOOOOOOOOOOAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEERRRRRRRRRRRRRRRRRRRRR!!!!!!!!!!!!!!!
  ros::Subscriber RightElbowSubscriber                        = nh.subscribe("RightArm/ElbowAngle" , 1, subscribeRightElbowAngle );


   ros::ServiceServer IKPlannerService = nh.advertiseService("XR1/IKLPT" ,  serviceIKPlanner);


  // cough out the target position as the result of IK;
  ros::Publisher LAPP  = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetPosition", 1);
  ros::Publisher RAPP  = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetPosition", 1);
    ros::Publisher LHPP  = nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/TargetPosition", 1);
    ros::Publisher RHPP  = nh.advertise<xr1controllerros::HandMsgs>("/RightHand/TargetPosition", 1);
  ros::Publisher MBPP  = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/TargetPosition", 1);
  LeftArmPositionPublisher    = &LAPP;
  RightArmPositionPublisher   = &RAPP;
  MainBodyPositionPublisher = &MBPP;
    LeftHandPositionPublisher = &LHPP;
    RightHandPositionPublisher = &RHPP;



  LeftElbowAngle   = 2.5;
  RightElbowAngle  = -2.5;


  // Draw some random stuff every three seconds or so
  ros::Timer timer = nh.createTimer(ros::Duration(0.005), broadcastTransform);


    XR1_ptr->setJointPosition(XR1::Neck_X , 0.5);





    // Basic testing ---------------------------------------------------------------------



//    VectorXd testy = VectorXd::Zero(7);
//    VectorXd shity = VectorXd::Zero(7);
//
//    testy << 0.805673833997187  ,  1.86216218006166  ,  0.795732612652139 ,   -0.172666403436580  ,  -0.410773351865951 ,   -2.11365969021934  ,  0.293479279663677;
//
//    shity << M_PI / 2, M_PI / 2 -0.3491,M_PI / 2,0,0,-M_PI / 2,0;
//
//    testy = testy + shity;
//
//    XR1_ptr->updatingCallback(testy, XR1::LeftArm , XR1::ActualPosition);
//
//    XR1_ptr->triggerCalculation();
//
//    Affine3d transform;
//
//    XR1_ptr->getEndEffectorTransformation(XR1::LeftArm , transform);
//
//    Quaterniond testqua;
//    testqua = transform.rotation();
//
//    std::cout << testqua.w() <<" "<< testqua.x() <<" "<< testqua.y() <<" "<< testqua.z() << std::endl;
//
//    std::cout << transform.translation().x() <<" "<<transform.translation().y() <<" "<<transform.translation().z() << std::endl;



    //    XR1_ptr->setControlMode(XR1::LeftArm , XR1::IKMode);
    //    Affine3d transform;

    //    transform.matrix() <<   0.172364,-0.0596322,  0.983226, 0.0195315,
    //                            0.379472,  0.925144, -0.010413,   0.11014,
    //                           -0.909005,  0.374902,   0.18209,    0.2695,
    //                                   0,         0,         0,         1;

    //    XR1_ptr->setEndEffectorPosition(XR1::LeftArm , transform , 3.0);

    // -----------------------------------------------------------------------------------



  ros::spin();


}