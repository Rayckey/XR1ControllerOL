//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"


//void XR1ControllerOL::QuaCallBack(uint64_t id, double w, double x, double y, double z) {
//
//    // ROS_INFO("[%f][%f][%f][%f]",w,x,y,z);
//    // if (precision > 1){
//
//    // If it is the base frame
//    if (id == ActuatorController::toLongId("192.168.1.4", 0))
//        XR1_ptr->tiltCallback(w, x, y, z);
//
//        // If it is a MoCap module
//    else {
//        IMU_ptr->quaternioncallback(ActuatorController::toByteId(id), w, x, y, z);
//    }
//
//    // }
//
//}


//void XR1ControllerOL::requestAcc(const ros::TimerEvent &) {
//    m_pController->requestSingleQuaternion(ActuatorController::toLongId("192.168.1.4", 0));
//    // m_pController->requestSingleQuaternion();
//}
//
//void XR1ControllerOL::requestQue(const ros::TimerEvent &) {
//    m_pController->requestAllQuaternions();
//}


//void XR1ControllerOL::MoCapCallback(const ros::TimerEvent &) {
//
////    if (XR1_ptr->getMetaMode() == XR1::MoCapMode) {
////
////        std::vector<double> temp_vec = IMU_ptr->getJointAngles();
////
////        XR1_ptr->setMoCapPosition(IMU_ptr->getJointAngles());
////
////        XR1_ptr->getTargetPosition(XR1::LeftArm , temp_vec7d);
////
////        setJointPosition(XR1::LeftArm, temp_vec7d);
////
////    }
//
//}


// void XR1ControllerOL::accCallBack(uint8_t id , double x , double y , double z , int pres){
//  // ROS_INFO("[%d][%f][%f][%f]",pres,x,y,z);
// }



//void XR1ControllerOL::subscribetiltInit(const std_msgs::Bool &msg) {
//    XR1_ptr->tiltInit();
//}
//
//void XR1ControllerOL::subscribeMoCapInit(const std_msgs::Bool &msg) {
//    ROS_INFO("Here be initialized");
////    XR1_ptr->setMetaMode(XR1::MoCapMode);
//    IMU_ptr->Initialize();
//}