//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"


void XR1ControllerOL::QuaCallBack(uint64_t id, double w, double x, double y, double z) {

     ROS_INFO("[%f][%f][%f][%f]",w,x,y,z);
    // if (precision > 1){

    // If it is the base frame
//    if (id == ActuatorController::toLongId("192.168.1.4", 6))
//        XRB_ptr->tiltCallback(w, x, y, z);


}


void XR1ControllerOL::requestQue() {
    m_pController->requestAllQuaternions();
}


void XR1ControllerOL::subscribetiltInit(const std_msgs::Bool &msg) {
    XRB_ptr->tiltInit();
}
