//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"


void XR1ControllerOL::QuaCallBack(uint64_t id, double w, double x, double y, double z) {


    // if (precision > 1){

    // If it is the base frame
    if (id == ActuatorController::toLongId("192.168.1.4", 0)){
//        ROS_INFO("[%f][%f][%f][%f]",w,x,y,z);
        XRB_ptr->tiltCallback(w, x, y, z);
    }



}


void XR1ControllerOL::requestQue() {
    // get the sweet sweet bottom IMU reading
    m_pController->requestSingleQuaternion(ActuatorController::toLongId("192.168.1.4" , 0));
}


void XR1ControllerOL::subscribeTiltStart(const std_msgs::Bool &msg) {


    if (msg.data){

        xr1controller_mutex.lock();
        for (uint8_t control_group : control_group_flags){

//            if (XR1_ptr->getSubControlMode(control_group) != XR1::IKMode){

            XR1_ptr->setSubControlMode(control_group , XR1::DriveMode);

            ROS_INFO("Drive  for Control Group [%d] , is now ON",  (int)control_group );

//            }

        }

        XR1_ptr->setSubControlMode(XR1::OmniWheels , XR1::RoamMode);
        xr1controller_mutex.unlock();
    }


    else {

        xr1controller_mutex.lock();
        for (uint8_t control_group : control_group_flags){

//            if (XR1_ptr->getSubControlMode(control_group) == XR1::AnimationMode){

            XR1_ptr->setSubControlMode(control_group , XR1::DirectMode);

            ROS_INFO("Drive for Control Group [%d] , is now OFF",  (int)control_group );

//            }

        }

        XR1_ptr->setSubControlMode(XR1::OmniWheels , XR1::DirectMode);

        clearStates();
        xr1controller_mutex.unlock();
    }




}

 void XR1ControllerOL::accCallBack(uint8_t id , double x , double y , double z , int pres){
//   ROS_INFO("[%d][%f][%f][%f]",pres,x,y,z);
 }

