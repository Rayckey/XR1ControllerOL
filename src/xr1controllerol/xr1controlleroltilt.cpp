//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"

void XR1ControllerOL::subscribeTiltStart(const std_msgs::Bool &msg) {


    if (msg.data){


        for (uint8_t control_group : control_group_flags){

//            if (XR1_ptr->getSubControlMode(control_group) != XR1::IKMode){

            setControlMode(control_group , XR1::DriveMode);

            ROS_INFO("Drive  for Control Group [%d] , is now ON",  (int)control_group );

//            }

        }

        setControlMode(XR1::OmniWheels , XR1::RoamMode);

    }


    else {


        for (uint8_t control_group : control_group_flags){

//            if (XR1_ptr->getSubControlMode(control_group) == XR1::AnimationMode){

            setControlMode(control_group , XR1::DirectMode);

            ROS_INFO("Drive for Control Group [%d] , is now OFF",  (int)control_group );

//            }

        }

        setControlMode(XR1::OmniWheels , XR1::DirectMode);

        clearStates();
    }




}



void XR1ControllerOL::subscribeSLAMStart(const std_msgs::Bool &msg){

    if (msg.data){


        for (uint8_t control_group : control_group_flags){

//            if (XR1_ptr->getSubControlMode(control_group) != XR1::IKMode){

            setControlMode(control_group , XR1::DriveMode);

            ROS_INFO("Drive  for Control Group [%d] , is now ON",  (int)control_group );

//            }

        }

        setControlMode(XR1::OmniWheels , XR1::VelocityMode);

    }


    else {


        for (uint8_t control_group : control_group_flags){

//            if (XR1_ptr->getSubControlMode(control_group) == XR1::AnimationMode){

            setControlMode(control_group , XR1::DirectMode);

            ROS_INFO("Drive for Control Group [%d] , is now OFF",  (int)control_group );

//            }

        }

        setControlMode(XR1::OmniWheels , XR1::DirectMode);

        clearStates();
    }


}



bool XR1ControllerOL::serviceQueryBalance(xr1controllerol::BalanceQueryRequest &req,
                         xr1controllerol::BalanceQueryResponse &res){


    if (req.isQuery){
        bool hasIdle, hasActive, hasPassive;

        XRB_ptr->getOptions(hasIdle , hasActive , hasPassive);

        res.hasIdle = hasIdle;

        res.hasActive = hasActive;

        res.hasPassive = hasPassive;
    }


    return true;
}

void XR1ControllerOL::subscribeBLCIdle(const std_msgs::Bool &msg){
    XRB_ptr->setIdle(msg.data);
}
void XR1ControllerOL::subscribeBLCActive(const std_msgs::Bool &msg){
    XRB_ptr->setActive(msg.data);
}
void XR1ControllerOL::subscribeBLCPassive(const std_msgs::Bool &msg){
    XRB_ptr->setPassive(msg.data);
}
