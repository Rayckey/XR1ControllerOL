//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"


void XR1ControllerOL::Omni2Actuator() {

    if (XR1_ptr->getSubControlMode(XR1::OmniWheels) == XR1::RoamMode){

        XRA_ptr->employFilteredOmniCmd();

        XR1_ptr->getTargetVelocity(XR1::OmniWheels , temp_vec3d);

        for (uint8_t i = XR1::OmniWheels ; i < XR1::MainBody ; i++) {
            m_pController->setVelocity(i , temp_vec3d(i - XR1::OmniWheels) );
//        qDebug() << "LF: " << temp_cmd(0) << "RF: " << temp_cmd(1) << "BK: " << temp_cmd(2);
            // simulation call -------------------------------------------------------------
//            XR1_ptr->updatingCallback(i , XR1::ActualVelocity,temp_cmd(i-XR1::OmniWheels));
            // -----------------------------------------------------------------------------
        }

    }

}


void XR1ControllerOL::subscribeOmniCommands(const geometry_msgs::Twist & msg){

    if (XR1_ptr->getSubControlMode(XR1::OmniWheels)){
        temp_omni_cmd << msg.angular.x , msg.linear.y , msg.linear.z ;

        XRA_ptr->setUnfilteredOmniCmd(temp_omni_cmd);
    }

}

