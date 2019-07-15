//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"


void XR1ControllerOL::Omni2Actuator() {


    // if we're using ALP's profile velocity
    if (XR1_ptr->getSubControlMode(XR1::OmniWheels) == XR1::RoamMode){

        XR1_ptr->getTargetVelocity(XR1::OmniWheels , temp_vec3d);

        for (uint8_t i = XR1::OmniWheels ; i < XR1::MainBody ; i++) {
            m_pController->setVelocity(i , temp_vec3d(i - XR1::OmniWheels) );
//            ROS_INFO("LF: [%f] , RF: [%f] , BK: [%f] " , temp_vec3d(0) ,temp_vec3d(1) ,temp_vec3d(2)) ;
            // simulation call -------------------------------------------------------------
//            XR1_ptr->updatingCallback(i , XR1::ActualVelocity,temp_vec3d(i-XR1::OmniWheels));
            // -----------------------------------------------------------------------------
        }

        // counting the seoonds you left me standing
        if (fabs(temp_vec3d(0)) + fabs(temp_vec3d(1)) + fabs(temp_vec3d(2)) < 10){
            previous_omni_state++;
        }

        else {
            previous_omni_state = 0;
        }

        if (previous_omni_state > 2000){
            setControlMode(XR1::OmniWheels , XR1::DirectMode);
            previous_omni_state = 0;
        }


        // counting the seconds before you pass me by
        omni_cmd_expire_counter++;

        if (omni_cmd_expire_counter > 1000){
            omni_cmd_expire_counter = 0;
            temp_omni_cmd << 0,0,0;
            XRA_ptr->setTargetOmniCmd(temp_omni_cmd);
        }

    }


    // if we're using brute velocity mode
    else if (XR1_ptr->getSubControlMode(XR1::OmniWheels) == XR1::VelocityMode){

        XR1_ptr->getTargetVelocity(XR1::OmniWheels , temp_vec3d);

        // simulation call -------------------------------------------------------------
        ROS_INFO("LF: [%f] , RF: [%f] , BK: [%f] " , temp_vec3d(0) ,temp_vec3d(1) ,temp_vec3d(2)) ;
        // -----------------------------------------------------------------------------

        for (uint8_t i = XR1::OmniWheels ; i < XR1::MainBody ; i++) {
            m_pController->setVelocity(i , temp_vec3d(i - XR1::OmniWheels) );
        }
    }

}


void XR1ControllerOL::subscribeOmniCommands(const geometry_msgs::Twist & msg){

    if (XR1_ptr->getSubControlMode(XR1::OmniWheels) == XR1::RoamMode){
        temp_omni_cmd << msg.angular.x , msg.linear.y , msg.linear.z ;

        omni_cmd_expire_counter = 0;

        ROS_INFO(" New Omni Command Received [%f] [%f] [%f]" , temp_omni_cmd(0) ,temp_omni_cmd(1) ,temp_omni_cmd(2)) ;


        XRA_ptr->setTargetOmniCmd(temp_omni_cmd);
    }

    else{
        temp_omni_cmd << msg.angular.x , msg.linear.y , msg.linear.z ;
        ROS_INFO(" New Omni Command Received [%f] [%f] [%f]" , temp_omni_cmd(0) ,temp_omni_cmd(1) ,temp_omni_cmd(2)) ;
        XR1_ptr->SetOmniWheelsVelocity(temp_omni_cmd);
    }

}

void XR1ControllerOL::publishOmni() {

    // Publish the current Omni wheels velocity
    if (XR1_ptr->getSubControlMode(XR1::OmniWheels) > XR1::DirectMode){

        XR1_ptr->getOmniWheelsVelocity(temp_vec);

        geometry_msgs::Twist temp_twist;

        temp_twist.linear.y = temp_vec(1);

        temp_twist.linear.z = temp_vec(2);

        temp_twist.angular.x = temp_vec(0);


        OmniSpeedPublisher.publish(temp_twist);
    }


//    // broadcast the current omni position
//    XR1_ptr->getOmniWheelsPosition(temp_vec);
//
//    transform.setRotation(tf::Quaternion(temp_vec(0),0,0));
//
//    transform.setOrigin(tf::Vector3(0,temp_vec(1),temp_vec(2)));
//
//    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Omni", "/Knee"));
}