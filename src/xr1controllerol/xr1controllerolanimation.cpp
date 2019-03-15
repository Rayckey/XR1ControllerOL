//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"

// receive animation start signal
void XR1ControllerOL::subscribeStartAnimation(const std_msgs::Bool& msg){
    animation_switch = msg.data;

    if (animation_switch)
        ROS_INFO("Animation is now ON");

    else {
        ROS_INFO("Animation is now OFF");
        XRA_ptr->clearAll();
        switch2HighFrequency(false);
    }

}


// receive an animation order
void XR1ControllerOL::subscribeSetAnimation(const xr1controllerol::AnimationMsgs& msg){

    if (msg.AnimationType)
        XRA_ptr->setAnimation(msg.AnimationType , msg.AnimationID);

    else {
//        ROS_INFO("Number of Animation Left: %d " , XRA_ptr->popAnimation());
    }
}


void XR1ControllerOL::subscribeSetCollisionDetection(const std_msgs::Bool & msg){
    collision_detection_switch = msg.data;

    if (collision_detection_switch){
        ROS_INFO("Set collision detection to ON");
        XR1_ptr->setInverseDynamicsOption(XR1::FullDynamics_PASSIVE);
    }


    else{
        ROS_INFO("Set collision detection to OFF");
        XR1_ptr->setInverseDynamicsOption(XR1::GravityCompensation);
    }



    if (XR1_ptr->isXR1Okay()){

    }

    else {
        if (collision_detection_switch){

        }

        else {

            // Im guessing you want to lift the curse on thy princess eh?
            XR1_ptr->liftLockdown();
            switch2HighFrequency(false);

        }
    }
}


// animation main loop
void XR1ControllerOL::animationCallback(){


        switch2HighFrequency(true);

        XRA_ptr->getNextState();

        applyJointTarget();


        if (previous_omni_state != XRA_ptr->isOmniWheelsMoving()){
            previous_omni_state = XRA_ptr->isOmniWheelsMoving();
            activateOmni();
        }


        if (previous_omni_state){
            Omni2Actuator();
        }



}


// change the mode of omniwheels
void XR1ControllerOL::activateOmni(){
    if (previous_omni_state){
        for (uint8_t i = XR1::OmniWheels; i < XR1::MainBody ; i ++)
            m_pController->activateActuatorMode(i , Actuator::Mode_Vel);
    }

    else{
        for (uint8_t i = XR1::OmniWheels; i < XR1::MainBody ; i ++)
            m_pController->activateActuatorMode(i , Actuator::Mode_Pos);
    }
}

// apply velocities
void XR1ControllerOL::Omni2Actuator(){

    XR1_ptr->getTargetVelocity(XR1::OmniWheels , temp_vec3d);

    for (uint8_t i = XR1::OmniWheels ; i < XR1::MainBody ; i++) {
        m_pController->setVelocity(i , temp_vec3d(i - XR1::OmniWheels) );
    }

}

void XR1ControllerOL::collisionDetectionCallback(){

    if (collision_detection_switch) {
        if (XR1_ptr->CollisionDetection(XR1::LeftArm) ) {

            ROS_INFO("Collision Occured");

            switch2HighFrequency(false);
            switch2HighFrequency(true);


            XR1_ptr->employLockdown();
            XRA_ptr->clearAll();
            animation_switch = false;
            collision_detection_switch = false;
            return;

        }


        if (XR1_ptr->CollisionDetection(XR1::RightArm)){

            ROS_INFO("Collision Occured");

            switch2HighFrequency(false);
            switch2HighFrequency(true);


            XR1_ptr->employLockdown();
            XRA_ptr->clearAll();
            animation_switch = false;
            collision_detection_switch = false;
            return;

        }
    }

}