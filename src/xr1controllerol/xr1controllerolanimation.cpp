//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"

// receive animation start signal
void XR1ControllerOL::subscribeStartAnimation(const std_msgs::Bool& msg){


    if (msg.data){


        for (uint8_t control_group : control_group_flags){

            if (XR1_ptr->getSubControlMode(control_group) == XR1::DirectMode){
                setControlMode(control_group , XR1::AnimationMode);

                animation_switch = true;
            }

            if (animation_switch)
                ROS_INFO("Animation is now ON");

        }

    }


    else {

        animation_switch = false;
        ROS_INFO("Animation is now OFF");
        clearStates();
    }

}


void XR1ControllerOL::clearStates() {
    XRA_ptr->clearAll();
    XR1_ptr->clearStates();
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



    // we want to turn it on
    if (msg.data){
        ROS_INFO("Set collision detection to ON");


        for (uint8_t control_group : control_group_flags){
            if (XR1_ptr->getSubControlMode(control_group) >= XR1::TeachMode){
                return ;
            }
        }

        collision_detection_switch = msg.data;

        XR1_ptr->setInverseDynamicsOption(XR1::FullDynamics_PASSIVE);
    }

    // we want to turn it off
    else{
        collision_detection_switch = msg.data;
        ROS_INFO("Set collision detection to OFF");
        XR1_ptr->setInverseDynamicsOption(XR1::GravityCompensation);
    }


    // decide if we need to lift a lock down or two
    if (XR1_ptr->isXR1Okay()){

    }

    else {
        if (collision_detection_switch){

        }

        else {

            // lift the curse on thy princess eh?
            XR1_ptr->liftLockdown();

        }
    }
}


// animation main loop
void XR1ControllerOL::animationCallback(){

        XRA_ptr->getNextState();

//        if (previous_omni_state != XRA_ptr->isOmniWheelsMoving()){
//            previous_omni_state = XRA_ptr->isOmniWheelsMoving();
//            activateOmni();
//        }
//
//
//        if (previous_omni_state){
//            Omni2Actuator();
//        }

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

    // MAKE SURE THE COLLISION DETECTION IS ON
    if (XR1_ptr->getInverseDynamicsOption() >= XR1::FullDynamics){

        if (collision_detection_switch) {
            if (XR1_ptr->CollisionDetection(XR1::LeftArm) ) {

                ROS_INFO("Collision Occured");

                XR1_ptr->employLockdown();
                XRA_ptr->clearAll();
                animation_switch = false;
                collision_detection_switch = false;
                return;

            }


            if (XR1_ptr->CollisionDetection(XR1::RightArm)){

                ROS_INFO("Collision Occured");

                XR1_ptr->employLockdown();
                XRA_ptr->clearAll();
                animation_switch = false;
                collision_detection_switch = false;
                return;

            }
        }
    }

    // if it is not, turn collision detection off
    else {
        collision_detection_switch = false;
    }




}