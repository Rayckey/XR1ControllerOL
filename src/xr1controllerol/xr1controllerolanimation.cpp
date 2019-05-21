//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"

// receive animation start signal
void XR1ControllerOL::subscribeStartAnimation(const std_msgs::Bool& msg){


    if (msg.data){


        for (uint8_t control_group : control_group_flags){

//            if (XR1_ptr->getSubControlMode(control_group) != XR1::IKMode){

                setControlMode(control_group , XR1::AnimationMode);

                ROS_INFO("Animation for Control Group [%d] , is now ON",  (int)control_group );

//            }

        }

    }


    else {


        for (uint8_t control_group : control_group_flags){

//            if (XR1_ptr->getSubControlMode(control_group) == XR1::AnimationMode){

                setControlMode(control_group , XR1::DirectMode);

                ROS_INFO("Animation for Control Group [%d] , is now OFF",  (int)control_group );

//            }

        }

        XRA_ptr->setSingleTransitionPeriod(3);

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


void XR1ControllerOL::subscribeSetIdle(const std_msgs::Bool &msg) {
    XRA_ptr->setIdleOption(msg.data);
}

bool XR1ControllerOL::serviceQueryAnimation(xr1controllerol::AnimationQueryRequest &req,
                                            xr1controllerol::AnimationQueryResponse &res){

    int type_id , ani_id, pro_id;

    if (XRA_ptr->checkProgress(type_id,ani_id , pro_id) || XR1_ptr->getSubControlMode(XR1::HeadBody) != XR1::AnimationMode){
        res.isIdle = false;
        res.AnimationType = type_id;
        res.AnimationID = ani_id;
        res.AnimationProgress = pro_id;
    }

    else {
        res.isIdle = true;
    }



    return true;

}


void XR1ControllerOL::subscribeSetCollisionDetection(const std_msgs::Bool & msg){



    // we want to turn it on
    if (msg.data){



        for (uint8_t control_group : control_group_flags){
            if (XR1_ptr->getSubControlMode(control_group) >= XR1::TeachMode){

                ROS_INFO("Collision detection is unable to activate");

                return ;
            }
        }

        ROS_INFO("Set collision detection to ON");

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


void XR1ControllerOL::collisionDetectionCallback(){

    // MAKE SURE THE COLLISION DETECTION IS ON
    if (XR1_ptr->getInverseDynamicsOption() >= XR1::FullDynamics){

        if (collision_detection_switch) {
            if (XR1_ptr->CollisionDetection(XR1::LeftArm) ) {

                ROS_INFO("Collision Occured");

                for (uint8_t control_group : control_group_flags){
                    setControlMode(control_group , XR1::DirectMode);
                }
                setControlMode(XR1::OmniWheels , XR1::DirectMode);

                XR1_ptr->employLockdown();
                XRA_ptr->clearAll();

                std_msgs::Bool msg;
                msg.data = true;

                CollisionEventPublisher.publish(msg);

//                collision_detection_switch = false;

                XR1_ptr->liftLockdown();
                XR1_ptr->setInverseDynamicsOption(XR1::FullDynamics_PASSIVE);



                return;

            }


            if (XR1_ptr->CollisionDetection(XR1::RightArm)){

                ROS_INFO("Collision Occured");

                for (uint8_t control_group : control_group_flags){
                    setControlMode(control_group , XR1::DirectMode);
                }
                setControlMode(XR1::OmniWheels , XR1::DirectMode);

                XR1_ptr->employLockdown();
                XRA_ptr->clearAll();


                std_msgs::Bool msg;
                msg.data = true;

                CollisionEventPublisher.publish(msg);

//                collision_detection_switch = false;

                XR1_ptr->liftLockdown();
                XR1_ptr->setInverseDynamicsOption(XR1::FullDynamics_PASSIVE);



                return;

            }
        }
    }



    // if it is not, turn collision detection off
    else {
        collision_detection_switch = false;
    }

}