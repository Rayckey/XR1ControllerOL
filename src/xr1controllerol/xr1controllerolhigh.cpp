//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"




void XR1ControllerOL::judgeControlGroupModes(){


    judgeActuatorModes(XR1::MainBody);
    judgeActuatorModes(XR1::LeftArm);
    judgeActuatorModes(XR1::RightArm);
//    judgeActuatorModes(XR1::LeftHand);
//    judgeActuatorModes(XR1::RightHand);

}

void XR1ControllerOL::judgeActuatorModes(uint8_t control_group){



    std::vector<uint8_t> temp_vector = control_group_map[control_group];


    if (XR1_ptr->getControlMode(control_group) == XR1::PositionMode || XR1_ptr->getControlMode(control_group) == XR1::IKMode){

        for (uint8_t id : temp_vector) {
            if ((int) m_pController->getActuatorAttribute(id, Actuator::INIT_STATE) == Actuator::Initialized) {

                if (high_frequency_switch)
                    m_pController->activateActuatorMode(id, ActuatorMode::Mode_Pos);
                else
                    m_pController->activateActuatorMode(id, mode_map[XR1::PositionMode]);
            }
        }
    }

    else if (XR1_ptr->getControlMode(control_group) == XR1::VelocityMode){

        for (uint8_t id : temp_vector) {
            if ((int) m_pController->getActuatorAttribute(id, Actuator::INIT_STATE) == Actuator::Initialized) {

                if (high_frequency_switch)
                    m_pController->activateActuatorMode(id, ActuatorMode::Mode_Vel);
                else
                    m_pController->activateActuatorMode(id, mode_map[XR1::VelocityMode]);
            }
        }

    }

    else if (XR1_ptr->getControlMode(control_group) == XR1::ForceMode){

        for (uint8_t id : temp_vector) {
            if ((int) m_pController->getActuatorAttribute(id, Actuator::INIT_STATE) == Actuator::Initialized) {

                if (high_frequency_switch)
                    m_pController->activateActuatorMode(id, ActuatorMode::Mode_Cur);
                else
                    m_pController->activateActuatorMode(id, mode_map[XR1::ForceMode]);
            }
        }

    }


}




void XR1ControllerOL::stateTransition(){

    std::vector<double> state_cmd = XR1_ptr->getNextState();

    if (state_cmd[0] < 0.5){

        // set the modes , if they are the same it will not affect the actuators
        switch2HighFrequency(true);

        applyJointTarget();
    }

    else {
        switch2HighFrequency(false);
    }

}


void XR1ControllerOL::switch2HighFrequency(bool option) {

    if (high_frequency_switch == option){
        // do nothing
    }

    else {
        high_frequency_switch = option;

        ROS_INFO("High Frequency mode set to [%d]" , high_frequency_switch);
        judgeControlGroupModes();
    }

}

void XR1ControllerOL::setJointTarget(uint8_t joint_id){

    switch (XR1_ptr->getControlMode(joint_id)){

        case XR1::PositionMode:

            m_pController->setPosition(joint_id , XR1_ptr->getTargetJointPosition(joint_id));

            break;


        case XR1::VelocityMode:

            m_pController->setVelocity(joint_id , XR1_ptr->getTargetJointVelocity(joint_id));

            break;

        case XR1::ForceMode:

            m_pController->setCurrent(joint_id , XR1_ptr->getTargetJointCurrent(joint_id));

            break;

        default:

            break;

    }

}

void XR1ControllerOL::applyJointsTargets(){

    for (uint8_t control_group : control_group_flags){

        if (XR1_ptr->inHighFrequencyControl(control_group) && XR1_ptr->isXR1Okay()){

            temp_ids = XR1_ptr->getControlGroupIDs(control_group);

            if (XR1_ptr->getSubControlMode(control_group) == XR1::AnimationMode){

            }

            else {
                for (uint8_t id : temp_ids)
                    temp_value = XR1_ptr->getNextState(id);

            }


            setControlGroupTarget(control_group);
        }
    }
}




void XR1ControllerOL::gravityCallback() {


    if (XR1_ptr->getControlMode(XR1::LeftArm) == XR1::ForceMode) {
        for (uint8_t i = XR1::LeftArm; i < XR1::Left_Wrist_Z; i++) {
            ROS_INFO("The Current for joint [%d] is [%f]", (int) i, XR1_ptr->getTargetJointCurrent(i));

//            setJointCurrent(i, XR1_ptr->getTargetJointCurrent(i));

        }
    }
//
    if (XR1_ptr->getControlMode(XR1::RightArm) == XR1::ForceMode) {
        for (uint8_t i = XR1::RightArm; i < XR1::Right_Wrist_Z; i++) {
            ROS_INFO("The Current for joint [%d] is [%f]", (int) i, XR1_ptr->getTargetJointCurrent(i));

//            setJointCurrent(i, XR1_ptr->getTargetJointCurrent(i));

        }
    }


}
