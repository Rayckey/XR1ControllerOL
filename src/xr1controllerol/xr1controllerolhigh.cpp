//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"


void XR1ControllerOL::setJointTarget(uint8_t joint_id){

    switch (XR1_ptr->getControlMode(joint_id)){

        case XR1::PositionMode:

            m_pController->setPosition(joint_id , XR1_ptr->getTargetJointPosition(joint_id));

            break;


        case XR1::VelocityMode:

            m_pController->setVelocity(joint_id , XR1_ptr->getTargetJointVelocity(joint_id));

            break;

        case XR1::ForceMode:

            ROS_INFO("The Current for joint [%d] is [%f]", joint_id, XR1_ptr->getTargetJointCurrent(joint_id));
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
//                for (uint8_t id : temp_ids)
//                    temp_value = XR1_ptr->getNextState(id);

            }


            setControlGroupTarget(control_group);
        }
    }
}