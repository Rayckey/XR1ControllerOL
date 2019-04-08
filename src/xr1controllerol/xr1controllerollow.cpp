//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"





void XR1ControllerOL::setControlGroupTarget(uint8_t control_group){

    if (XR1_ptr->isXR1Okay()){


        switch (XR1_ptr->getControlMode(control_group)){

            case XR1::PositionMode :

                XR1_ptr->getTargetPosition(control_group , temp_vec7d);

                for (uint8_t joint_id : control_group_map[control_group])
                    m_pController->setPosition(joint_id , temp_vec7d(joint_id - control_group));

                break;

            case XR1::VelocityMode :

                XR1_ptr->getTargetVelocity(control_group , temp_vec7d);

                for (uint8_t joint_id : control_group_map[control_group])
                    m_pController->setVelocity(joint_id , temp_vec7d(joint_id - control_group));

                break;

            case XR1::ForceMode :

                XR1_ptr->getTargetCurrent(control_group , temp_vec7d);

                for (uint8_t joint_id : control_group_map[control_group])
                    m_pController->setCurrent(joint_id , temp_vec7d(joint_id - control_group));

                break;

            default:
                break;
        }

    }

}




double XR1ControllerOL::getTargetJointPosition(uint8_t joint_id, bool vanilla) {

    return XR1_ptr->getTargetJointPosition(joint_id, vanilla);

}

