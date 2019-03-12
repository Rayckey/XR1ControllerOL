//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"




void XR1ControllerOL::setJointPosition(uint8_t control_group, VectorXd & JA) {


    if (XR1_ptr->isXR1Okay()) {

        std::vector<uint8_t> temp_vector = control_group_map[control_group];

        for (uint8_t id : temp_vector) {
            if ((int) m_pController->getActuatorAttribute(id, Actuator::INIT_STATE) == Actuator::Initialized) {
                m_pController->setPosition(id, JA(id - control_group));
            }
        }

    }


}


double XR1ControllerOL::getTargetJointPosition(uint8_t joint_id, bool vanilla) {

    return XR1_ptr->getTargetJointPosition(joint_id, vanilla);

}

void XR1ControllerOL::setJointVelocity(uint8_t control_group, VectorXd & JV) {

    if (XR1_ptr->isXR1Okay()) {
        std::vector<uint8_t> temp_vector = control_group_map[control_group];

        for (uint8_t id : temp_vector) {
            if ((int) m_pController->getActuatorAttribute(id, Actuator::INIT_STATE) == Actuator::Initialized) {
                m_pController->setVelocity(id, JV(id - control_group));
            }
        }
    }

}

void XR1ControllerOL::setJointCurrent(uint8_t control_group, VectorXd & JC) {

    if (XR1_ptr->isXR1Okay()) {
        std::vector<uint8_t> temp_vector = control_group_map[control_group];

        for (uint8_t id : temp_vector) {
            if ((int) m_pController->getActuatorAttribute(id, Actuator::INIT_STATE) == Actuator::Initialized) {
                m_pController->setCurrent(id, JC(id - control_group));
            }
        }
    }

}


void XR1ControllerOL::setJointCurrent(uint8_t joint_idx, double JC) {

    if ((int) m_pController->getActuatorAttribute(joint_idx, Actuator::INIT_STATE) == Actuator::Initialized) {
        if (XR1_ptr->isXR1Okay()) {
            m_pController->setCurrent(joint_idx, JC);
        }
    }
}
