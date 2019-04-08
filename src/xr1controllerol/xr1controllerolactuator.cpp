//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"



void XR1ControllerOL::launchAllMotors() {

    m_pController->launchAllActuators();
    if (allActuatorHasLaunched()) {
        XR1_ptr->setInverseDynamicsOption(XR1::GravityCompensation);
    }

}

bool XR1ControllerOL::allActuatorHasLaunched() {
    vector<uint8_t> idArray = m_pController->getActuatorIdArray();
    for (int i = 0; i < idArray.size(); ++i) {
        if (m_pController->getActuatorAttribute((uint8_t) idArray.at(i), Actuator::INIT_STATE) != Actuator::Initialized)
            return false;
    }
    return true;
}



void XR1ControllerOL::stopAllMotors() {

    std::vector<uint8_t> IDArray = m_pController->getActuatorIdArray();

    for (int i = 0; i < IDArray.size(); i++) {
        m_pController->closeActuator(IDArray.at(i));
    }

}





void XR1ControllerOL::setSubControlMode(uint8_t control_group , uint8_t option){
    XR1_ptr->setSubControlMode(control_group, option);
}



void XR1ControllerOL::updatingCallback(uint8_t id, uint8_t attrId, double value) {
    if (attribute_map.find(attrId) != attribute_map.end())
        XR1_ptr->updatingCallback(id, attribute_map[attrId], value);
}