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



void XR1ControllerOL::setControlMode(uint8_t control_group, uint8_t option) {


    if (control_modes.find(control_group) == control_modes.end()){
        ROS_INFO("Wrong input received for Control Mode");
        return;
    }


    if (control_modes[control_group] == option){
    }

    else {


        if (high_frequency_switch){


            if (control_group == XR1::HeadBody || control_group == XR1::MainBody){
                XR1_ptr->setControlMode(control_group , option);
                control_modes[control_group] = option;
            }




        }

        else {

            ROS_INFO("Setting Control Group [%d] to Mode [%d]" , control_group , option);

            control_modes[control_group] = option;

            XR1_ptr->setControlMode(control_group , option);

            if (control_group_map.find(control_group) != control_group_map.end()){
                std::vector<uint8_t> temp_vector = control_group_map[control_group];

                for (uint8_t id : temp_vector) {
                    if ((int) m_pController->getActuatorAttribute(id, Actuator::INIT_STATE) == Actuator::Initialized) {
                        m_pController->activateActuatorMode(id, mode_map[option]);
                    }
                }
            }
        }

    }

}


void XR1ControllerOL::setSubControlMode(uint8_t control_group , uint8_t option){
    XR1_ptr->setSubControlMode(control_group, option);
}



void XR1ControllerOL::updatingCallback(uint8_t id, uint8_t attrId, double value) {
    if (attribute_map.find(attrId) != attribute_map.end())
        XR1_ptr->updatingCallback(id, attribute_map[attrId], value);
}