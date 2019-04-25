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
    if (attribute_map.find(attrId) != attribute_map.end()){
        XR1_ptr->updatingCallback(id, attribute_map[attrId], value);

//        if (id == XR1::Left_Elbow_X && attrId == Actuator::ACTUAL_POSITION){
//            std::cout << value << std::endl;
//        }
    }

}



void XR1ControllerOL::actuatorOperation(uint8_t nId, uint8_t nType) {

    switch (nType) {
        case Actuator::Recognize_Finished:
            if (m_pController->hasAvailableActuator()) {
                ROS_INFO("Recognized Actuators");
                RecognizeFinished = true;
            }
            break;
        case Actuator::Launch_Finished:
            if (allActuatorHasLaunched()) {

//            setControlMode(XR1::OmniWheels, XR1::DirectMode);
                setControlMode(XR1::MainBody, XR1::DirectMode);
                setControlMode(XR1::HeadBody, XR1::DirectMode);
                setControlMode(XR1::LeftArm, XR1::DirectMode);
                setControlMode(XR1::RightArm, XR1::DirectMode);
                setControlMode(XR1::LeftHand, XR1::DirectMode);
                setControlMode(XR1::RightHand, XR1::DirectMode);

                ROS_INFO("All Actuators Have Launched");
            }

            break;
        default:
            break;
    }
}

void XR1ControllerOL::readingCallback() {

    // ROS_INFO(" the current time is [%f]" , (float)this_event.current_real);

    for (uint8_t i = XR1::LeftArm; i < XR1::RightArm; ++i) {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {
            m_pController->getCVPValue(i);
        }
    }

    for (uint8_t i = XR1::RightArm; i < XR1::LeftHand; ++i) {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {
            m_pController->getCVPValue(i);
        }
    }

    for (uint8_t i = XR1::Back_Z; i <= XR1::Back_Y; ++i) {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {
            m_pController->getCVPValue(i);
        }
    }

    for (uint8_t i = XR1::Neck_Z; i < XR1::LeftArm; ++i) {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {
            m_pController->getCVPValue(i);
        }
    }

    if (hand_command_switch) {
        for (uint8_t i = XR1::LeftHand; i < XR1::Actuator_Total; ++i) {
            if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {
                m_pController->regainAttrbute(i, Actuator::ACTUAL_POSITION);
                m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
            }
        }


        for (uint8_t i = XR1::OmniWheels; i < XR1::MainBody; ++i)
        {
            if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized)
            {
                m_pController->getCVPValue(i);
            }
        }


        if ((int) m_pController->getActuatorAttribute((uint8_t) XR1::Knee_X, Actuator::INIT_STATE) ==
            Actuator::Initialized) {
            m_pController->getCVPValue((uint8_t)XR1::Knee_X);
        }
    }


//    if (power_reading_counter > (200*5)){
//
//        power_reading_counter = 0;
//
//        if ((int) m_pController->getActuatorAttribute((uint8_t)XR1::Back_Y, Actuator::INIT_STATE) == Actuator::Initialized)
//            m_pController->regainAttrbute((uint8_t)XR1::Back_Y , Actuator::VOLTAGE);
//    }


    hand_command_switch = !hand_command_switch;

}
