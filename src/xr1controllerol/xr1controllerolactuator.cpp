//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"



void XR1ControllerOL::launchAllMotors() {

    m_pController->launchAllActuators();

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

    control_modes[XR1::OmniWheels] = 0;
    control_modes[XR1::MainBody] = 0;
    control_modes[XR1::LeftArm] = 0;
    control_modes[XR1::RightArm] = 0;
    control_modes[XR1::LeftHand] = 0;
    control_modes[XR1::RightHand] = 0;
    control_modes[XR1::HeadBody] = 0;


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

    else if (attrId == Actuator::FIRMWARE_VERSION){

        if ((uint32_t)value >    0x401){
            actuator_ready_4_cvp[id] = true;
        }

        else {
            actuator_ready_4_cvp[id] = false;
            std::cout << "actuato " << (int)id << " is really fucking old lmao" << std::endl;
        }
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

                ROS_INFO("Inited tilting");
                //XRB_ptr->tiltInit();
                XRB_ptr->tiltInit(temp_acc, 1.0, 15.0);

                ROS_INFO("All Actuators Have Launched");
                ros::Duration(0.1).sleep() ;

                for(uint8_t setBrake_times = 0; setBrake_times < 3; setBrake_times++)
                {
                    BrakeOpen = m_pController->setBrakeStatus( true );
                    if (BrakeOpen){
                        std::cout << "Open Brake successed! \n";
                        break;
                    }
                    if (setBrake_times >=2)
                        {
                            std::cout <<"Open Brake Failed! We would Shutdown the actuators \n";
                            std_msgs::Bool param;
                            param.data = true;
                            subscribeShutdown( param );
                    }
                }

            }

            break;
        default:
            break;
    }
}

void XR1ControllerOL::readingCallback() {

    for (uint8_t i = XR1::LeftArm; i < XR1::LeftHand; ++i) {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {

            if (actuator_ready_4_cvp[i])
                m_pController->getCVPValue(i);
            else {
                m_pController->regainAttrbute(i,Actuator::ACTUAL_POSITION);
                m_pController->regainAttrbute(i, Actuator::ACTUAL_VELOCITY);
                m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
            }
        }
    }


    for (uint8_t i = XR1::MainBody; i < XR1::HeadBody; ++i) {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {

            if (actuator_ready_4_cvp[i])
                m_pController->getCVPValue(i);
            else {
                if (m_b100Hz_switch){
                    m_pController->regainAttrbute(i,Actuator::ACTUAL_POSITION);
                    m_pController->regainAttrbute(i, Actuator::ACTUAL_VELOCITY);
                    m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
                }
            }
        }
    }



    for (uint8_t i = XR1::HeadBody; i < XR1::LeftArm; ++i) {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {

            if (actuator_ready_4_cvp[i])
                m_pController->getCVPValue(i);
            else {
                if (m_b100Hz_switch){
                    m_pController->regainAttrbute(i,Actuator::ACTUAL_POSITION);
                    m_pController->regainAttrbute(i, Actuator::ACTUAL_VELOCITY);
//                m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
                }
            }
        }
    }


    for (uint8_t i = XR1::OmniWheels; i < XR1::MainBody; ++i)
    {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized)
        {

            if (actuator_ready_4_cvp[i])
                m_pController->getCVPValue(i);
            else {
                if (m_b100Hz_switch) {
                    m_pController->regainAttrbute(i, Actuator::ACTUAL_POSITION);
                    m_pController->regainAttrbute(i, Actuator::ACTUAL_VELOCITY);
                }
            }
        }
    }

    if (m_b100Hz_switch) {

        for (uint8_t i = XR1::LeftHand; i < XR1::Actuator_Total; ++i) {
            if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {
                m_pController->regainAttrbute(i, Actuator::ACTUAL_POSITION);
                m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
            }
        }

//        if ((int) m_pController->getActuatorAttribute((uint8_t)XR1::Knee_X, Actuator::INIT_STATE) == Actuator::Initialized) {
//            m_pController->regainAttrbute(i, Actuator::ACTUAL_POSITION);
//            m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
//        }

    }

    ++power_reading_counter;
    if (power_reading_counter > (200*5)){

        power_reading_counter = 0;

        if ((int) m_pController->getActuatorAttribute((uint8_t)XR1::Back_Y, Actuator::INIT_STATE) == Actuator::Initialized)
            m_pController->regainAttrbute((uint8_t)XR1::Back_Y , Actuator::VOLTAGE);

        std_msgs::Float32 temp;
        temp.data = m_pController->getActuatorAttribute((uint8_t)XR1::Back_Y,Actuator::VOLTAGE);
        voltagePub.publish(temp);
    }

    m_b100Hz_switch = !m_b100Hz_switch;

}

