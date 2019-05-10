//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"



void XR1ControllerOL::subscribeRobotMode(const xr1controllerros::ChainModeChange &msg) {
    setControlMode(msg.ChainID , msg.Mode);
}


void XR1ControllerOL::judgeControlGroupModes() {

    for (uint8_t control_group : control_group_flags)
        judgeActuatorModes(control_group);

}

void XR1ControllerOL::judgeActuatorModes(uint8_t control_group) {

    if (control_modes[control_group] != XR1_ptr->getSubControlMode(control_group))
        setControlMode(control_group, XR1_ptr->getSubControlMode(control_group));

}


void XR1ControllerOL::setControlMode(uint8_t control_group, uint8_t option) {

    if (allActuatorHasLaunched() && RecognizeFinished){
        if (control_modes.find(control_group) == control_modes.end()) {
            ROS_INFO("Wrong input received for Control Mode");
            return;
        }

        // redundant command
        if (control_modes[control_group] == option) {

            ROS_INFO("Redundant Mode change messaged received : Control Group [%d], Mode [%d]", control_group, option);

        } else {

            ROS_INFO("Setting Control Group [%d] to Mode [%d]", control_group, option);

            control_modes[control_group] = option;

            XR1_ptr->setSubControlMode(control_group, option);


            uint8_t temp_mode = XR1_ptr->getSubControlMode(control_group);

            // low profile position mode
            if (temp_mode < XR1::IKMode) {

                for (uint8_t joint_id : control_group_map[control_group]) {

                    if ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized)
                        m_pController->activateActuatorMode(joint_id, Actuator::Mode_Profile_Pos);

                }
            }

                // high frequency position mode
            else if (temp_mode <= XR1::DriveMode) {

                for (uint8_t joint_id : control_group_map[control_group]) {

                    if ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized)
                        m_pController->activateActuatorMode(joint_id, Actuator::Mode_Pos);

                }

            }

                // high frequency velocity mode
            else if (temp_mode <= XR1::RoamMode) {

                for (uint8_t joint_id : control_group_map[control_group]) {

                    if ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized)
                        m_pController->activateActuatorMode(joint_id, Actuator::Mode_Vel);

                }

            }

                // high frequency current mode
            else if (temp_mode <= XR1::TeachMode) {

                collision_detection_switch = false;

                ROS_INFO("Collision detection is unable to activate");

                for (uint8_t joint_id : control_group_map[control_group]) {

                    if ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized)
                        m_pController->activateActuatorMode(joint_id, Actuator::Mode_Cur);

                }

            }


        }
    }



}
