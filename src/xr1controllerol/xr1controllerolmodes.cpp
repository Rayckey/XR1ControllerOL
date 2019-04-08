//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"




void XR1ControllerOL::judgeControlGroupModes(){


    for (uint8_t control_group : control_group_flags)
        judgeActuatorModes(control_group);


//    judgeActuatorModes(XR1::MainBody);
//    judgeActuatorModes(XR1::LeftArm);
//    judgeActuatorModes(XR1::RightArm);
////    judgeActuatorModes(XR1::LeftHand);
////    judgeActuatorModes(XR1::RightHand);

}

void XR1ControllerOL::judgeActuatorModes(uint8_t control_group){



    std::vector<uint8_t> temp_vector = control_group_map[control_group];



}



void XR1ControllerOL::setControlMode(uint8_t control_group, uint8_t option) {


    if (control_modes.find(control_group) == control_modes.end()){
        ROS_INFO("Wrong input received for Control Mode");
        return;
    }

    // redundant command
    if (control_modes[control_group] == option){
    }

    else {

            ROS_INFO("Setting Control Group [%d] to Mode [%d]" , control_group , option);

            control_modes[control_group] = option;

            XR1_ptr->setSubControlMode(control_group , option);


            uint8_t temp_mode = XR1_ptr->getSubControlMode(control_group);


        if (temp_mode < XR1::IKMode) {
            for (uint8_t joint_id : control_group_map[control_group]) {

                if ((int) m_pController->getActuatorAttribute(id, Actuator::INIT_STATE) == Actuator::Initialized)
                    m_pController->activateActuatorMode(joint_id, Actuator::Mode_Profile_Pos);

            }
        }

        else if (temp_mode < XR1::DriveMode)





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