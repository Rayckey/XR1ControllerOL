//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"




void XR1ControllerOL::judgeControlGroupModes(){


    judgeActuatorModes(XR1::MainBody);
    judgeActuatorModes(XR1::LeftArm);
    judgeActuatorModes(XR1::RightArm);
    judgeActuatorModes(XR1::LeftHand);
    judgeActuatorModes(XR1::RightHand);

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


        XR1_ptr->getTargetPosition(XR1::LeftArm , temp_vec7d);
        setJointPosition(XR1::LeftArm , temp_vec7d);

//        ROS_INFO("Unleasing at time [%f]" , ros::WallTime::now().toSec()) ;
//        ROS_INFO("[%f][%f][%f][%f][%f][%f][%f]" , XR1_ptr->getTargetJointPosition(XR1::Left_Shoulder_X , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Left_Shoulder_Y , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Left_Elbow_Z , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Left_Elbow_X , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Left_Wrist_Z , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Left_Wrist_X , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Left_Wrist_Y , true)) ;

        XR1_ptr->getTargetPosition(XR1::RightArm , temp_vec7d);
        setJointPosition(XR1::RightArm , temp_vec7d);

        XR1_ptr->getTargetPosition(XR1::MainBody , temp_vec7d);
        setJointPosition(XR1::MainBody , temp_vec7d);


//        ROS_INFO("Unleasing at time [%f]" , ros::WallTime::now().toSec()) ;
//        ROS_INFO("[%f][%f][%f][%f][%f][%f][%f]" , XR1_ptr->getTargetJointPosition(XR1::Knee_X , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Back_Z , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Back_X , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Back_Y , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Neck_Z , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Neck_X , true),
//                 XR1_ptr->getTargetJointPosition(XR1::Head , true)) ;

        XR1_ptr->getTargetPosition(XR1::LeftHand , temp_vec5d);
        setJointPosition(XR1::LeftHand , temp_vec5d);

        XR1_ptr->getTargetPosition(XR1::RightHand , temp_vec5d);
        setJointPosition(XR1::RightHand , temp_vec5d);

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
