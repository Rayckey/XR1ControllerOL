//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"


void XR1ControllerOL::setupJointStateTable(){

    m_jointlookup["Wheel_Left"] = 1;
    m_jointlookup["Wheel_Right"] = 2;
    m_jointlookup["Wheel_Back"] = 3;
    m_jointlookup["Knee_X"] = 4;
    m_jointlookup["Back_Z"] = 5;
    m_jointlookup["Back_X"] = 6;
    m_jointlookup["Back_Y"] = 7;
    m_jointlookup["Neck_Z"] = 8;
    m_jointlookup["Neck_X"] = 9;
    m_jointlookup["Head_Y"] = 10;
    m_jointlookup["Left_Shoulder_X"] = 11;
    m_jointlookup["Left_Shoulder_Y"] = 12;
    m_jointlookup["Left_Elbow_Z"] = 13;
    m_jointlookup["Left_Elbow_X"] = 14;
    m_jointlookup["Left_Wrist_Z"] = 15;
    m_jointlookup["Left_Wrist_X"] = 16;
    m_jointlookup["Left_Wrist_Y"] = 17;
    m_jointlookup["Right_Shoulder_X"] = 18;
    m_jointlookup["Right_Shoulder_Y"] = 19;
    m_jointlookup["Right_Elbow_Z"] = 20;
    m_jointlookup["Right_Elbow_X"] = 21;
    m_jointlookup["Right_Wrist_Z"] = 22;
    m_jointlookup["Right_Wrist_X"] = 23;
    m_jointlookup["Right_Wrist_Y"] = 24;
    m_jointlookup["Left_Thumb"] = 25;
    m_jointlookup["Left_Index"] = 26;
    m_jointlookup["Left_Middle"] = 27;
    m_jointlookup["Left_Ring"] = 28;
    m_jointlookup["Left_Pinky"] = 29;
    m_jointlookup["Right_Thumb"] = 30;
    m_jointlookup["Right_Index"] = 31;
    m_jointlookup["Right_Middle"] = 32;
    m_jointlookup["Right_Ring"] = 33;
    m_jointlookup["Right_Pinky"] = 34;


    m_namelookup[1] = "Left_Front";
    m_namelookup[2] = "Right_Front";
    m_namelookup[3] = "Back_Wheel";
    m_namelookup[4] = "Knee_X";
    m_namelookup[5] = "Back_Z";
    m_namelookup[6] = "Back_X";
    m_namelookup[7] = "Back_Y";
    m_namelookup[8] = "Neck_Z";
    m_namelookup[9] = "Neck_X";
    m_namelookup[10] = "Head_Y" ;
    m_namelookup[11] = "Left_Shoulder_X" ;
    m_namelookup[12] = "Left_Shoulder_Y" ;
    m_namelookup[13] = "Left_Elbow_Z" ;
    m_namelookup[14] = "Left_Elbow_X" ;
    m_namelookup[15] = "Left_Wrist_Z" ;
    m_namelookup[16] = "Left_Wrist_X" ;
    m_namelookup[17] = "Left_Wrist_Y" ;
    m_namelookup[18] = "Right_Shoulder_X" ;
    m_namelookup[19] = "Right_Shoulder_Y" ;
    m_namelookup[20] = "Right_Elbow_Z" ;
    m_namelookup[21] = "Right_Elbow_X" ;
    m_namelookup[22] = "Right_Wrist_Z" ;
    m_namelookup[23] = "Right_Wrist_X" ;
    m_namelookup[24] = "Right_Wrist_Y" ;
    m_namelookup[25] = "Left_Thumb" ;
    m_namelookup[26] = "Left_Index" ;
    m_namelookup[27] = "Left_Middle" ;
    m_namelookup[28] = "Left_Ring" ;
    m_namelookup[29] = "Left_Pinky" ;
    m_namelookup[30] = "Right_Thumb" ;
    m_namelookup[31] = "Right_Index" ;
    m_namelookup[32] = "Right_Middle" ;
    m_namelookup[33] = "Right_Ring" ;
    m_namelookup[34] = "Right_Pinky" ;

    for (uint8_t fake_joint_id = 4; fake_joint_id <= 34 ; fake_joint_id++){

        temp_jointstate.name.push_back(m_namelookup[fake_joint_id]);

        temp_jointstate.position.push_back(0);

        temp_jointstate.velocity.push_back(0);

        temp_jointstate.effort.push_back(0);

    }

}




void XR1ControllerOL::subscribeJointStates(const sensor_msgs::JointState & msg){

    std::vector< uint8_t> temp_id;

    uint8_t joint_id;



    for (uint8_t msg_id = 0 ; msg_id < msg.name.size(); msg_id++){

        joint_id = m_jointlookup[msg.name[msg_id]];

        if ( XR1_ptr->getSubControlMode(m_control_group_lookup[ joint_id ]) == XR1::MoCapMode ){

            temp_id.push_back(joint_id);

            XR1_ptr->setJointPosition(joint_id , msg.position[msg_id] );

        }

    }


    while (temp_id.size()){


        if (temp_id.back() <= XR1::MainBody){

        }

        else {
            ROS_INFO( "Joint State set Joint [%d] to [%f]" , (int) temp_id.back() , XR1_ptr->getTargetJointPosition(temp_id.back()) );

            // Uncomment this to go to town ----------------------------------------
            m_pController->setPosition(temp_id.back() , XR1_ptr->getTargetJointPosition(temp_id.back()));
            // ---------------------------------------------------------------------
        }


        temp_id.pop_back();
    }


}


void XR1ControllerOL::publishJointStates(){

    std_msgs::Header temp_header;

    temp_header.stamp = ros::Time::now();
    temp_jointstate.header = temp_header;

    for (uint8_t fake_joint_id = 1; fake_joint_id <= 31 ; fake_joint_id++){

        temp_jointstate.position[fake_joint_id - 1] = XR1_ptr->getJointPosition(fake_joint_id + 3, true);

        temp_jointstate.velocity[fake_joint_id - 1] = XR1_ptr->getJointVelocity(fake_joint_id + 3, true);

        temp_jointstate.effort[fake_joint_id - 1] = XR1_ptr->getJointCurrent(fake_joint_id + 3, true);

    }


    m_joint_state_publisher.publish(temp_jointstate);
}




void XR1ControllerOL::subscribeSpecial(const std_msgs::Int8 & msg){

    setSubControlMode(XR1::MainBody , XR1::DirectMode);
    setSubControlMode(XR1::HeadBody , XR1::DirectMode);
    setSubControlMode(XR1::LeftArm , XR1::DirectMode);
    setSubControlMode(XR1::RightArm , XR1::DirectMode);
    setSubControlMode(XR1::LeftHand , XR1::DirectMode);
    setSubControlMode(XR1::RightHand , XR1::DirectMode);

    XRA_ptr->clearAll();

    XR1_ptr->clearStates();

    // yo we standing
    if (msg.data == 1){

        for (uint8_t i = XR1::Knee_X ; i < XR1::Actuator_Total ; i++){
            XR1_ptr->setState(i, 0, 3000);
        }

    }


    else if (msg.data == 2){


        XR1_ptr->setState(XR1::Knee_X, -0.609089391237561, 3000);
        XR1_ptr->setState(XR1::Back_Z, 0, 3000);
        XR1_ptr->setState(XR1::Back_X , 0.909627362219072 , 3000);
        XR1_ptr->setState(XR1::Back_Y , 0 , 3000);


        XR1_ptr->setState(XR1::Neck_Z, 0, 3000);
        XR1_ptr->setState(XR1::Neck_X , 0.5 , 3000);
        XR1_ptr->setState(XR1::Head , 0 , 3000);


        XR1_ptr->setState(XR1::Left_Elbow_X,0,3000);
        XR1_ptr->setState(XR1::Left_Elbow_Z,0,3000);
        XR1_ptr->setState(XR1::Left_Shoulder_Y,0,3000);
        XR1_ptr->setState(XR1::Left_Shoulder_X,-0.3,3000);
        XR1_ptr->setState(XR1::Left_Wrist_X,0,3000);
        XR1_ptr->setState(XR1::Left_Wrist_Y,0,3000);
        XR1_ptr->setState(XR1::Left_Wrist_Z,0,3000);


        XR1_ptr->setState(XR1::Right_Elbow_X,0,3000);
        XR1_ptr->setState(XR1::Right_Elbow_Z,0,3000);
        XR1_ptr->setState(XR1::Right_Shoulder_Y,0,3000);
        XR1_ptr->setState(XR1::Right_Shoulder_X,-0.3,3000);
        XR1_ptr->setState(XR1::Right_Wrist_X,0,3000);
        XR1_ptr->setState(XR1::Right_Wrist_Y,0,3000);
        XR1_ptr->setState(XR1::Right_Wrist_Z,0,3000);

        XR1_ptr->setState(XR1::Left_Index ,0 , 3000);
        XR1_ptr->setState(XR1::Left_Thumb ,0 , 3000);
        XR1_ptr->setState(XR1::Left_Middle,0 , 3000);
        XR1_ptr->setState(XR1::Left_Ring  ,0 , 3000);
        XR1_ptr->setState(XR1::Left_Pinky ,0 , 3000);


        XR1_ptr->setState(XR1::Left_Index ,0 , 3000);
        XR1_ptr->setState(XR1::Left_Thumb ,0 , 3000);
        XR1_ptr->setState(XR1::Left_Middle,0 , 3000);
        XR1_ptr->setState(XR1::Left_Ring  ,0 , 3000);
        XR1_ptr->setState(XR1::Left_Pinky ,0 , 3000);


    }


    else {

    }



}