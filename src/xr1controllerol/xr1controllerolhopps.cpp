//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"


void setupJointStateTable(){

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



    m_namelookup[4] = ["Knee_X"];
    m_namelookup[5] = ["Back_Z"];
    m_namelookup[6] = ["Back_X"];
    m_namelookup[7] = ["Back_Y"];
    m_namelookup[8] = ["Neck_Z"];
    m_namelookup[9] = ["Neck_X"];
    m_namelookup[10] = ["Head_Y"] ;
    m_namelookup[11] = ["Left_Shoulder_X"] ;
    m_namelookup[12] = ["Left_Shoulder_Y"] ;
    m_namelookup[13] = ["Left_Elbow_Z"] ;
    m_namelookup[14] = ["Left_Elbow_X"] ;
    m_namelookup[15] = ["Left_Wrist_Z"] ;
    m_namelookup[16] = ["Left_Wrist_X"] ;
    m_namelookup[17] = ["Left_Wrist_Y"] ;
    m_namelookup[18] = ["Right_Shoulder_X"] ;
    m_namelookup[19] = ["Right_Shoulder_Y"] ;
    m_namelookup[20] = ["Right_Elbow_Z"] ;
    m_namelookup[21] = ["Right_Elbow_X"] ;
    m_namelookup[22] = ["Right_Wrist_Z"] ;
    m_namelookup[23] = ["Right_Wrist_X"] ;
    m_namelookup[24] = ["Right_Wrist_Y"] ;
    m_namelookup[25] = ["Left_Thumb"] ;
    m_namelookup[26] = ["Left_Index"] ;
    m_namelookup[27] = ["Left_Middle"] ;
    m_namelookup[28] = ["Left_Ring"] ;
    m_namelookup[29] = ["Left_Pinky"] ;
    m_namelookup[30] = ["Right_Thumb"] ;
    m_namelookup[31] = ["Right_Index"] ;
    m_namelookup[32] = ["Right_Middle"] ;
    m_namelookup[33] = ["Right_Ring"] ;
    m_namelookup[34] = ["Right_Pinky"] ;


}




void subscribeJointStates(const sensor_msgs::JointState & msg){


    m_jointlookup




}