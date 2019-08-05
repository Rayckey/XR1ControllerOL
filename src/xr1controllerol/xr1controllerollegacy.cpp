//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"
#include <ros/package.h>

void XR1ControllerOL::subscribeRecordCommand(const std_msgs::Bool & msg){
    debug_switch = msg.data;
}

void XR1ControllerOL::subscribeWriteCommand(const std_msgs::Bool & msg){


    writeDebug(debug_pos , "debug_pos");
    writeDebug(debug_vel , "debug_vel");
    writeDebug(debug_acc , "debug_acc");
    writeDebug(debug_cur , "debug_cur");
    writeDebug(debug_tar_pos , "debug_tar_pos");
    writeDebug(debug_tar_vel , "debug_tar_vel");
    writeDebug(debug_tar_acc , "debug_tar_acc");
    writeDebug(debug_tar_cur , "debug_tar_cur");

    debug_pos.clear();
    debug_vel.clear();
    debug_acc.clear();
    debug_cur.clear();
    debug_tar_pos.clear();
    debug_tar_vel.clear();
    debug_tar_acc.clear();
    debug_tar_cur.clear();


}
void XR1ControllerOL::recorderDebug(){


    if (debug_switch){
        std::vector<double> temp_debug_pos;
        std::vector<double> temp_debug_vel;
        std::vector<double> temp_debug_acc;
        std::vector<double> temp_debug_cur;
        std::vector<double> temp_debug_tar_pos;
        std::vector<double> temp_debug_tar_vel;
        std::vector<double> temp_debug_tar_acc;
        std::vector<double> temp_debug_tar_cur;


        for (uint8_t joint_id = XR1::Back_Z ; joint_id < XR1::LeftHand ; joint_id ++){

            temp_debug_pos.push_back(XR1_ptr->getJointPosition(joint_id , true));
            temp_debug_vel.push_back(XR1_ptr->getJointVelocity(joint_id , true));
            temp_debug_acc.push_back(XR1_ptr->getJointAcceleration(joint_id));
            temp_debug_cur.push_back(XR1_ptr->getJointEffort(joint_id , true));
            temp_debug_tar_pos.push_back(XR1_ptr->getTargetJointPosition(joint_id , true));
            temp_debug_tar_vel.push_back(XR1_ptr->getTargetJointVelocity(joint_id , true));
            temp_debug_tar_acc.push_back(XR1_ptr->getTargetJointAcceleration(joint_id));
            temp_debug_tar_cur.push_back(XR1_ptr->getTargetJointEffort(joint_id , true));

        }


        debug_pos.push_back(temp_debug_pos);
        debug_vel.push_back(temp_debug_vel);
        debug_acc.push_back(temp_debug_acc);
        debug_cur.push_back(temp_debug_cur);
        debug_tar_pos.push_back(temp_debug_tar_pos);
        debug_tar_vel.push_back(temp_debug_tar_vel);
        debug_tar_acc.push_back(temp_debug_tar_acc);
        debug_tar_cur.push_back(temp_debug_tar_cur);
    }

}

void XR1ControllerOL::writeDebug(std::vector<std::vector<double>> debug_data, std::string file_name){

    std::string path = ros::package::getPath("xr1controllerol");

    std::ofstream myfile;
    myfile.open (path + "/" + file_name);


    if (debug_data.size()) {
        for (int i = 0; i < debug_data.size(); ++i)
        {
            for (int j = 0; j < debug_data[i].size() - 1; ++j)
            {
                myfile << debug_data[i][j] << ",";
            }

            myfile << debug_data[i][debug_data[i].size() - 1] << "\n";
        }
    }

    myfile.close();

}


//void XR1ControllerOL::QuaCallBack(uint64_t id, double w, double x, double y, double z) {
//
//    // ROS_INFO("[%f][%f][%f][%f]",w,x,y,z);
//    // if (precision > 1){
//
//    // If it is the base frame
//    if (id == ActuatorController::toLongId("192.168.1.4", 0))
//        XR1_ptr->tiltCallback(w, x, y, z);
//
//        // If it is a MoCap module
//    else {
//        IMU_ptr->quaternioncallback(ActuatorController::toByteId(id), w, x, y, z);
//    }
//
//    // }
//
//}


//void XR1ControllerOL::requestAcc(const ros::TimerEvent &) {
//    m_pController->requestSingleQuaternion(ActuatorController::toLongId("192.168.1.4", 0));
//    // m_pController->requestSingleQuaternion();
//}
//
//void XR1ControllerOL::requestQue(const ros::TimerEvent &) {
//    m_pController->requestAllQuaternions();
//}


//void XR1ControllerOL::MoCapCallback(const ros::TimerEvent &) {
//
////    if (XR1_ptr->getMetaMode() == XR1::MoCapMode) {
////
////        std::vector<double> temp_vec = IMU_ptr->getJointAngles();
////
////        XR1_ptr->setMoCapPosition(IMU_ptr->getJointAngles());
////
////        XR1_ptr->getTargetPosition(XR1::LeftArm , temp_vec7d);
////
////        setJointPosition(XR1::LeftArm, temp_vec7d);
////
////    }
//
//}




//void XR1ControllerOL::subscribetiltInit(const std_msgs::Bool &msg) {
//    XR1_ptr->tiltInit();
//}
//
//void XR1ControllerOL::subscribeMoCapInit(const std_msgs::Bool &msg) {
//    ROS_INFO("Here be initialized");
////    XR1_ptr->setMetaMode(XR1::MoCapMode);
//    IMU_ptr->Initialize();
//}