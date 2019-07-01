//
// Created by rocky on 19-3-7.
//

#include "../../include/xr1controllerol/xr1controllerolmsgulit.h"


using namespace Eigen;

Eigen::VectorXd BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(4);

    res(0) =  msg.Knee,
    res(1) =  msg.Back_Z,
    res(2) =  msg.Back_X,
    res(3) =  msg.Back_Y;

    return res;
}


Eigen::VectorXd HeadMsgs2VectorXd(const xr1controllerros::HeadMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(3);

    res(0) =  msg.Neck_Z,
    res(1) =  msg.Neck_X,
    res(2) =  msg.Head;

    return res;
}


Eigen::VectorXd ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

    res(0) =  msg.Shoulder_X,
    res(1) =  msg.Shoulder_Y,
    res(2) =  msg.Elbow_Z,
    res(3) =  msg.Elbow_X,
    res(4) =  msg.Wrist_Z,
    res(5) =  msg.Wrist_X,
    res(6) =  msg.Wrist_Y;


    return res;
}

Eigen::VectorXd HandsMsgs2VectorXd(const xr1controllerros::HandMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(5);

    res(0) =  msg.Thumb,
    res(1) =  msg.Index,
    res(2) =  msg.Middle,
    res(3) =  msg.Ring,
    res(4) =  msg.Pinky;


    return res;
}


void BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs &msg , VectorXd & output_ref) {

    output_ref(0) =  msg.Knee,
    output_ref(1) =  msg.Back_Z,
    output_ref(2) =  msg.Back_X,
    output_ref(3) =  msg.Back_Y;

}


void HeadMsgs2VectorXd(const xr1controllerros::HeadMsgs &msg , VectorXd & output_ref) {

    output_ref(0) =  msg.Neck_Z,
    output_ref(1) =  msg.Neck_X,
    output_ref(2) =  msg.Head;

}

void ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs &msg , VectorXd & output_ref) {

    output_ref(0) =  msg.Shoulder_X,
    output_ref(1) =  msg.Shoulder_Y,
    output_ref(2) =  msg.Elbow_Z,
    output_ref(3) =  msg.Elbow_X,
    output_ref(4) =  msg.Wrist_Z,
    output_ref(5) =  msg.Wrist_X,
    output_ref(6) =  msg.Wrist_Y;

}

void HandsMsgs2VectorXd(const xr1controllerros::HandMsgs &msg , VectorXd & output_ref) {

    output_ref(0) =  msg.Thumb,
    output_ref(1) =  msg.Index,
    output_ref(2) =  msg.Middle,
    output_ref(3) =  msg.Ring,
    output_ref(4) =  msg.Pinky;

}



xr1controllerros::HandMsgs ConvertHandMsgs(Eigen::VectorXd & HandPosition) {

    xr1controllerros::HandMsgs msg;
    msg.Thumb = HandPosition(0);
    msg.Index = HandPosition(1);
    msg.Middle = HandPosition(2);
    msg.Ring = HandPosition(3);
    msg.Pinky = HandPosition(4);

    return msg;
}

xr1controllerros::HandMsgs ConvertHandMsgs(std::vector<double> HandPosition) {
    xr1controllerros::HandMsgs msg;
    msg.Thumb = HandPosition[0];
    msg.Index = HandPosition[1];
    msg.Middle = HandPosition[2];
    msg.Ring = HandPosition[3];
    msg.Pinky = HandPosition[4];
    return msg;
}


xr1controllerros::ArmMsgs ConvertArmMsgs(std::vector<double> input) {

    xr1controllerros::ArmMsgs msg;

    msg.Shoulder_X = input[0];
    msg.Shoulder_Y = input[1];
    msg.Elbow_Z = input[2];
    msg.Elbow_X = input[3];
    msg.Wrist_Z = input[4];
    msg.Wrist_X = input[5];
    msg.Wrist_Y = input[6];


    return msg;
}

xr1controllerros::ArmMsgs ConvertArmMsgs(Eigen::VectorXd & input) {
    xr1controllerros::ArmMsgs msg;

    msg.Shoulder_X = input(0);
    msg.Shoulder_Y = input(1);
    msg.Elbow_Z = input(2);
    msg.Elbow_X = input(3);
    msg.Wrist_Z = input(4);
    msg.Wrist_X = input(5);
    msg.Wrist_Y = input(6);

    return msg;
}

xr1controllerros::BodyMsgs ConvertBodyMsgs(std::vector<double> input) {

    xr1controllerros::BodyMsgs msg;

    msg.Knee = input[0];
    msg.Back_Z = input[1];
    msg.Back_X = input[2];
    msg.Back_Y = input[3];

    return msg;
}

xr1controllerros::BodyMsgs ConvertBodyMsgs(Eigen::VectorXd & input) {

    xr1controllerros::BodyMsgs msg;

    msg.Knee = input(0);
    msg.Back_Z = input(1);
    msg.Back_X = input(2);
    msg.Back_Y = input(3);

    return msg;
}


xr1controllerros::HeadMsgs ConvertHeadMsgs(std::vector<double> input) {

    xr1controllerros::HeadMsgs msg;

    msg.Neck_Z = input[0];
    msg.Neck_X = input[1];
    msg.Head = input[2];

    return msg;
}

xr1controllerros::HeadMsgs ConvertHeadMsgs(Eigen::VectorXd & input) {

    xr1controllerros::HeadMsgs msg;


    msg.Neck_Z = input(0);
    msg.Neck_X = input(1);
    msg.Head = input(2);

    return msg;
}



void ConvertHandMsgs(Eigen::VectorXd & HandPosition , xr1controllerros::HandMsgs & msg) {

    msg.Thumb = HandPosition(0);
    msg.Index = HandPosition(1);
    msg.Middle = HandPosition(2);
    msg.Ring = HandPosition(3);
    msg.Pinky = HandPosition(4);

}

void ConvertHandMsgs(std::vector<double> HandPosition , xr1controllerros::HandMsgs & msg) {

    msg.Thumb = HandPosition[0];
    msg.Index = HandPosition[1];
    msg.Middle = HandPosition[2];
    msg.Ring = HandPosition[3];
    msg.Pinky = HandPosition[4];

}


void ConvertArmMsgs(std::vector<double> input , xr1controllerros::ArmMsgs & msg) {

    msg.Shoulder_X = input[0];
    msg.Shoulder_Y = input[1];
    msg.Elbow_Z = input[2];
    msg.Elbow_X = input[3];
    msg.Wrist_Z = input[4];
    msg.Wrist_X = input[5];
    msg.Wrist_Y = input[6];

}

void ConvertArmMsgs(Eigen::VectorXd & input , xr1controllerros::ArmMsgs & msg) {

    msg.Shoulder_X = input(0);
    msg.Shoulder_Y = input(1);
    msg.Elbow_Z = input(2);
    msg.Elbow_X = input(3);
    msg.Wrist_Z = input(4);
    msg.Wrist_X = input(5);
    msg.Wrist_Y = input(6);

}

void ConvertBodyMsgs(std::vector<double> input , xr1controllerros::BodyMsgs & msg) {

    msg.Knee = input[0];
    msg.Back_Z = input[1];
    msg.Back_X = input[2];
    msg.Back_Y = input[3];

}

void ConvertBodyMsgs(Eigen::VectorXd & input , xr1controllerros::BodyMsgs & msg) {

    msg.Knee = input(0);
    msg.Back_Z = input(1);
    msg.Back_X = input(2);
    msg.Back_Y = input(3);

}


void ConvertHeadMsgs(std::vector<double> input , xr1controllerros::HeadMsgs & msg) {

    msg.Neck_Z = input[0];
    msg.Neck_X = input[1];
    msg.Head = input[2];

}

void ConvertHeadMsgs(Eigen::VectorXd & input , xr1controllerros::HeadMsgs & msg) {

    msg.Neck_Z = input(0);
    msg.Neck_X = input(1);
    msg.Head = input(2);

}


void MultiArray2DequeVector(const std_msgs::Float64MultiArray & input_msg , std::deque<std::vector<double>> & output_data){

    std::vector<double> temp_vec;

    output_data.clear();

    int num_of_row = input_msg.layout.dim[0].size;
    int num_of_joints = input_msg.layout.dim[1].size;

    for (int i = 0 ; i < num_of_row ; i++){
        temp_vec.clear();
        for (int j = 0 ; j < num_of_joints ; j ++) {
            temp_vec.push_back(input_msg.data[i*input_msg.layout.dim[0].stride + j]);
        }
        output_data.push_back(temp_vec);
    }

}