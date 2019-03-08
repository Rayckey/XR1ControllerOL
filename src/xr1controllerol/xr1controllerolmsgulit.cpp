//
// Created by rocky on 19-3-7.
//

#include "../../include/xr1controllerol/xr1controllerolmsgulit.h"


using namespace Eigen;

Eigen::VectorXd BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

    res << msg.Knee,
            msg.Back_Z,
            msg.Back_X,
            msg.Back_Y,
            msg.Neck_Z,
            msg.Neck_X,
            msg.Head;

    return res;
}

Eigen::VectorXd ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

    res << msg.Shoulder_X,
            msg.Shoulder_Y,
            msg.Elbow_Z,
            msg.Elbow_X,
            msg.Wrist_Z,
            msg.Wrist_X,
            msg.Wrist_Y;


    return res;
}

Eigen::VectorXd HandsMsgs2VectorXd(const xr1controllerros::HandMsgs &msg) {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(5);

    res << msg.Thumb,
            msg.Index,
            msg.Middle,
            msg.Ring,
            msg.Pinky;


    return res;
}


void BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs &msg , VectorXd & output_ref) {



    output_ref << msg.Knee,
            msg.Back_Z,
            msg.Back_X,
            msg.Back_Y,
            msg.Neck_Z,
            msg.Neck_X,
            msg.Head;

}

void ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs &msg , VectorXd & output_ref) {


    output_ref << msg.Shoulder_X,
            msg.Shoulder_Y,
            msg.Elbow_Z,
            msg.Elbow_X,
            msg.Wrist_Z,
            msg.Wrist_X,
            msg.Wrist_Y;

}

void HandsMsgs2VectorXd(const xr1controllerros::HandMsgs &msg , VectorXd & output_ref) {


    output_ref << msg.Thumb,
            msg.Index,
            msg.Middle,
            msg.Ring,
            msg.Pinky;


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
    msg.Neck_Z = input[4];
    msg.Neck_X = input[5];
    msg.Head = input[6];

    return msg;
}

xr1controllerros::BodyMsgs ConvertBodyMsgs(Eigen::VectorXd & input) {

    xr1controllerros::BodyMsgs msg;

    msg.Knee = input(0);
    msg.Back_Z = input(1);
    msg.Back_X = input(2);
    msg.Back_Y = input(3);
    msg.Neck_Z = input(4);
    msg.Neck_X = input(5);
    msg.Head = input(6);

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
    msg.Neck_Z = input[4];
    msg.Neck_X = input[5];
    msg.Head = input[6];

}

void ConvertBodyMsgs(Eigen::VectorXd & input , xr1controllerros::BodyMsgs & msg) {

    msg.Knee = input(0);
    msg.Back_Z = input(1);
    msg.Back_X = input(2);
    msg.Back_Y = input(3);
    msg.Neck_Z = input(4);
    msg.Neck_X = input(5);
    msg.Head = input(6);

}