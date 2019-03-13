#include "xr1controllerol.h"
#include <ros/package.h>
#include <iostream>


XR1ControllerOL::XR1ControllerOL() :
    hand_command_switch(true)
    ,high_frequency_switch(false)
    ,power_reading_counter(30000)
    ,animation_switch(false)
    ,previous_omni_state(false)
    ,collision_detection_switch(false){

    std::vector<double> sit_pos;

    while (sit_pos.size() < XR1::Actuator_Total)
        sit_pos.push_back(0);


    std::string path = ros::package::getPath("xr1controllerol");

    XR1_ptr = new XR1Controller(path + "/cream.xr1para", sit_pos);

    XRA_ptr = new XR1ControllerALP(path + "/ALP" , XR1_ptr, 169 , 10 , 1 , 1 );

    IMU_ptr = new XR1IMUmethods();

    m_pController = ActuatorController::getInstance();


    JointAttributePublisher = nh.advertise<xr1controllerros::JointAttributeMsgs>("XR1/JointAttribute", 1);

    ActuatorLaunchedPublisher = nh.advertise<std_msgs::Bool>("XR1/LaunchedSignal", 1);

    LaunchSubscriber = nh.subscribe("/startSimulation", 1, &XR1ControllerOL::subscribeLaunch, this);
    ShutdownSubscriber = nh.subscribe("/stopSimulation", 1, &XR1ControllerOL::subscribeShutdown, this);

    MainBodyPositionSubscriber = nh.subscribe("/MainBody/TargetPosition", 100,
                                 &XR1ControllerOL::subscribeMainBodyPosition, this);
    MainBodyCurrentSubscriber = nh.subscribe("/MainBody/TargetCurrent", 100, &XR1ControllerOL::subscribeMainBodyCurrent,
                                this);

    LeftArmPositionSubscriber = nh.subscribe("/LeftArm/TargetPosition", 100, &XR1ControllerOL::subscribeLeftArmPosition,
                                this);
    LeftArmVelocitySubscriber = nh.subscribe("/LeftArm/TargetVelocity", 100, &XR1ControllerOL::subscribeLeftArmVelocity,
                                this);
    LeftArmCurrentSubscriber = nh.subscribe("/LeftArm/TargetCurrent", 100, &XR1ControllerOL::subscribeLeftArmCurrent,
                                            this);

    RightArmPositionSubscriber = nh.subscribe("/RightArm/TargetPosition", 100,
                                 &XR1ControllerOL::subscribeRightArmPosition, this);
    RightArmVelocitySubscriber = nh.subscribe("/RightArm/TargetVelocity", 100,
                                 &XR1ControllerOL::subscribeRightArmVelocity, this);
    RightArmCurrentSubscriber = nh.subscribe("/RightArm/TargetCurrent", 100, &XR1ControllerOL::subscribeRightArmCurrent,
                                this);


    MainBodyModeChangeSubscriber = nh.subscribe("/XR1/MainBodyChainModeChange", 10,
                                   &XR1ControllerOL::subscribeMainBodyMode, this);
    LeftArmModeChangeSubscriber = nh.subscribe("/XR1/LeftArmChainModeChange", 10,
                                  &XR1ControllerOL::subscribeLeftArmMode, this);
    RightArmModeChangeSubscriber = nh.subscribe("/XR1/RightArmChainModeChange", 10,
                                   &XR1ControllerOL::subscribeRightArmMode, this);
    LeftHandModeChangeSubscriber = nh.subscribe("/XR1/LeftHandChainModeChange", 10,
                                   &XR1ControllerOL::subscribeLeftHandMode, this);
    RightHandModeChangeSubscriber = nh.subscribe("/XR1/RightHandChainModeChange", 10,
                                    &XR1ControllerOL::subscribeRightHandMode, this);


    HeadBodyModeChangeSubscriber = nh.subscribe("/XR1/HeadBodyChainModeChange", 10,
                                                &XR1ControllerOL::subscribeHeadBodyMode, this);
    BackBodyModeChangeSubscriber = nh.subscribe("/XR1/BackBodyChainModeChange", 10,
                                                &XR1ControllerOL::subscribeBackBodyMode, this);


    MetaModeSubscriber = nh.subscribe("/XR1/MetaModeChange", 1, &XR1ControllerOL::setMetaMode, this);

    LeftHandPositionSubscriber = nh.subscribe("/LeftHand/TargetPosition", 10,
                                 &XR1ControllerOL::subscribeLeftHandPosition, this);
    RightHandPositionSubscriber = nh.subscribe("/RightHand/TargetPosition", 10,
                                  &XR1ControllerOL::subscribeRightHandPosition, this);
    LeftHandCurrentSubscriber = nh.subscribe("/LeftHand/TargetCurrent", 10, &XR1ControllerOL::subscribeLeftHandCurrent,
                                this);
    RightHandCurrentSubscriber = nh.subscribe("/RightHand/TargetCurrent", 10,
                                 &XR1ControllerOL::subscribeRightHandCurrent, this);

    EStopSubscriber = nh.subscribe("XR1/EStop", 1, &XR1ControllerOL::subscribeEStop, this);


    LeftElbowSubscriber = nh.subscribe("LeftArm/ElbowAngle", 1, &XR1ControllerOL::subscribeLeftElbowAngle, this);
    RightElbowSubscriber = nh.subscribe("RightArm/ElbowAngle", 1, &XR1ControllerOL::subscribeRightElbowAngle, this);


    tiltInitSubscriber = nh.subscribe("XR1/tiltInit", 1, &XR1ControllerOL::subscribetiltInit, this);
    MoCapInitSubscriber = nh.subscribe("XR1/MoCapInit", 1, &XR1ControllerOL::subscribeMoCapInit, this);


    AnimationSwitchSubscriber = nh.subscribe("/startAnimation" , 1 , &XR1ControllerOL::subscribeStartAnimation , this);
    AnimationSetSubscriber = nh.subscribe("/setAnimation" , 1 , &XR1ControllerOL::subscribeSetAnimation , this);
    CollisionDetectionSubscriber = nh.subscribe("/setCollisionDetection" , 1 , &XR1ControllerOL::subscribeSetCollisionDetection , this);


    IKPlannerService = nh.advertiseService("XR1/IKLPT" , &XR1ControllerOL::serviceIKPlanner, this);
    IKTrackingService = nh.advertiseService("XR1/IKTT" , &XR1ControllerOL::serviceIKTracking , this);



    MainBodyPositionPublisher = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Position", 1);
    MainBodyCurrentPublisher = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Current", 1);

    LeftArmPositionPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Position", 1);
    LeftArmVelocityPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Velocity", 1);
    LeftArmCurrentPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Current", 1);

    RightArmPositionPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Position", 1);
    RightArmVelocityPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Velocity", 1);
    RightArmCurrentPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Current", 1);

    LeftHandPositionPublisher = nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/Position", 1);
    RightHandPositionPublisher = nh.advertise<xr1controllerros::HandMsgs>("/RightHand/Position", 1);




    m_pController->m_sOperationFinished->connect_member(this, &XR1ControllerOL::actuatorOperation);


    // Getting all the group ID right ------------------------------------------------
    std::vector<uint8_t> temp_vector;

    temp_vector.clear();
    for (uint8_t i = XR1::MainBody; i < XR1::LeftArm; i++) {
        temp_vector.push_back(i);
    }
    control_group_map[XR1::MainBody] = temp_vector;

    temp_vector.clear();
    for (uint8_t i = XR1::LeftArm; i < XR1::RightArm; i++) {
        temp_vector.push_back(i);
    }
    control_group_map[XR1::LeftArm] = temp_vector;


    temp_vector.clear();
    for (uint8_t i = XR1::RightArm; i < XR1::LeftHand; i++) {
        temp_vector.push_back(i);
    }
    control_group_map[XR1::RightArm] = temp_vector;


    temp_vector.clear();
    for (uint8_t i = XR1::LeftHand; i < XR1::RightHand; i++) {
        temp_vector.push_back(i);
    }
    control_group_map[XR1::LeftHand] = temp_vector;


    temp_vector.clear();
    for (uint8_t i = XR1::RightHand; i < XR1::Actuator_Total; i++) {
        temp_vector.push_back(i);
    }
    control_group_map[XR1::RightHand] = temp_vector;
    // ------------------------------------------------



    attribute_map[Actuator::ACTUAL_POSITION] = XR1::ActualPosition;
    attribute_map[Actuator::ACTUAL_VELOCITY] = XR1::ActualVelocity;
    attribute_map[Actuator::ACTUAL_CURRENT] = XR1::ActualCurrent;


    mode_map[XR1::PositionMode] = Actuator::Mode_Profile_Pos;
    mode_map[XR1::VelocityMode] = Actuator::Mode_Profile_Vel;
    mode_map[XR1::ForceMode] = Actuator::Mode_Cur;
    mode_map[XR1::IKMode] = Actuator::Mode_Profile_Pos;

    control_modes[XR1::OmniWheels] = 0;
    control_modes[XR1::MainBody] = 0;
    control_modes[XR1::LeftArm] = 0;
    control_modes[XR1::RightArm] = 0;
    control_modes[XR1::LeftHand] = 0;
    control_modes[XR1::RightHand] = 0;
    control_modes[XR1::HeadBody] = 0;
    control_modes[XR1::BackBody] = 0;



    //Update Callback
    m_pController->m_sActuatorAttrChanged->connect_member(this, &XR1ControllerOL::updatingCallback);

    m_pController->m_sQuaternionL->connect_member(this, &XR1ControllerOL::QuaCallBack);

    // m_pController->m_sAcceleration->connect_member(this, &XR1ControllerOL::accCallBack);


    // Elbow lower anlges, measured from the top, by default:
    // For left arm, the desirable range is 1.0 to 3.0
    // For right arm, the desirable ranfge is -3.0 to -1.0
    LeftElbowAngle = 2.5;
    RightElbowAngle = -2.5;
    ROS_INFO("OL finished");


    temp_vec5d = VectorXd::Zero(5);
    temp_vec7d = VectorXd::Zero(7);
    temp_vec3d = VectorXd::Zero(3);

    XR1_ptr->setInverseDynamicsOption(XR1::GravityCompensation);


}

XR1ControllerOL::~XR1ControllerOL() {
    // unregister all publishers here

    JointAttributePublisher.shutdown();
    ActuatorLaunchedPublisher.shutdown();

}


void XR1ControllerOL::QuaCallBack(uint64_t id, double w, double x, double y, double z) {

    // ROS_INFO("[%f][%f][%f][%f]",w,x,y,z);
    // if (precision > 1){

    // If it is the base frame
    if (id == ActuatorController::toLongId("192.168.1.4", 0))
        XR1_ptr->tiltCallback(w, x, y, z);

    // If it is a MoCap module
    else {
        IMU_ptr->quaternioncallback(ActuatorController::toByteId(id), w, x, y, z);
    }

    // }

}



// void XR1ControllerOL::accCallBack(uint8_t id , double x , double y , double z , int pres){
//  // ROS_INFO("[%d][%f][%f][%f]",pres,x,y,z);
// }

void XR1ControllerOL::requestAcc(const ros::TimerEvent &) {
    m_pController->requestSingleQuaternion(ActuatorController::toLongId("192.168.1.4", 0));
    // m_pController->requestSingleQuaternion();
}

void XR1ControllerOL::requestQue(const ros::TimerEvent &) {
    m_pController->requestAllQuaternions();
}


void XR1ControllerOL::setMetaMode(const std_msgs::Int32 &msg) {
    XR1_ptr->tiltInit();
    XR1_ptr->setMetaMode(msg.data);
}


void XR1ControllerOL::MoCapCallback(const ros::TimerEvent &) {

    if (XR1_ptr->getMetaMode() == XR1::MoCapMode) {

        std::vector<double> temp_vec = IMU_ptr->getJointAngles();

        XR1_ptr->setMoCapPosition(IMU_ptr->getJointAngles());

        XR1_ptr->getTargetPosition(XR1::LeftArm , temp_vec7d);

        setJointPosition(XR1::LeftArm, temp_vec7d);

    }

}


void XR1ControllerOL::launchAllMotors() {

    m_pController->launchAllActuators();
    if (allActuatorHasLaunched()) {
        XR1_ptr->setInverseDynamicsOption(XR1::GravityCompensation);
        std_msgs::Bool stuff;
        stuff.data = true;
        ActuatorLaunchedPublisher.publish(stuff);
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

void XR1ControllerOL::startSimulation() {
    launchAllMotors();
}

void XR1ControllerOL::stopAllMotors() {

    std::vector<uint8_t> IDArray = m_pController->getActuatorIdArray();

    for (int i = 0; i < IDArray.size(); i++) {
        m_pController->closeActuator(IDArray.at(i));
    }

}

void XR1ControllerOL::stopSimulation() {


    control_modes[XR1::MainBody] = 0;
    control_modes[XR1::LeftArm] = 0;
    control_modes[XR1::RightArm] = 0;
    control_modes[XR1::LeftHand] = 0;
    control_modes[XR1::RightHand] = 0;
    control_modes[XR1::HeadBody] = 0;
    control_modes[XR1::BackBody] = 0;

    stopAllMotors();
}


void XR1ControllerOL::setControlMode(uint8_t control_group, uint8_t option) {


    if (control_modes.find(control_group) == control_modes.end()){
        ROS_INFO("Wrong input received for Control Mode");
        return;
    }


    if (control_modes[control_group] == option){
    }

    else {


        if (high_frequency_switch){


            if (control_group == XR1::HeadBody || control_group == XR1::BackBody){
                XR1_ptr->setControlMode(control_group , option);
                control_modes[control_group] = option;
            }




        }

        else {

            ROS_INFO("Setting Control Group [%d] to Mode [%d]" , control_group , option);

            control_modes[control_group] = option;

            XR1_ptr->setControlMode(control_group , option);

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



void XR1ControllerOL::updatingCallback(uint8_t id, uint8_t attrId, double value) {
    if (attribute_map.find(attrId) != attribute_map.end())
        XR1_ptr->updatingCallback(id, attribute_map[attrId], value);
}


void XR1ControllerOL::subscribeLaunch(const std_msgs::Bool &msg) {
    launchAllMotors();
}

void XR1ControllerOL::subscribeShutdown(const std_msgs::Bool &msg) {
    stopAllMotors();
}

void XR1ControllerOL::subscribeEStop(const std_msgs::Bool &msg) {

    if (msg.data) {
        XR1_ptr->employLockdown();
        for (uint8_t i = XR1::OmniWheels; i < XR1::Actuator_Total; i++)
            m_pController->activateActuatorMode(i, Actuator::Mode_Pos);
    } else {
        XR1_ptr->liftLockdown();
        setControlMode(XR1::LeftArm, XR1::PositionMode);
        setControlMode(XR1::RightArm, XR1::PositionMode);
        setControlMode(XR1::MainBody, XR1::PositionMode);
    }


}


void XR1ControllerOL::actuatorOperation(uint8_t nId, uint8_t nType) {

    switch (nType) {
    case Actuator::Recognize_Finished:
        if (m_pController->hasAvailableActuator()) {
            ROS_INFO("Recognized Actuators");
        }
        break;
    case Actuator::Launch_Finished:
        if (allActuatorHasLaunched()) {
            std_msgs::Bool stuff;
            stuff.data = true;
            ActuatorLaunchedPublisher.publish(stuff);

            setControlMode(XR1::OmniWheels, XR1::PositionMode);

            setControlMode(XR1::MainBody, XR1::PositionMode);

            setControlMode(XR1::LeftArm, XR1::PositionMode);

            setControlMode(XR1::RightArm, XR1::PositionMode);

            setControlMode(XR1::LeftHand, XR1::PositionMode);

            setControlMode(XR1::RightHand, XR1::PositionMode);

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
            m_pController->regainAttrbute(i, Actuator::ACTUAL_POSITION);
            // m_pController->regainAttrbute(i, Actuator::ACTUAL_CURRENT);
            m_pController->regainAttrbute(i, Actuator::ACTUAL_VELOCITY);
        }
    }

    for (uint8_t i = XR1::Neck_Z; i < XR1::LeftArm; ++i) {
        if ((int) m_pController->getActuatorAttribute(i, Actuator::INIT_STATE) == Actuator::Initialized) {
            m_pController->regainAttrbute(i, Actuator::ACTUAL_POSITION);
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
                m_pController->regainAttrbute(i, Actuator::ACTUAL_VELOCITY);
            }
        }


        if ((int) m_pController->getActuatorAttribute((uint8_t) XR1::Knee_X, Actuator::INIT_STATE) ==
                Actuator::Initialized) {
            m_pController->regainAttrbute((uint8_t) XR1::Knee_X, Actuator::ACTUAL_POSITION);
        }
    }


    if (power_reading_counter > (200*5)){

        power_reading_counter = 0;

        if ((int) m_pController->getActuatorAttribute((uint8_t)XR1::Back_Y, Actuator::INIT_STATE) == Actuator::Initialized)
            m_pController->regainAttrbute((uint8_t)XR1::Back_Y , Actuator::VOLTAGE);
    }


    hand_command_switch = !hand_command_switch;

}





void XR1ControllerOL::unleaseCallback(const ros::TimerEvent &) {

    readingCallback();

    unleaseJointInfo();

    gravityCallback();

    broadcastTransform();

    if (animation_switch)
        animationCallback();
    else
        stateTransition();

    collisionDetectionCallback();

}




void XR1ControllerOL::unleaseJointInfo(){
    // send out all the current information
    XR1_ptr->getJointPositions(XR1::MainBody, temp_vec7d ,true);
    ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
    MainBodyPositionPublisher.publish(temp_bodymsgs);

    XR1_ptr->getJointCurrents(XR1::MainBody, temp_vec7d , true);
    ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
    MainBodyCurrentPublisher.publish(temp_bodymsgs);

    XR1_ptr->getJointPositions(XR1::LeftArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    LeftArmPositionPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointVelocities(XR1::LeftArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    LeftArmVelocityPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointCurrents(XR1::LeftArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    LeftArmCurrentPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointPositions(XR1::RightArm, temp_vec7d, true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    RightArmPositionPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointVelocities(XR1::RightArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    RightArmVelocityPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointCurrents(XR1::RightArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    RightArmCurrentPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointCurrents(XR1::LeftHand, temp_vec5d, true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    LeftHandPositionPublisher.publish(temp_handmsgs);

    XR1_ptr->getJointCurrents(XR1::RightHand, temp_vec5d , true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    RightHandPositionPublisher.publish(temp_handmsgs);
}




void XR1ControllerOL::broadcastTransform() {

//    //just to be sure
//    XR1_ptr->setInverseDynamicsOption(XR1::GravityCompensation);

    // This function triggers almost all the computation in the library
    XR1_ptr->triggerCalculation(collision_detection_switch);

    // Publish the left one
    XR1_ptr->getEndEffectorTransformation(XR1::LeftArm, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


    // Publish the right one
    XR1_ptr->getEndEffectorTransformation(XR1::RightArm, itsafine);
    // std::cout << itsafine.matrix() << std::endl;
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));



    // Publish the head
    XR1_ptr->getEndEffectorTransformation(XR1::MainBody, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/Head"));



    // Publish the Base
    XR1_ptr->getBaseTransformation(XR1::MainBody, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Base", "/Back_Y"));

}




void XR1ControllerOL::getEndEffectorTransformation(uint8_t control_group, Affine3d &TransformationReference) {
    XR1_ptr->getEndEffectorTransformation(control_group, TransformationReference);
}




void XR1ControllerOL::clearStates() {
    XR1_ptr->clearStates();
}

void XR1ControllerOL::gravityCallback() {


    if (XR1_ptr->getControlMode(XR1::LeftArm) == XR1::ForceMode) {
        for (uint8_t i = XR1::LeftArm; i < XR1::Left_Wrist_Z; i++) {
            ROS_INFO("The Current for joint [%d] is [%f]", (int) i, XR1_ptr->getTargetJointCurrent(i));

            setJointCurrent(i, XR1_ptr->getTargetJointCurrent(i));

        }
    }
//
    if (XR1_ptr->getControlMode(XR1::RightArm) == XR1::ForceMode) {
        for (uint8_t i = XR1::RightArm; i < XR1::Right_Wrist_Z; i++) {
            ROS_INFO("The Current for joint [%d] is [%f]", (int) i, XR1_ptr->getTargetJointCurrent(i));

            setJointCurrent(i, XR1_ptr->getTargetJointCurrent(i));

        }
    }


}
