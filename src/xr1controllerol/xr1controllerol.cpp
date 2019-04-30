#include "xr1controllerol.h"
#include <ros/package.h>
#include <iostream>


XR1ControllerOL::XR1ControllerOL() :
    hand_command_switch(true)
    ,power_reading_counter(30000)
    ,previous_omni_state(false)
    ,collision_detection_switch(false)
    ,RecognizeFinished(false) {

    std::vector<double> sit_pos;

    while (sit_pos.size() < XR1::Actuator_Total)
        sit_pos.push_back(0);



    // Bunch of calculation objects ---------------------------------------
    std::string path = ros::package::getPath("xr1controllerol");

    XR1_ptr = new XR1Controller(path + "/fungus.xr1para", sit_pos);

    XRA_ptr = new XR1ControllerALP(path + "/ALP", XR1_ptr, 169, 10, 1);

    IMU_ptr = new XR1IMUmethods();

    m_pController = ActuatorController::getInstance();

    // --------------------------------------------------------------------



    // Bunch of Target Subscribers --------------------------------------
    LaunchSubscriber = nh.subscribe("/startSimulation", 1, &XR1ControllerOL::subscribeLaunch, this);
    ShutdownSubscriber = nh.subscribe("/stopSimulation", 1, &XR1ControllerOL::subscribeShutdown, this);

    MainBodyPositionSubscriber = nh.subscribe("/MainBody/TargetPosition", 100,
                                              &XR1ControllerOL::subscribeMainBodyPosition, this);
    MainBodyCurrentSubscriber = nh.subscribe("/MainBody/TargetCurrent", 100, &XR1ControllerOL::subscribeMainBodyCurrent,
                                             this);

    HeadBodyPositionSubscriber = nh.subscribe("/HeadBody/TargetPosition", 100,
                                              &XR1ControllerOL::subscribeHeadBodyPosition, this);

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

    LeftHandPositionSubscriber = nh.subscribe("/LeftHand/TargetPosition", 10,
                                              &XR1ControllerOL::subscribeLeftHandPosition, this);
    RightHandPositionSubscriber = nh.subscribe("/RightHand/TargetPosition", 10,
                                               &XR1ControllerOL::subscribeRightHandPosition, this);
    LeftHandCurrentSubscriber = nh.subscribe("/LeftHand/TargetCurrent", 10, &XR1ControllerOL::subscribeLeftHandCurrent,
                                             this);
    RightHandCurrentSubscriber = nh.subscribe("/RightHand/TargetCurrent", 10,
                                              &XR1ControllerOL::subscribeRightHandCurrent, this);

    ModeChangeSubscriber = nh.subscribe("/XR1/ChainModeChange", 10,
                                        &XR1ControllerOL::subscribeRobotMode, this);

    EStopSubscriber = nh.subscribe("XR1/EStop", 1, &XR1ControllerOL::subscribeEStop, this);

    // -----------------------------------------------------------------------





    // Animation callbacks -------------------------------------------------------
    AnimationSwitchSubscriber = nh.subscribe("/startAnimation", 1, &XR1ControllerOL::subscribeStartAnimation, this);
    AnimationSetSubscriber = nh.subscribe("/setAnimation", 1, &XR1ControllerOL::subscribeSetAnimation, this);
    IdleSwitchSubscriber = nh.subscribe("/setIdle", 1, &XR1ControllerOL::subscribeSetIdle, this);
    QueryAnimationService = nh.advertiseService("/queryAnimation", &XR1ControllerOL::serviceQueryAnimation, this);
    // ---------------------------------------------------------------------------



    // Collision Detection set ---------------------------------------------------
    CollisionDetectionSubscriber = nh.subscribe("/setCollisionDetection", 1,
                                                &XR1ControllerOL::subscribeSetCollisionDetection, this);
    // ---------------------------------------------------------------------------




    // Inverse Kinematics callbacks ----------------------------------------------
    IKPlannerService = nh.advertiseService("XR1/IKLPT", &XR1ControllerOL::serviceIKPlanner, this);
    IKTrackingService = nh.advertiseService("XR1/IKTT", &XR1ControllerOL::serviceIKTracking, this);
    HandGripService = nh.advertiseService("XR1/HGQ", &XR1ControllerOL::serviceHandGrip, this);

    // ---------------------------------------------------------------------------


    // Decide if the Robot is ready ------------------------------------------------------------
    ReadinessService = nh.advertiseService("XR1/Ready", &XR1ControllerOL::serviceReady, this);
    // -----------------------------------------------------------------------------------------


    // Joint Information Publishers ---------------------------------------------------
    HeadBodyPositionPublisher = nh.advertise<xr1controllerros::HeadMsgs>("/HeadBody/Position", 1);
    HeadBodyVelocityPublisher = nh.advertise<xr1controllerros::HeadMsgs>("/HeadBody/Velocity", 1);
    HeadBodyCurrentPublisher = nh.advertise<xr1controllerros::HeadMsgs>("/HeadBody/Current", 1);

    MainBodyPositionPublisher = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Position", 1);
    MainBodyVelocityPublisher = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Velocity", 1);
    MainBodyCurrentPublisher = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Current", 1);

    LeftArmPositionPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Position", 1);
    LeftArmVelocityPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Velocity", 1);
    LeftArmCurrentPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Current", 1);

    RightArmPositionPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Position", 1);
    RightArmVelocityPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Velocity", 1);
    RightArmCurrentPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Current", 1);

    LeftHandPositionPublisher = nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/Position", 1);
    RightHandPositionPublisher = nh.advertise<xr1controllerros::HandMsgs>("/RightHand/Position", 1);

    LeftHandCurrentPublisher = nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/Current", 1);
    RightHandCurrentPublisher = nh.advertise<xr1controllerros::HandMsgs>("/RightHand/Current", 1);
    // -------------------------------------------------------------------------------


    // Omni information Publisher and Subscriber -------------------------------------

    OmniSpeedPublisher = nh.advertise<geometry_msgs::Twist>("/OmniWheels/Velocity",1);
    OmniSpeedSubscriber = nh.subscribe("/cmd_vel" , 10 , &XR1ControllerOL::subscribeOmniCommands ,this);

    // -------------------------------------------------------------------------------


    // Getting all the group ID right ------------------------------------------------
    control_group_map[XR1::OmniWheels] = XR1_ptr->getControlGroupIDs(XR1::OmniWheels);

    control_group_map[XR1::MainBody] = XR1_ptr->getControlGroupIDs(XR1::MainBody);

    control_group_map[XR1::HeadBody] = XR1_ptr->getControlGroupIDs(XR1::HeadBody);

    control_group_map[XR1::LeftArm] = XR1_ptr->getControlGroupIDs(XR1::LeftArm);

    control_group_map[XR1::RightArm] = XR1_ptr->getControlGroupIDs(XR1::RightArm);

    control_group_map[XR1::LeftHand] = XR1_ptr->getControlGroupIDs(XR1::LeftHand);

    control_group_map[XR1::RightHand] = XR1_ptr->getControlGroupIDs(XR1::RightHand);


//    control_group_flags.push_back(XR1::OmniWheels);
    control_group_flags.push_back(XR1::MainBody);
    control_group_flags.push_back(XR1::HeadBody);
    control_group_flags.push_back(XR1::LeftArm);
    control_group_flags.push_back(XR1::RightArm);
    control_group_flags.push_back(XR1::LeftHand);
    control_group_flags.push_back(XR1::RightHand);
    // -----------------------------------------------------------------------------





//    mode_map[XR1::PositionMode] = Actuator::Mode_Profile_Pos;
//    mode_map[XR1::VelocityMode] = Actuator::Mode_Profile_Vel;
//    mode_map[XR1::ForceMode] = Actuator::Mode_Cur;
//    mode_map[XR1::IKMode] = Actuator::Mode_Profile_Pos;

// Buffered control modes -----------------------------
    control_modes[XR1::OmniWheels] = 0;
    control_modes[XR1::MainBody] = 0;
    control_modes[XR1::LeftArm] = 0;
    control_modes[XR1::RightArm] = 0;
    control_modes[XR1::LeftHand] = 0;
    control_modes[XR1::RightHand] = 0;
    control_modes[XR1::HeadBody] = 0;
    control_modes[XR1::MainBody] = 0;
    // --------------------------------------------------


    //Actuators Update Callback --------------------------------
    m_pController->m_sActuatorAttrChanged->connect_member(this, &XR1ControllerOL::updatingCallback);

    m_pController->m_sOperationFinished->connect_member(this, &XR1ControllerOL::actuatorOperation);

    attribute_map[Actuator::ACTUAL_POSITION] = XR1::ActualPosition;
    attribute_map[Actuator::ACTUAL_VELOCITY] = XR1::ActualVelocity;
    attribute_map[Actuator::ACTUAL_CURRENT] = XR1::ActualCurrent;

    // -----------------------------------------------




    // Hopps Port integration -----------------------------

    setupJointStateTable();

    uint8_t temp_id = 1;

    while (temp_id < XR1::MainBody) { m_control_group_lookup[temp_id] = XR1::OmniWheels ;  temp_id++;}

    while (temp_id < XR1::HeadBody) { m_control_group_lookup[temp_id] = XR1::MainBody ;  temp_id++;}

    while (temp_id < XR1::LeftArm) { m_control_group_lookup[temp_id] = XR1::HeadBody ;  temp_id++;}

    while (temp_id < XR1::RightArm) { m_control_group_lookup[temp_id] = XR1::LeftArm ;  temp_id++;}

    while (temp_id < XR1::LeftHand) { m_control_group_lookup[temp_id] = XR1::RightArm ;  temp_id++;}

    while (temp_id < XR1::RightHand) { m_control_group_lookup[temp_id] = XR1::LeftHand ;  temp_id++;}

    while (temp_id < XR1::Actuator_Total) { m_control_group_lookup[temp_id] = XR1::RightHand ;  temp_id++;}


    m_joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/ginger/joint_states", 10);

    m_joint_state_subscriber = nh.subscribe("/joint_states", 10 , &XR1ControllerOL::subscribeJointStates , this);


    // ----------------------------------------------------








    temp_vec5d = VectorXd::Zero(5);
    temp_vec7d = VectorXd::Zero(7);
    temp_vec3d = VectorXd::Zero(3);
    temp_vec4d = VectorXd::Zero(4);

    XR1_ptr->setInverseDynamicsOption(XR1::GravityCompensation);



    // Legacy ---------------------------------------------------------------------

//    tiltInitSubscriber = nh.subscribe("XR1/tiltInit", 1, &XR1ControllerOL::subscribetiltInit, this);
//    MoCapInitSubscriber = nh.subscribe("XR1/MoCapInit", 1, &XR1ControllerOL::subscribeMoCapInit, this);
    // m_pController->m_sAcceleration->connect_member(this, &XR1ControllerOL::accCallBack);
    //    m_pController->m_sQuaternionL->connect_member(this, &XR1ControllerOL::QuaCallBack);
    // ----------------------------------------------------------------------------


    ROS_INFO("OL Constructor finished");
}

bool XR1ControllerOL::serviceReady(xr1controllerol::askReadinessRequest & req,
                  xr1controllerol::askReadinessResponse & res){

    if (req.isAsking){

        ROS_INFO("Oh shit them asking");


        std::vector<uint8_t> unava_act;

        for (int i = XR1::OmniWheels ; i < XR1::Actuator_Total ; i++){

            if ((int)m_pController->getActuatorAttribute((uint8_t)i, Actuator::INIT_STATE) == Actuator::Initialized){

            }

            else {
                unava_act.push_back((uint8_t)i);
            }

        }


        for (uint8_t ids : unava_act){
            m_pController->closeActuator( ids );
        }

        ros::Duration(0.5).sleep();

        for (uint8_t ids : unava_act){
            m_pController->launchActuator( ids);
        }

    }


    for (int i = XR1::MainBody ; i < XR1::Actuator_Total ; i++){

        if ((int)m_pController->getActuatorAttribute((uint8_t)i, Actuator::INIT_STATE) == Actuator::Initialized) {


        }

        else {

            ROS_INFO("Oh shit them not launched fam");

             res.isReady = false;
             return true;
        }

    }

    ROS_INFO("It is ON SON");

    res.isReady = true;





    return true;
}



XR1ControllerOL::~XR1ControllerOL() {
    // unregister all publishers here

    HeadBodyPositionPublisher  .shutdown();
    HeadBodyVelocityPublisher .shutdown();
    HeadBodyCurrentPublisher.shutdown();
    MainBodyPositionPublisher .shutdown();
    MainBodyVelocityPublisher .shutdown();
    MainBodyCurrentPublisher.shutdown();
    LeftArmPositionPublisher .shutdown();
    LeftArmVelocityPublisher .shutdown();
    LeftArmCurrentPublisher.shutdown();
    RightArmPositionPublisher .shutdown();
    RightArmVelocityPublisher .shutdown();
    RightArmCurrentPublisher.shutdown();
    LeftHandPositionPublisher .shutdown();
    RightHandPositionPublisher.shutdown();
    LeftHandCurrentPublisher .shutdown();
    RightHandCurrentPublisher .shutdown();

}




void XR1ControllerOL::startSimulation() {
    launchAllMotors();
}


void XR1ControllerOL::stopSimulation() {


    control_modes[XR1::MainBody] = 0;
    control_modes[XR1::LeftArm] = 0;
    control_modes[XR1::RightArm] = 0;
    control_modes[XR1::LeftHand] = 0;
    control_modes[XR1::RightHand] = 0;
    control_modes[XR1::HeadBody] = 0;

    stopAllMotors();
}






void XR1ControllerOL::subscribeLaunch(const std_msgs::Bool &msg) {
    launchAllMotors();
}


void XR1ControllerOL::subscribeShutdown(const std_msgs::Bool &msg) {
    stopAllMotors();
}



void XR1ControllerOL::subscribeEStop(const std_msgs::Bool &msg) {

    // set true to lock the robot
    if (msg.data) {
        XR1_ptr->employLockdown();
        for (uint8_t i = XR1::OmniWheels; i < XR1::Actuator_Total; i++)
            m_pController->activateActuatorMode(i, Actuator::Mode_Pos);
    }
    // set false to unlock the robot
    else {
        XR1_ptr->liftLockdown();
        setControlMode(XR1::LeftArm, XR1::DirectMode);
        setControlMode(XR1::RightArm, XR1::DirectMode);
        setControlMode(XR1::MainBody, XR1::DirectMode);
        setControlMode(XR1::HeadBody, XR1::DirectMode);
        setControlMode(XR1::LeftHand, XR1::DirectMode);
        setControlMode(XR1::RightHand, XR1::DirectMode);
    }


}





void XR1ControllerOL::unleaseCallback(const ros::TimerEvent &) {

    // Things to do On each loop

    // request to read all the values
    readingCallback();

    // check all the control modes
    judgeControlGroupModes();

    // calculate all the tf info and dynamics stuff
    broadcastTransform();

    // send out all the joint information
    unleaseJointInfo();


    // unlease joint states information
    publishJointStates();


    // collision detection check
    collisionDetectionCallback();

    // assign next animation step
    animationCallback();

    // apply high frequency target if it exists
    applyJointsTargets();

    // send out omni commands if they are in the right modes
    Omni2Actuator();

}




void XR1ControllerOL::unleaseJointInfo(){
    // send out all the current information



    XR1_ptr->getJointPositions(XR1::MainBody, temp_vec7d ,true);
    ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
    MainBodyPositionPublisher.publish(temp_bodymsgs);

    XR1_ptr->getJointVelocities(XR1::MainBody, temp_vec7d ,true);
    ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
    MainBodyVelocityPublisher.publish(temp_bodymsgs);

    XR1_ptr->getJointCurrents(XR1::MainBody, temp_vec7d , true);
    ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
    MainBodyCurrentPublisher.publish(temp_bodymsgs);


    XR1_ptr->getJointPositions(XR1::HeadBody, temp_vec7d ,true);
    ConvertHeadMsgs(temp_vec7d , temp_headmsgs);
    HeadBodyPositionPublisher.publish(temp_headmsgs);

    XR1_ptr->getJointVelocities(XR1::HeadBody, temp_vec7d ,true);
    ConvertHeadMsgs(temp_vec7d , temp_headmsgs);
    HeadBodyPositionPublisher.publish(temp_headmsgs);

    XR1_ptr->getJointCurrents(XR1::HeadBody, temp_vec7d , true);
    ConvertHeadMsgs(temp_vec7d , temp_headmsgs);
    HeadBodyPositionPublisher.publish(temp_headmsgs);


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

    XR1_ptr->getJointPositions(XR1::LeftHand, temp_vec5d, true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    LeftHandPositionPublisher.publish(temp_handmsgs);

    XR1_ptr->getJointPositions(XR1::RightHand, temp_vec5d , true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    RightHandPositionPublisher.publish(temp_handmsgs);

    XR1_ptr->getJointCurrents(XR1::LeftHand, temp_vec5d, true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    LeftHandCurrentPublisher.publish(temp_handmsgs);

    XR1_ptr->getJointCurrents(XR1::RightHand, temp_vec5d , true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    RightHandCurrentPublisher.publish(temp_handmsgs);

}




void XR1ControllerOL::broadcastTransform() {

    // This function triggers almost all the computation in the library
    XR1_ptr->triggerCalculation(true);

    // Publish the left one
    XR1_ptr->getEndEffectorTransformation(XR1::LeftArm, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


    // Publish the right one
    XR1_ptr->getEndEffectorTransformation(XR1::RightArm, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));


    // Publish the head
    XR1_ptr->getEndEffectorTransformation(XR1::HeadBody, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/Head"));


    XR1_ptr->getBaseTransformation(XR1::OmniWheels, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Odom", "/Base"));


    XR1_ptr->getBaseTransformation(XR1::MainBody, itsafine);
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Base", "/Back_Y"));

}




void XR1ControllerOL::getEndEffectorTransformation(uint8_t control_group, Affine3d &TransformationReference) {
    XR1_ptr->getEndEffectorTransformation(control_group, TransformationReference);
}

