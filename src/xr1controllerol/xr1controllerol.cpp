#include "xr1controllerol.h"
#include <ros/package.h>
#include <iostream>

XR1ControllerOL::XR1ControllerOL() :
        m_b100Hz_switch(true)
    ,power_reading_counter(30000)
    ,previous_omni_state(0)
    ,omni_cmd_expire_counter(0)
    ,collision_detection_switch(true)
    ,RecognizeFinished(false),
    unlease_counter(0),
    low_frequency_threshold(5),
    low_frequency_counter(0)
    ,debug_switch(false){


    // Bunch of calculation objects ---------------------------------------
    std::string path = ros::package::getPath("xr1controllerol");


    XRB_ptr = new XR1ControllerBLC(path + "/BLC" ,path + "/ALP" );

    XRB_ptr->setIdle(false);
    XRB_ptr->setActive(false);
    XRB_ptr->setPassive(false);
    string param_file;
    tf_switch = 0;
    ros::param::get("/actuator_bridge/param_file",param_file);
    ros::param::get("/actuator_bridge/tf_switch",tf_switch);
    ROS_INFO("param: param_file is %s", param_file.c_str());
    ROS_INFO("param: tf_switch  is %d", tf_switch);

    if (param_file.length()){
        std::cout<< " Parameter file given is: " + param_file << std::endl;

    }else{
        std::cout<< " No paramters given!!!!!! Using EmeraldEndive version" << std::endl;
//        abort();
        param_file = "EmeraldEndive.xr1para";
    }

    if (tf_switch){
        std::cout<< " Parameter tf_switch given is: " << tf_switch << std::endl;

    }else{
        std::cout<< " No paramters given!!!!!! default open tf_switch" << std::endl;
//        abort();
        tf_switch = 1;
    }


    XR1_ptr = new XR1Controller(path + "/xr1paras/" + param_file);

    XRA_ptr = new XR1ControllerALP(path + "/ALP", XR1_ptr, XRB_ptr);

    IMU_ptr = new XR1IMUmethods();

    m_pController = ActuatorController::getInstance();

    // --------------------------------------------------------------------



    // Bunch of Target Subscribers --------------------------------------
    LaunchSubscriber = nh.subscribe("/startSimulation", 1, &XR1ControllerOL::subscribeLaunch, this);
    ShutdownSubscriber = nh.subscribe("/stopSimulation", 1, &XR1ControllerOL::subscribeShutdown, this);

    OmniCurrentSubscriber = nh.subscribe("/Wheel/TargetCurrent", 100,
                                              &XR1ControllerOL::subscribeWheelCurrent, this);

    MainBodyPositionSubscriber = nh.subscribe("/MainBody/TargetPosition", 100,
                                              &XR1ControllerOL::subscribeMainBodyPosition, this);
    MainBodyCurrentSubscriber = nh.subscribe("/MainBody/TargetEffort", 100, &XR1ControllerOL::subscribeMainBodyCurrent,
                                             this);

    HeadBodyPositionSubscriber = nh.subscribe("/HeadBody/TargetPosition", 100,
                                              &XR1ControllerOL::subscribeHeadBodyPosition, this);

    LeftArmPositionSubscriber = nh.subscribe("/LeftArm/TargetPosition", 100, &XR1ControllerOL::subscribeLeftArmPosition,
                                             this);
    LeftArmVelocitySubscriber = nh.subscribe("/LeftArm/TargetVelocity", 100, &XR1ControllerOL::subscribeLeftArmVelocity,
                                             this);
    LeftArmCurrentSubscriber = nh.subscribe("/LeftArm/TargetEffort", 100, &XR1ControllerOL::subscribeLeftArmCurrent,
                                            this);

    RightArmPositionSubscriber = nh.subscribe("/RightArm/TargetPosition", 100,
                                              &XR1ControllerOL::subscribeRightArmPosition, this);
    RightArmVelocitySubscriber = nh.subscribe("/RightArm/TargetVelocity", 100,
                                              &XR1ControllerOL::subscribeRightArmVelocity, this);
    RightArmCurrentSubscriber = nh.subscribe("/RightArm/TargetEffort", 100, &XR1ControllerOL::subscribeRightArmCurrent,
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
    XRA_ptr->m_sAnimationFinished.subscribeSignal(this, &XR1ControllerOL::signalAnimationFinished);
    AnimationResultPublisher = nh.advertise<xr1controllerol::AnimationMsgs>("/AnimationResult" , 1 );
    AnimationSwitchSubscriber = nh.subscribe("/startAnimation", 1, &XR1ControllerOL::subscribeStartAnimation, this);
    AnimationSetSubscriber = nh.subscribe("/setAnimation", 1, &XR1ControllerOL::subscribeSetAnimation, this);
    IdleSwitchSubscriber = nh.subscribe("/setIdleAnimations", 1, &XR1ControllerOL::subscribeSetIdle, this);
    DefaultSwitchSubscriber = nh.subscribe("/setDefaultAnimation", 1, &XR1ControllerOL::subscribeSetDefault, this);
    QueryAnimationService = nh.advertiseService("/queryAnimation", &XR1ControllerOL::serviceQueryAnimation, this);
    OverwriteAnimationService = nh.advertiseService("/overwriteAnimation", &XR1ControllerOL::serviceOverwriteAnimation, this);
    // ---------------------------------------------------------------------------



    // Collision Detection set ---------------------------------------------------
    CollisionDetectionSubscriber = nh.subscribe("/setCollisionDetection", 1,
                                                &XR1ControllerOL::subscribeSetCollisionDetection, this);

    CollisionEventPublisher = nh.advertise<std_msgs::Bool>("/CollisionEvent",1);
    // ---------------------------------------------------------------------------




    // Inverse Kinematics callbacks ----------------------------------------------
    IKPlannerService = nh.advertiseService("XR1/IKPlanner", &XR1ControllerOL::serviceIKPlanner, this);
    IKLinearPlannerService = nh.advertiseService("XR1/IKLPT", &XR1ControllerOL::serviceIKLinearPlanner, this);
    IKTrackingService = nh.advertiseService("XR1/IKTT", &XR1ControllerOL::serviceIKTracking, this);
    HandGripService = nh.advertiseService("XR1/HGQ", &XR1ControllerOL::serviceHandGrip, this);
    // ---------------------------------------------------------------------------


    // Decide if the Robot is ready ------------------------------------------------------------
    ReadinessService = nh.advertiseService("XR1/Ready", &XR1ControllerOL::serviceReady, this);
    // -----------------------------------------------------------------------------------------

    // Report the robot's state ------------------------------------------------------------
    RobotStateService = nh.advertiseService("XR1/State" , &XR1ControllerOL::serviceState, this);
    // -------------------------------------------------------------------------------------


    // Joint Information Publishers ---------------------------------------------------
    HeadBodyPositionPublisher = nh.advertise<xr1controllerros::HeadMsgs>("/HeadBody/Position", 1);
    HeadBodyVelocityPublisher = nh.advertise<xr1controllerros::HeadMsgs>("/HeadBody/Velocity", 1);
    HeadBodyCurrentPublisher = nh.advertise<xr1controllerros::HeadMsgs>("/HeadBody/Effort", 1);

    MainBodyPositionPublisher = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Position", 1);
    MainBodyVelocityPublisher = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Velocity", 1);
    MainBodyCurrentPublisher = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/Effort", 1);

    LeftArmPositionPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Position", 1);
    LeftArmVelocityPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Velocity", 1);
    LeftArmCurrentPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/Effort", 1);

    RightArmPositionPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Position", 1);
    RightArmVelocityPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Velocity", 1);
    RightArmCurrentPublisher = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/Effort", 1);

    LeftHandPositionPublisher = nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/Position", 1);
    RightHandPositionPublisher = nh.advertise<xr1controllerros::HandMsgs>("/RightHand/Position", 1);

    LeftHandCurrentPublisher = nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/Current", 1);
    RightHandCurrentPublisher = nh.advertise<xr1controllerros::HandMsgs>("/RightHand/Current", 1);
    // -------------------------------------------------------------------------------
    voltagePub = nh.advertise<std_msgs::Float32>("ginger_actuator/voltage",1);

    // Omni information Publisher and Subscriber -------------------------------------

    OmniSpeedPublisher = nh.advertise<geometry_msgs::Twist>("/OmniWheels/Velocity",1);
    OmniSpeedSubscriber = nh.subscribe("/XR1/cmd_vel" , 10 , &XR1ControllerOL::subscribeOmniCommands ,this);

    slamIdleSubscriber =  nh.subscribe("/BLC/setIdle" , 10 , &XR1ControllerOL::subscribeBLCIdle ,this);
    slamActiveSubscriber =  nh.subscribe("/BLC/setActive" , 10 , &XR1ControllerOL::subscribeBLCActive ,this);
    slamPassiveSubscriber =  nh.subscribe("/BLC/setPassive" , 10 , &XR1ControllerOL::subscribeBLCPassive ,this);

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

    while (actuator_ready_4_cvp.size() < XR1::Actuator_Total)
        actuator_ready_4_cvp.push_back(false);

    m_pController->m_sActuatorAttrChanged->connect_member(this, &XR1ControllerOL::updatingCallback);

    m_pController->m_sOperationFinished->connect_member(this, &XR1ControllerOL::actuatorOperation);

    attribute_map[Actuator::ACTUAL_POSITION] = XR1::ActualPosition;
    attribute_map[Actuator::ACTUAL_VELOCITY] = XR1::ActualVelocity;
    attribute_map[Actuator::ACTUAL_CURRENT] = XR1::ActualEffort;

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

    m_special_subscriber = nh.subscribe("/XR1/special_command" , 10 , &XR1ControllerOL::subscribeSpecial , this);

    m_IMUPublisher = nh.advertise<sensor_msgs::Imu>("/Base/IMU" , 32 );

    temp_acc << 0,0,0;
    // ----------------------------------------------------








    temp_vec5d = VectorXd::Zero(5);
    temp_vec7d = VectorXd::Zero(7);
    temp_vec3d = VectorXd::Zero(3);
    temp_vec4d = VectorXd::Zero(4);

    XR1_ptr->setInverseDynamicsOption(XR1::FullDynamics_PASSIVE);

    XRA_ptr->setSingleTransitionPeriod(3);



    // Legacy ---------------------------------------------------------------------

    tiltStartSubscriber = nh.subscribe("/startTilting", 1, &XR1ControllerOL::subscribeTiltStart, this);
    slamStartSubscriber = nh.subscribe("/startSLAMing", 1, &XR1ControllerOL::subscribeSLAMStart, this);
    QueryBalanceService = nh.advertiseService("/queryBalance", &XR1ControllerOL::serviceQueryBalance, this);
//    MoCapInitSubscriber = nh.subscribe("XR1/MoCapInit", 1, &XR1ControllerOL::subscribeMoCapInit, this);
     m_pController->m_sAcceleration->connect_member(this, &XR1ControllerOL::accCallBack);
     m_pController->m_sQuaternionL->connect_member(this, &XR1ControllerOL::QuaCallBack);
    // ----------------------------------------------------------------------------





    // temp debug -----------------------------------------------------------------

    RecordCommandSubscriber    = nh.subscribe("/XR1/RecordDebug" , 1, &XR1ControllerOL::subscribeRecordCommand,this);

    WriteCommandSubscriber     = nh.subscribe("/XR1/WriteDebug" , 1, &XR1ControllerOL::subscribeWriteCommand,this);

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


bool XR1ControllerOL::serviceState(xr1controllerol::RobotStateQueryRequest & req,
                  xr1controllerol::RobotStateQueryResponse & res){

    res.isOkay = false;
    res.RobotState = false;
    res.CollisionSwitch = false;
    res.HeadBodyMode = 0;
    res.MainBodyMode = 0;
    res.LeftArmMode = 0;
    res.RightArmMode = 0;
    res.LeftHandMode = 0;
    res.RightHandMode = 0;



    if (req.isQuery){

        res.isOkay = XR1_ptr->isXR1Okay();

        res.CollisionSwitch = collision_detection_switch;

        res.RobotState = XR1_ptr->getErrorCode();

        res.HeadBodyMode = XR1_ptr->getSubControlMode(XR1::HeadBody);
        res.MainBodyMode = XR1_ptr->getSubControlMode(XR1::MainBody);
        res.LeftArmMode  = XR1_ptr->getSubControlMode(XR1::LeftArm);
        res.RightArmMode = XR1_ptr->getSubControlMode(XR1::RightArm);
        res.LeftHandMode = XR1_ptr->getSubControlMode(XR1::LeftHand);
        res.RightHandMode= XR1_ptr->getSubControlMode(XR1::RightHand);

    }


    if (req.requestLift){

        if (XR1_ptr->isXR1Okay()){
            ROS_INFO("y tho");
        }
        else{
            XR1_ptr->liftLockdown();


            setControlMode(XR1::LeftArm, XR1::DirectMode);
            setControlMode(XR1::RightArm, XR1::DirectMode);
            setControlMode(XR1::MainBody, XR1::DirectMode);
            setControlMode(XR1::HeadBody, XR1::DirectMode);
            setControlMode(XR1::LeftHand, XR1::DirectMode);
            setControlMode(XR1::RightHand, XR1::DirectMode);
        }

    }


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
    m_pController->setBrakeStatus( false );
    stopAllMotors();
}



void XR1ControllerOL::subscribeEStop(const std_msgs::Bool &msg) {

    // set true to lock the robot
    if (msg.data) {
        XR1_ptr->employLockdown();
        XRA_ptr->clearAll();
        for (uint8_t i = XR1::OmniWheels; i < XR1::Actuator_Total; i++)
            m_pController->activateActuatorMode(i, Actuator::Mode_Pos);
    }
    // set false to unlock the robot
    else {
        XR1_ptr->liftLockdown();
        XRA_ptr->clearAll();
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

    requestQue();

    // request to read all the values
    readingCallback();

    // check all the control modes
    judgeControlGroupModes();


    // This function triggers almost all the computation in the library
    XR1_ptr->triggerCalculation(true);


    low_frequency_counter++;

    if (low_frequency_counter > low_frequency_threshold ) low_frequency_counter = 0;

    if (low_frequency_counter == 0){
        // calculate all the tf info and dynamics stuff
        if(tf_switch){
            broadcastTransform();
        }
    
        // send out all the joint information
        unleaseJointInfo();

        publishOmni();
    }

    if (m_b100Hz_switch){
        // unlease joint states information
        publishJointStates();
    }



    // collision detection check
    collisionDetectionCallback();

    // assign next animation step
    animationCallback();

    // apply high frequency target if it exists
    applyJointsTargets();

    // send out omni commands if they are in the right modes
    Omni2Actuator();


    recorderDebug();

}




void XR1ControllerOL::unleaseJointInfo(){
    // send out all the current information



    XR1_ptr->getJointPositions(XR1::MainBody, temp_vec7d ,true);
    ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
    MainBodyPositionPublisher.publish(temp_bodymsgs);

    XR1_ptr->getJointVelocities(XR1::MainBody, temp_vec7d ,true);
    ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
    MainBodyVelocityPublisher.publish(temp_bodymsgs);

    XR1_ptr->getJointEfforts(XR1::MainBody, temp_vec7d , true);
    ConvertBodyMsgs(temp_vec7d , temp_bodymsgs);
    MainBodyCurrentPublisher.publish(temp_bodymsgs);


    XR1_ptr->getJointPositions(XR1::HeadBody, temp_vec7d ,true);
    ConvertHeadMsgs(temp_vec7d , temp_headmsgs);
    HeadBodyPositionPublisher.publish(temp_headmsgs);

    XR1_ptr->getJointVelocities(XR1::HeadBody, temp_vec7d ,true);
    ConvertHeadMsgs(temp_vec7d , temp_headmsgs);
    HeadBodyPositionPublisher.publish(temp_headmsgs);

    XR1_ptr->getJointEfforts(XR1::HeadBody, temp_vec7d , true);
    ConvertHeadMsgs(temp_vec7d , temp_headmsgs);
    HeadBodyPositionPublisher.publish(temp_headmsgs);


    XR1_ptr->getJointPositions(XR1::LeftArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    LeftArmPositionPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointVelocities(XR1::LeftArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    LeftArmVelocityPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointEfforts(XR1::LeftArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    LeftArmCurrentPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointPositions(XR1::RightArm, temp_vec7d, true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    RightArmPositionPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointVelocities(XR1::RightArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    RightArmVelocityPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointEfforts(XR1::RightArm, temp_vec7d , true);
    ConvertArmMsgs(temp_vec7d , temp_armmsgs);
    RightArmCurrentPublisher.publish(temp_armmsgs);

    XR1_ptr->getJointPositions(XR1::LeftHand, temp_vec5d, true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    LeftHandPositionPublisher.publish(temp_handmsgs);

    XR1_ptr->getJointPositions(XR1::RightHand, temp_vec5d , true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    RightHandPositionPublisher.publish(temp_handmsgs);

    XR1_ptr->getJointEfforts(XR1::LeftHand, temp_vec5d, true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    LeftHandCurrentPublisher.publish(temp_handmsgs);

    XR1_ptr->getJointEfforts(XR1::RightHand, temp_vec5d , true);
    ConvertHandMsgs(temp_vec5d , temp_handmsgs);
    RightHandCurrentPublisher.publish(temp_handmsgs);

}




void XR1ControllerOL::broadcastTransform() {


    // Publish the left one
    XR1_ptr->getEndEffectorTransformation(XR1::LeftArm, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/LeftEndEffector"));


    // Publish the right one
    XR1_ptr->getEndEffectorTransformation(XR1::RightArm, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/RightEndEffector"));


    // Publish the head
    XR1_ptr->getEndEffectorTransformation(XR1::HeadBody, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Back_Y", "/Head"));


    XR1_ptr->getBaseTransformation(XR1::OmniWheels, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Odom", "/Base"));


    XR1_ptr->getBaseTransformation(XR1::MainBody, itsafine);
//    std::cout << itsafine.matrix() <<std::endl;
    tf::transformEigenToTF(itsafine, transform);
    EFF_Broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Base", "/Back_Y"));

}




void XR1ControllerOL::getEndEffectorTransformation(uint8_t control_group, Affine3d &TransformationReference) {
    XR1_ptr->getEndEffectorTransformation(control_group, TransformationReference);
}

