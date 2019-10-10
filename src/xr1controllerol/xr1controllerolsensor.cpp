//
// Created by waldfeng on 19-9-30.
//
#include "xr1sensors.h"
#include "../../include/xr1controllerol/xr1controllerolmsgulit.h"

XR1Sensors::XR1Sensors(ros::NodeHandle &nh, XR1ControllerBLC *XRB_ptr, ActuatorController *m_pController){

    XRB4Sensor_ptr = XRB_ptr;
    m_pController4Sensor = m_pController;

    battery_publish_interval = 2000;
    sensor_publish_interval = 6;
    tempSensorIP = "192.168.1.4";
    temp_acc << 0.0,0.0,0.0;

    //sensors msg publisher
    m_IMUPublisher = nh.advertise<sensor_msgs::Imu>("/XR1/IMU" , 32 );
    BatteryInfoPublisher = nh.advertise<xr1controllerros::BatteryState>("/XR1/BatteryInfo" , 1, true);
    UltrasonicPublisher = nh.advertise<xr1controllerros::Ultrasonic>("/XR1/Ultrasonic" , 1, true);
    DropCollisionDetectPublisher = nh.advertise<xr1controllerros::DropCollision>("/XR1/DropCollisionDetect" , 1, true);
    SmogPublisher = nh.advertise<std_msgs::Float32>("/XR1/Smog" , 1, true);
    PM25Publisher = nh.advertise<std_msgs::UInt16>("/XR1/PM25" , 1, true);
    TemperaturePublisher = nh.advertise<xr1controllerros::Temperature>("/XR1/Temperature" , 1, true);

    //get temperature sensor ip
    ros::Subscriber tempSensorIP  = nh.subscribe("/Sensors/IP" , 1, &XR1Sensors::subscribeTempSensorIP,this);

    std::cout << endl << "XR1Sensors init finished" << endl;
}


void XR1Sensors::requestSensorQue( uint16_t low_frequency_counter ) {

    //IMU request frequency = 200hz
    m_pController4Sensor->requestSingleQuaternion(ActuatorController::toLongId("192.168.1.4" , 0));

    //ultrasonic PM2.5 smog dropcollision temperature sensors request requency = 33hz
    if ((low_frequency_counter % sensor_publish_interval) == 0){  //33hz
        m_pController4Sensor->requestSensor(ULTRASONIC,"192.168.1.4");
        m_pController4Sensor->requestSensor(PM,"192.168.1.4");
        m_pController4Sensor->requestSensor(SMOG,"192.168.1.4");
        m_pController4Sensor->requestSensor(DROPCOLLISION,"192.168.1.4");
    }

    //battery info request every 10s
    if ((low_frequency_counter % battery_publish_interval) == 0){
        publishBatteryInfo();
        m_pController4Sensor->requestSensor(TEMPERATURE,tempSensorIP);
    }

    //std::cout << "we called request Que\n";
}

void XR1Sensors::QuaCallBack(uint64_t id, double w, double x, double y, double z) {

    // if (precision > 1){

    // If it is the base frame
    if (id == ActuatorController::toLongId("192.168.1.4", 0)){
//        ROS_INFO("[%f][%f][%f][%f]",w,x,y,z);

        temp_qua.x() = x;
        temp_qua.y() = y;
        temp_qua.z() = z;
        temp_qua.w() = w;

        // publish the last buffered acceleration and quaternion
        tf::quaternionEigenToMsg(temp_qua, temp_orientation);
        tf::vectorEigenToMsg(temp_acc , temp_linear_acceleration);
        temp_imu_msg.orientation = temp_orientation;
        temp_imu_msg.linear_acceleration = temp_linear_acceleration;
        temp_imu_msg.header.stamp = ros::Time::now();

        m_IMUPublisher.publish(temp_imu_msg);

        XRB4Sensor_ptr->tiltCallback(w, x, y, z);
    }

}

//acc callback function
void XR1Sensors::accCallBack(uint8_t id , double x , double y , double z , int pres){
//   ROS_INFO("[%d][%f][%f][%f]",pres,x,y,z);

    static double gravity_g = 9.81;

    temp_acc << x*gravity_g, y*gravity_g, z*gravity_g;
    //std::cout <<"\n got temp_acc: "<<temp_acc;
 }

//read and public sensors info: BatteryInfo PM25 SMOG Temperature Ultrasonic etc.
void XR1Sensors::publishBatteryInfo(void) {
    //publish batter info
    xr1controllerros::BatteryState BatteryInfoRos;
    BatteryStatus * BatteryInfo_now = m_pController4Sensor->readBatteryStatus("192.168.1.4");
    ConvertBatteryInfoMsgs( BatteryInfo_now, BatteryInfoRos);
    BatteryInfoRos.header.stamp = ros::Time::now();
    BatteryInfoPublisher.publish(BatteryInfoRos);
}

void XR1Sensors::ultrasonicCallback(std::vector<Ultrasonic> ucVector){
    //public Ultrasonic data
    xr1controllerros::Ultrasonic UltrasonicRos;
    ConvertUltrasonicMsgs(ucVector, UltrasonicRos);
    UltrasonicRos.header.stamp = ros::Time::now();
    UltrasonicPublisher.publish(UltrasonicRos);
}

void XR1Sensors::dropCollisionCallback(std::vector<DropCollision> dcVector){
    //public dropCollision data
    xr1controllerros::DropCollision DropCollisionRos;
    ConvertDropCollisionDetectMsgs(dcVector, DropCollisionRos);
    DropCollisionRos.header.stamp = ros::Time::now();
    DropCollisionDetectPublisher.publish(DropCollisionRos);
}

void XR1Sensors::PM25Callback(uint16_t pm){
    //publish PM25 data
    std_msgs::UInt16 PM25Ros;
    PM25Ros.data = pm;
    PM25Publisher.publish(PM25Ros);

}

void XR1Sensors::SmogCallback(double smog){
    //public Smog data
    std_msgs::Float32 smogRos;
    smogRos.data = float(smog);
    SmogPublisher.publish(smogRos);

}

void XR1Sensors::temperatureCallback(std::string ipStr, double temp){
    //public temperature data
    xr1controllerros::Temperature temperatureRos;
    temperatureRos.IP = ipStr;
    temperatureRos.Temperature = float(temp);
    temperatureRos.header.stamp = ros::Time::now();
    TemperaturePublisher.publish(temperatureRos);
}

void XR1Sensors::subscribeTempSensorIP(const std_msgs::String &TempSensorIPMsg){

    tempSensorIP = TempSensorIPMsg.data;
}

