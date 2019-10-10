#ifndef XR1SENSORS_H
#define XR1SENSORS_H

#include "Eigen/Dense"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "actuatorcontroller.h"
#include "xr1controllerblc.h"

//standard ros message
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

//Message for sensors
#include <sensor_msgs/Imu.h>
#include <xr1controllerros/Ultrasonic.h>
#include <xr1controllerros/BatteryState.h>
#include <xr1controllerros/DropCollision.h>
#include <xr1controllerros/Temperature.h>

using namespace Eigen;

class XR1Sensors{

public:
    XR1Sensors(ros::NodeHandle &nh, XR1ControllerBLC *XRB_ptr, ActuatorController *m_pController);

    void requestSensorQue( uint16_t low_frenqueny_counter );
    //read and public sensors info: IMU BatteryInfo PM25 SMOG Temperature Ultrasonic etc.
    void accCallBack(uint8_t id , double x , double y , double z , int pres);

    void QuaCallBack(uint64_t id, double w, double x, double y, double z);

    void publishBatteryInfo(void);

    void ultrasonicCallback(std::vector<Ultrasonic> ucVector);

    void dropCollisionCallback(std::vector<DropCollision> dcVector);

    void PM25Callback(uint16_t pm);

    void SmogCallback(double smog);

    void temperatureCallback(std::string ipStr,double temp);

    //public variables
    Quaterniond temp_qua;
    Vector3d temp_acc;

private:
    //this pointer
    static XR1Sensors *xr1sensors_this;

    //static xr1olsensor_ptr;
    XR1ControllerBLC *XRB4Sensor_ptr;
    ActuatorController *m_pController4Sensor;
    //sensors ros msgs define
    geometry_msgs::Quaternion temp_orientation;
    geometry_msgs::Vector3 temp_linear_acceleration;
    sensor_msgs::Imu temp_imu_msg;

    //global variables
    std::string tempSensorIP;
    uint16_t sensor_publish_interval;
    uint16_t battery_publish_interval;

    // Sensor messages publisher ------------------
    ros::Publisher m_IMUPublisher;
    ros::Publisher BatteryInfoPublisher;
    ros::Publisher UltrasonicPublisher;
    ros::Publisher DropCollisionDetectPublisher;
    ros::Publisher SmogPublisher;
    ros::Publisher PM25Publisher;
    ros::Publisher TemperaturePublisher;
    // --------------------------------------------

    void subscribeTempSensorIP(const std_msgs::String& TempSensorIPMsg);


};
#endif // my_namespace__my_plugin_H
