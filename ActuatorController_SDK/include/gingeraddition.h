#ifndef GINGERADDITION_H
#define GINGERADDITION_H
#include <stdio.h>
#include <vector>
struct BatteryStatus{
    double voltage_;//V
    double current_;//A
    double dump_energy_;//persent
    double init_capacity_;//mAh
    double current_capacity_;//mAh
    uint32_t loop_count_;
    uint32_t status_;//1:charging,0:not charging
    uint8_t id_;
    uint8_t cell_type_;//0:锂 1:磷酸铁
    uint16_t update_counter_;
    bool valid_;//
};

enum SensorType{
    ULTRASONIC,
    PM,
    SMOG,
    DROPCOLLISION,
    TEMPERATURE,
};

//超声波
struct Ultrasonic{
    uint16_t distance;//距离
    bool isValid;//是否有效
};
//跌落碰撞
struct DropCollision{
    bool isDrop;
    bool isCollision;
};

enum Circuit_Channel{
    CH_NONE=-1,
    CH_5V=0,
    CH_12V_1,
    CH_12V_2,
    CH_VBAT_1,
    CH_VBAT_2,
    CH_VBAT_3,
    CH_VBAT_4,

};

#endif // GINGERADDITION_H
