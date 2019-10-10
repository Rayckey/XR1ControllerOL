#ifndef GINGERADDITION_H
#define GINGERADDITION_H
#include <stdio.h>
#include <vector>
struct BatteryStatus{
    BatteryStatus()
    {
        mainInfo = 0;
        current = 0;
        cellsBalance = 0;
        tempCount = 0;
        loopCount = 0;
        dumpEnergy = 0;
        totalEnergy = 0;
        chargeSwitch = 0;
    }

    uint32_t mainInfo;
    double current;   ---current
    uint8_t cellsCount;
    std::vector<uint16_t> cellsVoltage;//mV ----cellsVoltage
    uint32_t cellsBalance; 
    uint8_t tempCount;
    std::vector<int8_t> cellsTemp;
    uint16_t loopCount;
    double dumpEnergy;  --capcity
    double totalEnergy;  -- capcity
    uint8_t chargeSwitch;
    enum BtInfo{
        CHARGE_IN=0,   ---- charging
        CHARGE_OVERFLOW,
        DISCHARGE=4,   -----discharging
        DISCHARGE_OVERFLOW,
        DISCHARGE_SHORTCIRCUIT,
        CELL_OPENCIRCUIT=8,
        TEMP_SENSOR_OPENCIRCUIT,
        CELL_OVERVOLTAGE=12,  ---
        CELL_UNDERVOLTAGE,
        TOTAL_OVERVOLTAGE,
        TOTAL_UNDERVOLTAGE,
        CELL_CHARGE_OVERHEAT=18,
        CELL_DISCHARGE_OVERHEAT,
        CELL_CHARGE_UNDERHEAT,
        CELL_DISCHARGE_UNDERHEAT,
        CELL_CHARGE_DELTA_OVERHEAT,
        CELL_CHARGE_DELTA_UNDERHEAT
    };
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
