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
    double current;
    uint8_t cellsCount;
    std::vector<uint16_t> cellsVoltage;//mV
    uint32_t cellsBalance;
    uint8_t tempCount;
    std::vector<int8_t> cellsTemp;
    uint16_t loopCount;
    double dumpEnergy;
    double totalEnergy;
    uint8_t chargeSwitch;
    enum BtInfo{
        CHARGE_IN=0,
        CHARGE_OVERFLOW,
        DISCHARGE=4,
        DISCHARGE_OVERFLOW,
        DISCHARGE_SHORTCIRCUIT,
        CELL_OPENCIRCUIT=8,
        TEMP_SENSOR_OPENCIRCUIT,
        CELL_OVERVOLTAGE=12,
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

struct Ultrasonic{
    std::vector<int> sonicDistance;
    std::vector<bool> sonicStatus;
};
#endif // GINGERADDITION_H
