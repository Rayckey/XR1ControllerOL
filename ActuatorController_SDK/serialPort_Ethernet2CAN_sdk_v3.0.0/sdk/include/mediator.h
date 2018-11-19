#ifndef MEDIATOR_H
#define MEDIATOR_H
#include "asio/io_context.hpp"
#include "actuatordata.h"
#include <functional>
#include <string>
#include "CSignal.hpp"
#include "AbstractAutoRecognize.h"
#include "communication.h"
#include "actuatordefine.h"
#include <thread>
#include <mutex>
#include "versionnumber.h"
//#define TEST_DEBUG
#define LOG_DEBUG
//#define NO_HEART_BEAT
//#define WARNING_DEBUG


const double velScale = 6000;
const double curScale = 8.25;


#define mediator  Mediator::getInstance()//get instance of Mediator
#define requestCallback  std::function<void (uint8_t,uint8_t,double)>
#define errorInfoFunc std::function<void (uint8_t,uint16_t,std::string)>

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

class Mediator
{
public:
    static Mediator * getInstance();
    ~Mediator();
    void autoRecognize();//auto recognize motor
    void onCanConnected(uint32_t nCommunicationUnitId);
    void SendRequest(const std::vector<uint8_t> & buf);
    void SendRequest(uint64_t longId,const std::vector<uint8_t> & buf);
    void SendRequest(const std::string& target,const std::vector<uint8_t> & buf);
    void Handshake(uint64_t longId,bool bSuccess);
    void SetCurParam(const uint64_t longId,const double value, const int nProxyId);//set motor param values
    void SetSucceed(const uint64_t longId, const int nProxyId);//
    void SetFailed(const uint64_t longId, const int nProxyId);//设置下位机参数fail
    //void NullChartPointer();

    void reciveMotorInfo(uint32_t communicateUnitId,const uint32_t nDeviceMac, const uint8_t nDeviceId);
    void receiveNoDataProxy(const int nDeviceID);
    void checkServosStatus();//check servos are on or off
    void recognizeFinished(std::multimap<uint32_t,std::pair<uint8_t,uint32_t>> motorsInfo);
    void chartVauleChange(const int nChannelId,double values);//only use by chart
    asio::io_context * ioContext();
#ifdef IMU_ENABLE
    void receiveQuaternion(uint64_t imuId, double w,double x,double y,double z);
    void receiveAcceleration(uint64_t id, double x,double y,double z,int precision);
    void requestQuaternion(uint8_t nCmdId,uint64_t imuId=0);
#endif

    std::string versionString()const;
    int addCommunicationUnit(std::string unitStr,uint32_t unitNumber);
    void setUnitConnectionStatus(uint32_t nUnitId,uint8_t nStatus);
    void initCommunication(int nType);
//public slots:
    void response(uint32_t nUnitId,const std::vector<uint8_t> buf);
    void reconnectDevice(uint64_t longId);
    void errorOccur(uint64_t longId,uint16_t errorId, std::string errorStr);
    void motorAttrChanged(uint64_t longId,uint8_t nAttrId,double value);
    void receivePanelVersion(uint32_t communicationId,uint16_t softVersion,uint16_t hardVersion);
    static uint64_t toLongId(uint32_t communicationId, uint8_t byteId);
    static uint32_t toCommunicationId(uint64_t longId);
    static uint8_t toDeviceId(uint64_t longId);
    static std::string toString(uint64_t longId);


    void requestBatteryStatus(uint64_t longId);
    void readCircuitCurrent(uint64_t longId, uint8_t channelId);
    void receiveCircuitCurrent(uint64_t longId, uint8_t channelId,double value);
    void readCircuitSwitch(uint64_t longId, uint8_t channelId);
    void receiveCircuitSwitch(uint64_t longId, uint8_t channelId,uint8_t value);
    void setCircuitSwitch(uint64_t longId , uint8_t channelId,uint8_t value);
    void receiveBatteryStatus(uint64_t longId, BatteryStatus& status);
    void requestUltrasonic(uint64_t longId);
    void receiveUltrasonicStatus(uint64_t longId, Ultrasonic& status);
protected:
    Mediator();
public:
    void handleIO();
    void runOnce();
private:
    static Mediator *m_pInstance;
private:
    class GC{
    public:
        ~GC()
        {
            if(m_pInstance!=nullptr)
            {
                delete m_pInstance;
                m_pInstance = nullptr;
            }
        }
        static GC gc;
    };
public:
    CSignal<> m_sRecognizeFinished;
    CSignal<uint64_t,uint8_t,double> m_sRequestBack;
    CSignal<uint64_t,uint16_t,std::string> m_sError;
    CSignal<uint64_t,uint8_t,double> m_sMotorAttrChanged;
    CSignal<> m_sNewChartStart;
    CSignal<uint8_t,double> m_sChartValueChange;
    CSignal<uint64_t,BatteryStatus&> m_sBatteryStatus;
    CSignal<uint64_t,Ultrasonic> m_sUltrasonicStatus;
#ifdef IMU_ENABLE
    CSignal<uint64_t,double,double,double,double> m_sQuaternion;
    CSignal<uint64_t,double,double,double,int> m_sAcceleration;
#endif
private:
    VersionNumber * m_pVersionMgr;
    AbstractAutoRecognize * m_pRecognize;
    Communication * m_pCommunication;
    asio::io_context * m_pIoContext;
    std::thread * m_pIoThread;
    bool m_bStop;
    std::mutex m_ioMutex;
};

#endif // MEDIATOR_H
