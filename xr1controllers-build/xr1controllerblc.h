#ifndef XR1CONTROLLERBLC_H
#define XR1CONTROLLERBLC_H


#include <map>
#include <vector>
#include <string>
#include <list>
#include "xr1define.h"
#include "xr1controllerutil.h"



class XR1ControllerBLC
{

public:
    XR1ControllerBLC(std::string blc_library_path, std::string alp_library_path);

    // set unfiltered velocities
    void setTargetOmniCmd(Vector3d & joy_cmd);


    // get next state immediately
    std::vector<double> getNextState();


    // clear all the states , return to initial state, which will force robot to get into idle
    void clearAll();


    //Tilt Control----------------------------------------------------------------

    void tiltCallback(double w , double x , double y , double z);

    void tiltInit();

    // ---------------------------------------------------------------------------

    double getTargetBalanceAngle(uint8_t joint_id);

private:

    // internal functions
    void readLibraries(std::string blc_library_path, std::string alp_library_path);


    std::vector<double> readLibrary(std::string library_path , uint8_t col_num);

    std::vector<std::vector<double> > readIdle(std::string library_path);

    void assignIdleState();

    void assignActiveState();

    void assignBalanceState();

    void assignRotationState();

    void assignFowardBackwardState();

    void assignLeftRightState();

    void TiltCompensation();


    // time related varibles
    int blc_library_period;

    int transition_period;

    double period_s;

    int period_ms;



    // buffers for animation files
    std::vector<std::vector<double> > idle_states;

    std::vector<double> target_state;

    std::vector<double> foward_state;

    std::vector<double> backward_state;

    std::vector<double> leftward_state;

    std::vector<double> rightward_state;

    std::vector<double> leftrot_state;

    std::vector<double> rightrot_state;

    std::vector<double> temp_vel;

    double temp_;



    // varibles to determine different sizes
    int idle_counter;

    int ANIMMATION_ERROR_NUM;



    // Drive Mode speciaty -----------------------------------

    // clear the omni wheels commands, used internally
    void clearOmniCmd();
    double absoluteOmni(Vector3d & input);
    Vector3d temp_omni;
    Vector3d old_omni;
    Vector3d state_omni;

    double max_linear_speed;
    double max_angular_speed;
    // -------------------------------------------------------



    // Balance Speciaty --------------------------------------

        // tilt control members;
        Quaterniond rawQua;
        Quaterniond initQua;
        Matrix3d rawTilt;
        Matrix3d initTilt;
        Matrix3d currTilt;
        Matrix3d tempTilt;
        Vector3d actuAcc;
        Vector3d actuEul;
        Vector3d hatAcc;
        Vector3d innAcc;
        Vector3d noiAcc;
        Vector3d no2Acc;
        Vector3d kvcAcc;
        Vector3d ganAcc;
        Vector3d cvcAcc;
        void accKalman(double x , double y , double z);
        void TiltCalcualtion(Matrix3d & rotation_of_acc);
        void assignAcc2Joint();

    // -------------------------------------------------------

};

#endif // XR1CONTROLLERBLC_H
