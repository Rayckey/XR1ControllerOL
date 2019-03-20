#ifndef XR1CONTROLLERSTATES_H
#define XR1CONTROLLERSTATES_H


#include "Eigen/Dense"

#include <map>
#include <vector>
#include <deque>
#include <list>
#include "Eigen/Geometry"
#include "xr1define.h"
#include "xr1controllerutil.h"
#include "xr1controllerpm.h"





class XR1ControllerStates
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    XR1ControllerStates(int num_of_joints);


    // Ports for animation library
    void setState(std::vector<double> goal_configuration , int period_in_ms, int control_rate = 200);
    void setState(int joint_id , double goal_position , int period_in_ms , int control_rate = 200);
    bool isStateActive();
    bool isStateActive(int joint_id);
    std::vector<double> getNextState();
    double getNextState(int joint_id);
    VectorXd trackBothHands();
    void clearState();



private:


    // global configs
    std::map<uint8_t, uint8_t> ControlModes;


    //Private function called internally
    void getState();
    void calculateStates();
    void assignState();
    void readParameters(string parameters_path);

    //private function all calculating states
    void tinyTriPos(double &value, double & qmin , double &pt_s, double &pt_e);
    void tinyTriVel(double &value, double & qmin );
    void tinyTriAcc(double &value, double & qmin );

    // private members for calcualting states
    uint8_t XR1_State;
    std::vector<double> Qmin;
    std::vector<std::vector<double> > GeneratedCommands;
    int PlaybackIndex;
    std::vector<double> start_state;
    std::vector<double> goal_state;
    std::deque<std::vector<double> > tri_states;
    std::deque<std::vector<double> > tri_vels;
    std::deque<std::vector<double> > tri_accs;
    std::vector<double> temp_state;
    std::vector<double> ready_state;
    int poly_period_ms;
    double poly_period_s;
    int poly_rate;
    int poly_index;
    int poly_num;
    double poly_double;
    double grip_current;

};

#endif // XR1CONTROLLERSTATES_H
