#ifndef XR1CONTROLLERALP_H
#define XR1CONTROLLERALP_H


#include <map>
#include <vector>
#include <list>

#include "xr1define.h"
#include "xr1controller.h"
#include "RSignal.hpp"

#include "xr1controllerutil.h"
#include "xr1controllerblc.h"
#include "xr1alpdefine.h"




class XR1ControllerALP
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    XR1ControllerALP(string library_root_path , XR1Controller *pointer_to_xr1controller, XR1ControllerBLC *pointer_to_blc = nullptr);

    // switch between animation mode and drive mode
    void switchAnimationMode(int option , double optional_speed_filter = 0.1 , double optional_angular_speed_filter = 0.1);


    // set unfiltered velocities
    void setTargetOmniCmd(Vector3d & joy_cmd);


    // get next state immediately, also pops the current state
    void getNextState();


    bool isOmniWheelsMoving();

    // set this animation into que
    void setAnimation(int animation_type , int animation_id);

    // set this animation into que
    void overwriteAnimation(int animation_type , int animation_id , std::deque<std::vector<double>> animation_data);


    // set Idle stuff on or off
    void setIdleOption(bool isOn);

    // set Idle stuff on or off
    void setDefaultOption(bool isOn);


    // set this pose for face tracking or marker tracking
    void setHeadTrackingPosition(Affine3d & target_affine_from_back_y);

    // pop the next animation in queue
    int popAnimation();

    // check the current animation queue
    std::deque<int> queryAnimation();

    // check if the animation is in idle
    bool checkProgress ( int&  animation_type ,  int & animation_id , int & animation_progress);

    // Check the options
    void checkOptions(bool & hasIdle , bool & hasDefault);

    // get the propoer ID
    int getAnimationID(int long_id);

    // get the proper animation type
    int distinquishAnimationType(int animation_id);


    //Lengthen the single transition period
    void setSingleTransitionPeriod(int num_of_second);


    // clear all the states , return to initial state, which will force robot to get into idle
    void clearAll();


    // R message for animation finished signal
    RSignal<int ,int , bool> m_sAnimationFinished;


private:

    // important pointer
    XR1Controller * XR1_ptr;
    XR1ControllerBLC * XRB_ptr;

    // the que that decides what plays next
    std::deque<int> animation_que;

    // The option for whether idle animations are played
    bool idle_switch;

    // The option for whether idle animations are played
    bool default_switch;


    std::map<int ,int > control_modes_map;

    // internal functions
    void readLibraries(string library_path);

    int combineID(int animation_type , int animation_id);

    bool distinquishAnimationProcedure();

    void assignModeTransition();

    void judgeAnimationRecovery(int control_group);


    std::deque<std::vector<double> > readLibrary(string library_path, uint8_t animation_type);

    std::vector<std::string> readLibrariesName(string library_path, string prefix_string, string subfix_string);

    int extractLibraryID(const string &file_name, string prefix_string , string subfix_string);

    std::vector<std::vector<double> > readIdle(string library_path);

    bool is_digits(const std::string &str);

    void setNextState();

    void setAnimationTransition();

    void switchAnimationControlType();

    void setIdleTransition();

    void setIdleState();

    void pickIdleAnimation();

    void propagateStates(std::vector<double> next_serving);

    void propagateStates(uint8_t id, double next_serving);

    void feedPMinfos(int mode_fix = XR1::AnimationMode);

    void requestPMTarget();

    int distinquishAnimationType(std::vector<double> next_serving);




    // time related varibles
    int animation_library_period;

    int transition_period;

    int default_transition_period;

    double period_s;

    int period_ms;



    // the current state que
    std::deque<std::vector<double> >  states_que;



    // buffers for animation files
    std::map<int,std::deque<std::vector<double> > > animation_map;

    std::vector<std::vector<double> >  idle_animation;

    std::map<int,std::deque<std::vector<double> > > idle_map;

    std::map<int,int> animation_sizes_map;

    std::map<int,int> animation_type_map;

    std::map<int,int> animation_offsets_map;

    std::map<int,std::deque<std::vector<double> > >::iterator animation_constrains_iterator;

    std::map<int ,bool> animation_switched_signal;


    // all kinds of recorder states
    std::vector<double> rep_state;

    std::vector<double> temp_state;

    std::vector<double> pri_state;

    std::vector<double> old_state;

    std::vector<double> temp_vel;

    std::vector<double> pri_vel;

    std::vector<double> temp_acc;

    double temp_;



    // varibles to determine different sizes
    int idle_counter;

    int animation_type_current;

    int animation_id_current;

    int animation_size_current;

    int NUM_OF_ANIMATION;

    int NUM_OF_IDLE;

    int NUM_OF_MUTE;

    int NUM_OF_TEACH;

    int OFFSET_OF_ANIMATION;

    int OFFSET_OF_IDLE;

    int OFFSET_OF_MUTE;

    int OFFSET_OF_TEACH;

    int OFFSET_SIZE;

    int SIZE_OF_ANIMATION;

    int SIZE_OF_IDLE;

    int SIZE_OF_MUTE;

    int SIZE_OF_TEACH;

    int ANIMMATION_ERROR_NUM;


    // ids flags for the controllers
    std::vector<uint8_t> control_group_flags;
    std::vector<uint8_t> temp_ids;
    std::vector<double> temp_state_commands;


    // Drive Mode speciaty -----------------------------------

    // get filtered velocities
    void employCurrentOmniCmd();

    // clear the omni wheels commands, used internally
    void clearOmniCmd();
    double absoluteOmni(Vector3d & input);
    double omni_angle;
    int omni_old_mode;
    Vector3d temp_omni;
    Vector3d target_omni;
    Vector3d old_omni;
    Vector3d state_omni;
    int omni_acc_counter;
    int omni_max_counter;
    double omni_lnr_filter_val;
    double omni_ang_filter_val;
    // -------------------------------------------------------




};

#endif // XR1CONTROLLERALP_H
