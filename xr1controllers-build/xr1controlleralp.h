#ifndef XR1CONTROLLERALP_H
#define XR1CONTROLLERALP_H


#include <map>
#include <vector>
#include <list>
#include "xr1define.h"
#include "xr1controller.h"
#include "time.h"
#include "xr1controllerutil.h"
#include "xr1alpdefine.h"




class XR1ControllerALP
{

public:
    XR1ControllerALP(string library_root_path , XR1Controller *pointer_to_xr1controller, int num_of_animation, int num_of_idle, int num_of_mute, int num_of_teach);

    // get next state immediately, also pops the current state
    void getNextState();

    bool isOmniWheelsMoving();

    // set this animation into que
    void setAnimation(int animation_type , int animation_id);

    // set this pose for face tracking or marker tracking
    void setHeadTrackingPosition(Affine3d & target_affine_from_back_y);

    // pop the next animation in queue
    int popAnimation();

    // check the current animation queue
    std::list<int> queryAnimation();

    // clear all the states , return to initial state, which will force robot to get into idle
    void clearAll();

private:

    // important pointer
    XR1Controller * XR1_ptr;

    // the que that decides what plays next
    std::list<int> animation_que;



    // internal functions
    void readLibraries(string library_path);

    int combineID(int animation_type , int animation_id);

    int getAnimationID(int long_id);

    int distinquishAnimationType(int animation_id);


    std::list<std::vector<double> > readLibrary(string library_path , uint8_t animation_type);

    std::vector<std::vector<double> > readIdle(string library_path);

    void setNextState();

    void setAnimationTransition();

    void setIdleTransition();

    void setIdleState();

    void pickIdleAnimation();

    void propagateStates(std::vector<double> next_serving);

    void feedPMinfos();

    void requestPMTarget();

    int distinquishAnimationType(std::vector<double> next_serving);




    // time related varibles
    int animation_library_period;

    int transition_period;

    double period_s;



    // the current state que
    std::list<std::vector<double> >  states_que;



    // buffers for animation files
    std::map<int,std::list<std::vector<double> > > animation_map;

    std::vector<std::vector<double> >  idle_animation;

    std::map<int,std::list<std::vector<double> > > idle_map;

    std::map<int,int> animation_sizes_map;

    std::map<int,int> animation_type_map;

    std::map<int,int> animation_offsets_map;

    std::map<int,std::list<std::vector<double> > >::iterator animation_constrains_iterator;


    // all kinds of recorder states
    std::vector<double> rep_state;

    std::vector<double> actuator_state;

    std::vector<double> temp_state;

    std::vector<double> pri_state;

    std::vector<double> old_state;

    std::vector<double> temp_vel;

    std::vector<double> pri_vel;

    std::vector<double> temp_acc;



    // varibles to determine different sizes
    int idle_counter;

    int animation_type_current;

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
};

#endif // XR1CONTROLLERALP_H
