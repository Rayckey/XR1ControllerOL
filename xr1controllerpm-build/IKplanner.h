
//#pragma once

#include<iostream>
#include<Eigen/Dense>
#include <map>
#include <deque>
#include "xr1define.h"
#include "chaincontroller.h"
#include "xr1controllerutil.h"

using namespace Eigen;
// using namespace std;


class IKplanner
{
private:

	// private methods
//    void profile_point(std::vector<double> & path_ptr , double p0, double v0 , double pt , double vt , int tid);
//    double tri_dist(double pc, double vc , double vt , int tid);
    void getVelocity(uint8_t control_group );
    void getPosition(uint8_t control_group );
    void plotAll(uint8_t control_group);
    void getTargetPosition(uint8_t control_group , const Affine3d & transformation);
    void plotNext(uint8_t control_group);
    double tinyTriPos(double qmin , double pt_s , double pt_e , int poly_index , double poly_double , int poly_num , double poly_period_s);


	// point to the two arms
	ChainController * LA_ptr;
	ChainController * RA_ptr;


	//private varibles



    std::map<uint8_t, ChainController*> ControllerMap;
    std::map<uint8_t, uint8_t> ControlMode;

    std::map<uint8_t, Vector3d> posMap; //current spatial position only
    std::map<uint8_t, Vector3d> csMap; //current spatial speed only
    std::map<uint8_t, VectorXd> sMap;  // Current spatial speed twist
    std::map<uint8_t, Quaterniond, std::less<int>,
    Eigen::aligned_allocator<std::pair<const int, Quaterniond> >> quaMap; // current spatial quaternion only;
    std::map<uint8_t, Quaterniond, std::less<int>,
    Eigen::aligned_allocator<std::pair<const int, Quaterniond> >> targetquaMap; // target spatial quaternion only;
    std::map<uint8_t, Vector3d> uMap; //unit vector for both arms
    std::map<uint8_t, Vector3d> rMap; //unit rotation axis for both arms
    std::map<uint8_t, Vector3d> targetMap; // target spatial position only
    std::map<uint8_t, double> elbMap; // current elbow angle
    std::map<uint8_t, double> targetelbMap; // target elbow angle
    std::map<uint8_t, double> disMap; // linear distance
    std::map<uint8_t, double> drgMap; // angular distance
    std::map<uint8_t, double> edsMap; // elbow distance
    std::map<uint8_t, double> elbAccMap; // elbow accerlation map
    std::map<uint8_t, double> drgAccMap; // rotation accerlation map
    std::map<uint8_t, double> posAccMap; // position accelration map





    // private members for calcualting states
    uint8_t XR1_State;
    std::vector<double> Qmin;

    std::map<uint8_t , std::deque<double > > disQue;
    std::map<uint8_t , std::deque<double > > drgQue;
    std::map<uint8_t , std::deque<double > > elbQue;

    std::map<uint8_t,int> period_ms;
    std::map<uint8_t,double> period_s; // the sampling period in second
    int control_rate;
    std::map<uint8_t,int> step_num;

	// Projiang arrays


	double LA_e; // eh-eh-eh-elbow
	double RA_e;


	// Tempe vales used
	Matrix3d temp_rot;
    Quaterniond temp_qua;
	Vector3d temp_vec;
    Vector3d temp_war;
	Affine3d temp_afn;
    Affine3d temp_T; // spacial transform
    VectorXd temp_t; // spacial twist, v and w are seperated
    VectorXd temp_b; //body velocity
    VectorXd temp_s; //spacial velocity
    MatrixXd temp_adj; // b->a adjoint
    MatrixXd temp_jac; // body jacobians;
    VectorXd temp_target; // target twist





public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	IKplanner(ChainController * point2leftarm , ChainController * point2rightarm);

    bool setEndEffectorPosition(uint8_t control_group , const Affine3d & transformation, double elbow_angle, double period);

    void assignNextCommand();

    void setControlRate(int rate);

    bool getNextState(uint8_t control_group );



	~IKplanner();
};
