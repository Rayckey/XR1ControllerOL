
//#pragma once

#include<iostream>
#include<Eigen/Dense>
#include <map>
#include "xr1define.h"
#include "chaincontroller.h"
#include "xr1controllerutil.h"

using namespace Eigen;
// using namespace std;


class IKplanner
{
private:

	// private methods
    void profile_point(std::vector<double> & path_ptr , double p0, double v0 , double pt , double vt , int tid);
    double tri_dist(double pc, double vc , double vt , int tid);
    void getVelocity(uint8_t control_group );
    void getPosition(uint8_t control_group );
    void getTargetPosition(uint8_t control_group , Affine3d & transformation);
    void plotNext(uint8_t control_group);


	// point to the two arms
	ChainController * LA_ptr;
	ChainController * RA_ptr;


	//private varibles
    double period_s; // the sampling period in second


    std::map<uint8_t, ChainController*> ControllerMap;
    std::map<uint8_t, uint8_t> ControlMode;

    std::map<uint8_t, VectorXd> tMap; //initial
    std::map<uint8_t, VectorXd> bMap; //initial body
    std::map<uint8_t, VectorXd> sMap; //initial spatial
    std::map<uint8_t, VectorXd> cMap; //current spatial only has 5 numbers : dist , xyz rot, elbow angle;
    std::map<uint8_t, VectorXd> csMap; //current spatial speed only has 5 numbers : dist , xyz rot, elbow angle;
    std::map<uint8_t, Vector3d> uMap; //unit vector for both arms
    std::map<uint8_t, VectorXd> targetMap; // target spatial
    std::map<uint8_t, double> disMap;
    std::map<uint8_t, bool> ProMap; // switches for both arms

    VectorXd acc;
    VectorXd maxvel;

	// Projiang arrays


	double LA_e; // eh-eh-eh-elbow
	double RA_e;


	// Tempe vales used
	Matrix3d temp_rot;
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

	bool setEndEffectorPosition(uint8_t control_group , Affine3d & transformation);

    void assignNextCommand();

    void setPeriod(double p);



	~IKplanner();
};
