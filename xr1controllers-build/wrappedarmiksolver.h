#ifndef WRAPPEDARMIKSOLVER_H
#define WRAPPEDARMIKSOLVER_H

// Created by Chaojie Feng
// Modified and maintained by Weiqi Wang
// Wrapped with MDH by Weiqi Wang
// Added Reverse solve for elbow angle (t)
// now being used in optimzation
// Version 3.0


//#pragma once

#include<iostream>
#include<Eigen/Dense>


using namespace Eigen;
using namespace std;


class WrappedArmIKSolver
{
private:

	//Major Variables
	int NUM_LINKS;
	double la1;
	double la2;
	double la3;
	double arm_angle_offset;
	RowVector4d l1;
	RowVector4d l2;
	RowVector4d l3;
	RowVector4d l4;
	RowVector4d l5;
	RowVector4d l6;
	RowVector4d l7;
	MatrixXd dh_param;
	MatrixXd mdh_param;
	Vector3d p_end;
	Matrix3d R;
	Matrix4d TransMatrix;
	Affine3d ArmpitTransform;


	//other variable declaration in solve() function
	double angle_J1EndJ2, angle_J2J1End, d_13, d_end;
	double phi, radius, ratio, theta_4e;
	double d3, d3_f1;
    Matrix3d I3, R7t , R4t , R4e;
    Matrix4d I4, T4, T7, T_74, T, T_mat, T7i , D5;
	Vector3d ux, uy, uz;
	Vector3d u_axis_c, u_c, v_c;
	Vector3d p1, p_elbow_frame1, p_elbow_u, p_elbow_frame1_u, p7;
	Vector3d p_end_proj_xz, p_end_proj_xy, p_end_frame4, p_end_frame3;
	Vector3d revolute_pt, po;
	Vector4d pe, p_end_frame2;
	VectorXd d, ad, alpha, offset, ca, sa, ct, st;
	Matrix3d rot_matrix, rot_z1, rot_z2;

	//other variable declaration in verify() function
	Matrix4d V;


	// buffer saves all the transformation
	std::vector<Affine3d> transS;
	std::vector<Affine3d> transR;


    // reverse calculation varibles and functions
    Affine3d T_end;
    Affine3d T_4;
    Matrix2d A_squared;
    MatrixXd A;
    Vector2d elbow_sin_cos;
    void R2S(const VectorXd & joint_angles);
    void S2R(const VectorXd & joint_angles);
    VectorXd thetaS;
    VectorXd thetaR;
    VectorXd thetaTF;


	//private function
    void getTransMat(int, const VectorXd &);


    // temp variables
    Affine3d temp_affine;
    Matrix3d temp_rot;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Matrix3d rotTF;
	Vector3d p_elbow, pv;
	Matrix3d Rv;

    WrappedArmIKSolver(double a1, double a2, double a3, double arm_angle , Affine3d & armpit_transform);

    bool solve(const Affine3d & target_transformation, const double &t);

    double reverseSolve(const VectorXd & joint_angles);

    VectorXd queryPM();

    void queryPM(VectorXd & output_vec);

    ~WrappedArmIKSolver();
};

#endif // IKSOLVER_H
