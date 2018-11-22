
// Created by Chaojie Feng
// Modified for XR1Controller PM by Weiqi Wang
// Modified to match DH and wrist
// Version 1.0


//#pragma once

#include<iostream>
#include<Eigen/Dense>


using namespace Eigen;
using namespace std;


class IKsolver
{
private:

	//Major Variables
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
	Vector3d p_end;
	Matrix3d R;
	Matrix4d TransMatrix;


	//other variable declaration in solve() function
	double angle_J1EndJ2, angle_J2J1End, d_13, d_end;
	double phi, radius, ratio, theta_4e;
	double d3, d3_f1;
    Matrix3d I3, R7t , R5t , R5e;
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


	//private function
	Matrix3d rot_z(double);    //rotate about z-axis with specified angle
	Matrix4d getTransMat(int, VectorXd);
    Matrix4d getDransMat(int link_id, VectorXd t);

public:
	VectorXd theta;
    VectorXd thetaPM;
    VectorXd thetaTF;
    Matrix3d rotTF;
	Vector3d p_elbow, pv;
	Matrix3d Rv;

	IKsolver();

    IKsolver(double a1, double a2, double a3, double arm_angle);

    bool solve(const Vector3d& p, const Matrix3d& rot, const double &t);

    Matrix4d getEndEffectorTransform( VectorXd & PMAngles);

    Matrix3d EulerYZX(double y, double z, double x, Matrix3d &rots);

    VectorXd queryPM();

	void verify();

	void plot();

	~IKsolver();
};
