#include "IKsolver.h"
//#include<chrono>

#define PI 3.14159265
using namespace std;

IKsolver::IKsolver()
{
	// default Constructor
	la1 = 0.0492;
	la2 = 0.22452;
	la3 = 0.21923;
	p_end << 0.0, 0.4437, 0.0492;
    arm_angle_offset = 20;
	R = Matrix3d::Identity();
}

IKsolver::IKsolver(double a1, double a2, double a3, double arm_angle)
{
	//Overload parameters and construction

    la1 = a1;
    la2 = a2;
    la3 = a3;

    // MUTE EXLUSIVE EDIT--------------------------------
    // Add an extra part on the wrist==================
//    la3 = a3 + 0.1;
    // ================================================

    arm_angle_offset = arm_angle;

	l1 << 0, 0, PI / 2, PI / 2;
    l2 << 0, la1, PI / 2, PI / 2 + arm_angle_offset;
	l3 << la2, 0, PI / 2, PI / 2;
	l4 << 0, 0, -PI / 2, 0;
	l5 << la3, 0, PI / 2, 0;
	l6 << 0, 0, -PI / 2, -PI / 2;
	l7 << 0, 0, 0, 0;
	dh_param = MatrixXd::Zero(7, 4);
	dh_param.row(0) = l1;
	dh_param.row(1) = l2;
	dh_param.row(2) = l3;
	dh_param.row(3) = l4;
	dh_param.row(4) = l5;
	dh_param.row(5) = l6;
	dh_param.row(6) = l7;



	ux << 1, 0, 0;
	uy << 0, 1, 0;
	uz << 0, 0, 1;
	I3.setIdentity(3, 3);
	I4.setIdentity(4, 4);

    theta = VectorXd::Zero(7);
    thetaPM = VectorXd::Zero(7);
    thetaTF = VectorXd::Zero(7);
    p1 = Vector3d::Zero(3);

	cout << "construction of robotic arm completed......" << endl;
	cout << "ready to solve" << endl;

    TransMatrix = MatrixXd::Identity(4,4);
}

bool IKsolver::solve(const Vector3d &p, const Matrix3d &rot, const double & t)
{

    theta.setZero();
    p_end = p;
    R = rot;
    TransMatrix.topLeftCorner(3, 3) = R;
    TransMatrix.topRightCorner(3, 1) = p_end;

//cout << "p " << p << endl;
//cout << "rot " << rot << endl;

//	cout << endl << "Solving......" << endl << endl;
	//time counter starts
//	auto start = chrono::steady_clock::now();


	//get intuitive geometric properties from joints to calculate theta 4e
	//(not real theta 4)
	d_end = (p_end - p1).norm();
	d_13 = sqrt(pow(la1,2) + pow(la2,2));
	theta_4e = acos((pow(d_end, 2) - pow(la3, 2) - pow(d_13, 2)) / -(2 * d_13*la3));

	//get revolute axis and radius from geometric properties and angles
	angle_J2J1End = asin(la3 / d_end*sin(theta_4e));
	angle_J1EndJ2 = PI - angle_J2J1End - theta_4e;
	ratio = 1 / ((tan(angle_J2J1End) / tan(angle_J1EndJ2)) + 1);
	revolute_pt = p1 + (p_end - p1)*ratio;
	radius = sin(angle_J2J1End)*d_13;

	/*create circle on xz plane, and then rotation it with correct axis, then
	 search the point(p_2) on the circle where z is minimized*/
	u_axis_c = (p_end - p1) / (p_end - p1).norm();
	u_c = u_axis_c.cross(uy);
	u_c = u_c / u_c.norm();
	v_c = u_axis_c.cross(u_c);
	p_elbow = revolute_pt + radius*(u_c*sin(t) + v_c*cos(t));
	phi = -acos(u_axis_c.transpose()*uy);

	//Calculate theta(1) and theta(2)
	po << la1, 0, -la2;
	d3 = p_elbow.norm();
	p_elbow_u = p_elbow / d3;

    try {
        theta(0) = atan2(p_elbow(1), p_elbow(0));
        if (isnan(theta(0))) {
            throw "Error! Theta 1 is not a number. Please check whether task frame is within workspace";
        }
    }
    catch (const char* msg) {
        cout << msg << endl;

        return false;
//        abort();
    }

	p_elbow_frame1 = rot_z(theta(0))*po;
	d3_f1 = p_elbow_frame1.norm();
	p_elbow_frame1_u = p_elbow_frame1 / d3_f1;

    try {
        theta(1) = acos(p_elbow_frame1_u.transpose()*p_elbow_u);
        if (isnan(theta(1))) {
            throw "Error! theta 2 is not a number. Please check whether task frame is within workspace";
        }
    }
    catch (const char* msg) {
        cout << msg << endl;
//        abort();
        return false;
    }


	/*Extract first 4 links with calculated theta(1) and theta(2)
	in DH matrix to get coordinate system of J4. The frame is called
	"frame2" */
    T4 = getTransMat(4, theta);
    rot_matrix = T4.topLeftCorner(3, 3);
	T_mat.topLeftCorner(3, 3) = rot_matrix.transpose();
	T_mat.topRightCorner(3, 1) = -rot_matrix.transpose()*p_elbow;
	T_mat.bottomRows(1) = RowVector4d::Zero(1, 4);
	T_mat(3, 3) = 1;
	pe << p_end, 1;
	p_end_frame2 = T_mat*pe;

	//Projection on xy_frame2 to get J3
	Vector3d p_end_proj_xy(p_end_frame2(0), p_end_frame2(1), 0);

    try {
            if (p_end_frame2(1) < 0)
            {
                theta(2) = -acos(p_end_proj_xy.transpose() / p_end_proj_xy.norm()*ux);
            }
            else
            {
                theta(2) = acos(p_end_proj_xy.transpose() / p_end_proj_xy.norm()*ux);
            }
            if (isnan(theta(2))) {
                throw "Error! theta 3 is not a number. Please check whether task frame is within workspace";
            }
        }
        catch (const char* msg) {
            cout << msg << endl;
//            abort();
            return false;
        }

	//Rotate frame again to calculate J4. The new frame is called frame 3

	p_end_frame3 = rot_z(-theta(2))*p_end_frame2.topRows(3);
	Vector3d p_end_proj_xz(p_end_frame3(0), 0, p_end_frame3(2));

    try {
            theta(3) = -acos(p_end_proj_xz.transpose() / p_end_proj_xz.norm()*uz);
            if (isnan(theta(3))) {
                throw "Error! theta 4 is not a number.Please check whether task frame is within workspace";
            }
        }
        catch (const char* msg) {
            cout << msg << endl;
//            abort();
            return false;
        }


    //Calculate end-effector's orientation using cut-off method
    //-------------------------------------------------------------------
////    std::cout << "geting to orientation" << std::endl;
//    D5 = getDransMat(5, queryPM());
////    std::cout << "getDransMat" << std::endl;
//    R5t = D5.topLeftCorner(3, 3).transpose();
////    std::cout << R5t << std::endl;

//    R5e = R5t * R;

////    std::cout << R5e << std::endl;

//    theta(4) = atan2(- R5e(2,0),R5e(0,0));
//    theta(5) = asin(  R5e(1,0));
//    theta(6) = atan2( -R5e(1,2),  R5e(1,1) );
    //-------------------------------------------------------------------


    //Calculate end-effector's orientation using non-cutoff method that matches the MDH
    //-------------------------------------------------------------------
//    std::cout << "geting to orientation" << std::endl;
    D5 = getDransMat(4, queryPM());
//    std::cout << "getDransMat" << std::endl;
    R5t = D5.topLeftCorner(3, 3).transpose();
//    std::cout << R5t << std::endl;

    R5e = R5t * R;

//    std::cout << R5e << std::endl;
    // Take in consideration of the offsets and alphas
    theta(4) = atan2( R5e(1,2),R5e(0,2));
    theta(5) = asin(  R5e(2,2));
    theta(6) = atan2( R5e(2,1),-R5e(2,0) );
    //-------------------------------------------------------------------




	//Compensate for J2 due to arm angle offset
//	theta(1) += arm_angle_offset;

	//time counter ends
//	auto finish = std::chrono::steady_clock::now();
//	auto elapsed = chrono::duration_cast<chrono::microseconds>(finish - start).count();

//	cout << "runtime: " << elapsed << " us" << endl << endl;
//	cout << "theta: " << endl << theta << endl << endl;
//	cout << "elbow position: " << endl << p_elbow << endl << endl;



	//Debugging
//cout << "d_end " << d_end << endl;
//cout << "d_13 " << d_13 << endl;
//cout << "theta_4e " << theta_4e << endl;
//cout << "radius " << radius << endl;
//cout << "ratio " << ratio << endl;
//cout << "revolute_pt " << revolute_pt << endl;
//cout << "p_elbow " << p_elbow << endl;
//cout << "p_elbow_frame1 " << p_elbow_frame1 << endl;
//cout << "p_elbow_frame1_u " << p_elbow_frame1_u << endl;
////cout << "Ti " <<endl<< Ti << endl;
//cout << "p_end_frame2 " << endl << p_end_frame2 << endl;
//cout << "theta " << theta << endl;


//    return theta;
    return true;
}


VectorXd IKsolver::queryPM(){


// this is right arm
//    thetaPM = theta;
//    thetaPM(0) = theta(0) + PI/2.0;
//    thetaPM(1) = -theta(1) - PI/2.0 - 20.0/180.0*PI;
//    thetaPM(2) = theta(2) + PI/2.0;

    // this is left arm
    thetaPM = theta;
    thetaPM(0) = theta(0) - 1.5708;
    thetaPM(1) = theta(1) - PI/2.0 - arm_angle_offset;
    thetaPM(2) = theta(2) - 1.5708;



    return thetaPM;
}


void IKsolver::verify()
{
	cout << "Verifying....... " << endl << endl;
//	theta(1) -= arm_angle_offset;
	V = getTransMat(7, theta);
	pv = V.topRightCorner(3, 1);
	Rv = V.topLeftCorner(3, 3);
	cout << "position verified as: " << endl << pv << endl << endl;
	cout << "rotation verified as: " << endl << Rv << endl << endl;
}

void IKsolver::plot()
{

}

Matrix3d IKsolver::rot_z(double th)
{
	Matrix3d rot_z_mat;
	rot_z_mat << cos(th), -sin(th), 0,
				sin(th), cos(th), 0,
				0, 0, 1;
	return rot_z_mat;
}

Matrix4d IKsolver::getTransMat(int link_id, VectorXd t) {

	MatrixXd dh_temp;
	Matrix4d Ti;
	dh_temp = dh_param;
	dh_temp.rightCols(1) = t;

	d = dh_temp.col(0);
	ad = dh_temp.col(1);
	alpha = dh_temp.col(2);
	offset = dh_temp.col(3);

	ca = alpha.array().cos();
	sa = alpha.array().sin();
	ct = offset.array().cos();
	st = offset.array().sin();

	Ti = I4;
	for (int i = 1; i <= link_id; ++i)
	{
		T << ct(i - 1), -st(i - 1)*ca(i - 1), st(i - 1)*sa(i - 1), ad(i - 1)*ct(i - 1),
			st(i - 1), ct(i - 1)*ca(i - 1), -ct(i - 1)*sa(i - 1), ad(i - 1)*st(i - 1),
			0, sa(i - 1), ca(i - 1), d(i - 1),
			0, 0, 0, 1;
		Ti = Ti*T;
	}

	return Ti;

}

Matrix4d IKsolver::getDransMat(int link_id, VectorXd t){

    Matrix4d Ts = I4;

    d = dh_param.col(0);
    ad = dh_param.col(1);
    alpha = dh_param.col(2);
    offset = dh_param.col(3);
    offset += t;

    ca = alpha.array().cos();
    sa = alpha.array().sin();
    ct = offset.array().cos();
    st = offset.array().sin();


    for (int i = 1; i <= link_id; ++i)
    {
        T << ct(i - 1), -st(i - 1)*ca(i - 1), st(i - 1)*sa(i - 1), ad(i - 1)*ct(i - 1),
            st(i - 1), ct(i - 1)*ca(i - 1), -ct(i - 1)*sa(i - 1), ad(i - 1)*st(i - 1),
            0, sa(i - 1), ca(i - 1), d(i - 1),
            0, 0, 0, 1;
        Ts = Ts*T;
    }

    return Ts;

}

IKsolver::~IKsolver()
{
	cout << "Program ends." << endl << endl;
}





//Matrix4d IKsolver::getEndEffectorTransform( VectorXd & PMAngles){

//    // Replace the wrist z angle
//    double temp_double = PMAngles(4);
//    PMAngles(4) = 0;
//    Matrix4d Ts = getDransMat(5 , PMAngles);

//    // swap it back
//    PMAngles(4) = temp_double;

//    //Wrist rotation
//    Matrix3d wrist_rots;
//    EulerYZX(PMAngles(4) , PMAngles(5) , PMAngles(6) , wrist_rots);

//    Ts.block<3,3>(0,0) = Ts.block<3,3>(0,0) * wrist_rots;

//    return Ts;
//}
