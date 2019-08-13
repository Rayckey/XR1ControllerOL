#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

void ROTZ(double z , Matrix3d & output);

void ROTX(double x , Matrix3d & output);

void zTransform(double angular , double linear, Affine3d & output);

void xTransform(double angular , double linear, Affine3d & output);

void MDHTransform(double gamma,double b, double alpha, double d,double theta,double r , Affine3d & output);

void MDHKinematics(int up_to_joint, VectorXd &gamma, VectorXd &b, VectorXd &alpha, VectorXd &d, VectorXd &theta, VectorXd &r , std::vector<Affine3d> & trans_out, std::vector<Affine3d> &kines_out);

// Solve Body Jacobian matrix
void calcualteBodyJacobians(std::vector<Affine3d> & trans , std::vector<Affine3d> & kines , std::vector<MatrixXd> & Jaco);

// Solve Spatial Jacobian matrix
void calcualteSpatialJacobians(std::vector<Affine3d> & trans , std::vector<Affine3d> & kines ,  std::vector<MatrixXd> & JacoS, std::vector<MatrixXd> &JacoB);


// get adjoint from transformation
void Adjoint(MatrixXd & adj, Affine3d &temp_afn);
MatrixXd invAdjoint(MatrixXd & T);
MatrixXd EFF2BaseForceAdjoint(MatrixXd & T);


void tinyTriPos(double &value, double & qmin , double &pt_s, double &pt_e, int & pi, int & pn, double & pd, double & pps);
void tinyTriVel(double &value, double & qmin , int &pi , int &pn , double &pd , double &pps);
void tinyTriAcc(double &value, double & qmin , int &pi, int &pn, double &pd, double &pps);


