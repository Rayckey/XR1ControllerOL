#ifndef XR1CONTROLLERUTIL
#define XR1CONTROLLERUTIL


#include<Eigen/Dense>


using namespace Eigen;

Vector3d Matrix2XY(Matrix3d BaseRotation);

Matrix3d EulerZX(double z , double x);

Matrix3d XY2Matrix(Vector3d BaseRotation);

Vector3d Vector2XY(Vector3d BaseVector);

void EulerXYZ(double x , double y , double z , Matrix3d &input) ;

void EulerZYX(double x , double y , double z , Matrix3d &input) ;

Matrix3d EulerXYZ(double x , double y , double z ) ;

Matrix3d EulerZYX(double x , double y , double z ) ;

Vector3d Matrix2XYZ(Matrix3d BaseRotation);

void Matrix2XYZ(Matrix3d & BaseRotation , Vector3d & ouput);

MatrixXd pinv(MatrixXd input);

MatrixXd twist2trans(VectorXd twist);


double simpleFilter(double new_val , double old_val , double ratios);

VectorXd simpleFilter(VectorXd new_val , VectorXd old_val , double ratios);


double sign(double input);


// say hi to Hatty!
void Hatty_Hattington(Vector3d &x , Matrix3d & input);

void EulerYZX(double y, double z, double x, Matrix3d &rots);

std::vector<double> Evector2Svector(VectorXd & e );

void Svector2Evector(std::vector<double> v , VectorXd & e);

#endif




