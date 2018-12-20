#ifndef XR1IMUMETHODS_H
#define XR1IMUMETHODS_H


#include "Eigen/Dense"
#include "xr1IMUdefine.h"
#include "xr1define.h"
#include <vector>

using namespace Eigen;

class XR1IMUmethods
{
public:
    XR1IMUmethods();

    std::vector<double> getJointAngles();

    void Initialize();

    void quaternioncallback(uint8_t id , double w, double x, double y , double z );

    std::vector<uint8_t> checkModules();



private:



    Vector3d quaternion2ZYX(double w, double qx, double qy , double qz );

    Vector3d quaternion2XYZ(double w, double x, double qy , double qz );

    Matrix3d EulerXYZ(double x , double y , double z ) ;

    Matrix3d EulerZYX(double x , double y , double z ) ;

    Vector3d Matrix2ZYX(Matrix3d input);

    Vector3d Matrix2YZX(Matrix3d input);

    Vector3d Matrix2ZXY(Eigen::Matrix3d input);

    Vector3d Matrix2YXZ(Eigen::Matrix3d input);

    Vector3d Vector2XZ(Vector3d v);

    Vector3d Vector2YX(Vector3d v);

    Vector3d FingerVector2YX(Vector3d v);

    double EasyFilter(double u, double v, double filter_ratio  = 0.5);

    double tinyCurvefit(double double_index , double pt_s , double pt_1 , double pt_2 , double pt_e);

    std::vector<Quaterniond> Init_qs;

    std::vector<Quaterniond> Raw_qs;

    std::vector<Quaterniond> Cooked_qs;

    std::vector<Vector3d> Cooked_vs;

    Matrix3d LH_m;

    Matrix3d RH_m;

    Matrix3d B_m;

    Matrix3d H_m;

    std::vector<double> JointAngles;

    double PI;

    double Finger_Ratios;

    double Thumb_Ratios;

    std::vector<bool> Uncharted_Chart;

    std::vector<uint8_t> Lost_Ids;

};

#endif // XR1IMUMETHODS_H
