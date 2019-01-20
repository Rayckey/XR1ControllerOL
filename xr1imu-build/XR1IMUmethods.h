#ifndef XR1IMUMETHODS_H
#define XR1IMUMETHODS_H


#include "Eigen/Dense"
#include "xr1IMUdefine.h"
#include "xr1define.h"
#include "Eigen/StdVector"
#include <vector>
#include <map>

using namespace Eigen;

class XR1IMUmethods
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    void Euler2YXZ(double x , double y , double z , Matrix3d & output);

    Vector3d Matrix2ZYX(Matrix3d input);

    Vector3d Matrix2YZX(Matrix3d input);

    Vector3d Matrix2ZXY(Eigen::Matrix3d input);

    Vector3d Matrix2YXZ(Eigen::Matrix3d input);

    Vector3d Vector2XZ(Vector3d v);

    Vector3d Vector2YX(Vector3d v);

    Vector3d FingerVector2YX(Vector3d v);

    double EasyFilter(double u, double v, double filter_ratio  = 0.5);

    double tinyCurvefit(double double_index , double pt_s , double pt_1 , double pt_2 , double pt_e);


    std::map<int , Quaterniond,std::less<int>,
    Eigen::aligned_allocator<std::pair<const int, Eigen::Quaterniond> > > Init_qs;

    std::map<int , Quaterniond,std::less<int>,
    Eigen::aligned_allocator<std::pair<const int, Eigen::Quaterniond> > > Raw_qs;

    std::map<int , Quaterniond,std::less<int>,
    Eigen::aligned_allocator<std::pair<const int, Eigen::Quaterniond> > > Cooked_qs;


//    std::vector<Quaterniond> Init_qs;

//    std::vector<Quaterniond> Raw_qs;

//    std::vector<Quaterniond> Cooked_qs;


    //Temp varibles -----
    Matrix3d pre_rot;

    Matrix3d pre_pre_rot;
    Matrix3d rot;
    Quaterniond temp_q;
    Quaterniond temp_temp_q;
    Quaterniond new_q;

    std::map<int , Vector3d> Cooked_vs;

    Matrix3d LH_m;

    Matrix3d RH_m;

    Matrix3d B_m;

    Matrix3d H_m;

    std::vector<double> JointAngles;

    double Finger_Ratios;

    double Thumb_Ratios;


    Vector3d unit_y;
    Vector3d unit_x;
    Vector3d unit_z;

    std::map<int , bool> Uncharted_Chart;

    std::vector<uint8_t> Lost_Ids;

};

#endif // XR1IMUMETHODS_H
