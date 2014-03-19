#ifndef UW_LOCALIZATION_UW_MOTION_MODEL_HPP
#define UW_LOCALIZATION_UW_MOTION_MODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <base/samples/rigid_body_state.h>
#include <base/samples/Joints.hpp>

namespace uw_localization {

typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vector2d;
typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3d;
typedef Eigen::Matrix<double, 6, 1, Eigen::DontAlign> Vector6d;
typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Matrix3d;
typedef Eigen::Matrix<double, 6, 6, Eigen::DontAlign> Matrix6d;
typedef Eigen::Matrix<double, 12, 1, Eigen::DontAlign> Vector12d;
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Theta2d;
typedef Eigen::Matrix<double, 6, 3, Eigen::DontAlign> MatrixTCM;


const double kWaterDensity = 998.2;
const double kGravity = 9.81; 

struct UwVehicleParameter {
    double Length;
    double Radius;
    double Mass;
//    Matrix3d InertiaTensor;

    Vector6d ThrusterCoefficient;
    Vector6d LinearThrusterCoefficient;
    Vector6d SquareThrusterCoefficient;
    MatrixTCM TCM;
    double ThrusterVoltage;

    Theta2d DampingX;
    Theta2d DampingY;
    Theta2d DampingZ;

    bool floating;
};


class UwMotionModel {
 public:
    UwMotionModel();
    ~UwMotionModel() {}
    
    void init(const UwVehicleParameter& params);
     // x_t: u v w x y z
    const Vector6d& transition(const Vector6d& x_t, double t, const base::samples::Joints& status);
    /*
    void  updateOrientation(const base::samples::RigidBodyState& orientation);
    */
    void setThrusterVoltage(double voltage);
    
 protected:
    virtual const Vector3d& GravityBuoyancy(const Vector3d& euler) const;
    virtual const Vector3d& HydroDamping(const Vector3d& velocity) const;

    const Vector6d& DERIV(const Vector6d& Xt, const Vector3d& Ft, const Vector3d& euler);

    const Vector6d& runga_kutta(const Vector6d& Xt, double t, const Vector3d& Ft);


 private:
     UwVehicleParameter parameter;

     Matrix3d MassMatrix;
};

}

#endif
