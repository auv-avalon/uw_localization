#ifndef UW_LOCALIZATION_UW_MOTION_MODEL_HPP
#define UW_LOCALIZATION_UW_MOTION_MODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <base/samples/rigid_body_state.h>
#include <base/actuators/commands.h>
#include <base/actuators/status.h>

namespace uw_localization {

typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3d;
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> Vector4d;
typedef Eigen::Matrix<double, 6, 1, Eigen::DontAlign> Vector6d;
typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Matrix3d;
typedef Eigen::Matrix<double, 6, 6, Eigen::DontAlign> Matrix6d;
typedef Eigen::Matrix<double, 12, 1, Eigen::DontAlign> Vector12d;
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> Theta4d;


const double kWaterDensity = 998.2;
const double kGravity = 9.81; 

struct UwVehicleParameter {
    double Length;
    double Radius;
    double Mass;
    Matrix3d InertiaTensor;

    Vector6d ThrusterCoefficient;
    Matrix6d TCM;
    double ThrusterVoltage;

    Theta4d DampingX;
    Theta4d DampingY;
    Theta4d DampingZ;

    bool floating;
};


class UwMotionModel {
 public:
    UwMotionModel(const UwVehicleParameter& params);
    ~UwMotionModel() {}

     // x_t: u v w p q r x y z rot(x) rot(y) rot(z)
    const Vector12d& transition(const Vector12d& x_t, double t, const base::actuators::Status& status);
    /*
    void  updateOrientation(const base::samples::RigidBodyState& orientation);
    */

 protected:
    virtual const Vector6d& GravityBuoyancy(const Vector3d& euler) const;
    virtual const Matrix6d& HydroDamping(const Vector6d& velocity) const;

    const Vector12d& DERIV(const Vector12d& Xt, const Vector6d& Ft);

    const Vector12d& runga_kutta(const Vector12d& Xt, double t, const Vector6d& Ft);


 private:
     UwVehicleParameter parameter;

     Matrix6d MassMatrix;
};

}

#endif
