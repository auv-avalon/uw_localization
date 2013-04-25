#include "uw_motion_model.hpp"

namespace uw_localization {


UwMotionModel::UwMotionModel(const UwVehicleParameter& params) 
    : parameter(params) 
{
    Matrix3d TensorMass = params.Mass * Matrix3d::Identity();
    Matrix3d InertiaMass = Matrix3d::Zero();
    
    double common = M_PI * kWaterDensity * (parameter.Radius * parameter.Radius) * parameter.Length;
   
    InertiaMass(0,0) = 0.1 * params.Mass;
    InertiaMass(1,1) = common;
    InertiaMass(2,2) = common;
//    TensorMass.block<3, 3>(0, 0) = params.Mass * Matrix3d::Identity();
//    TensorMass.block<3, 3>(3, 3) = params.InertiaTensor;

//    InertiaMass(4,4) = (1.0 / 12.0) * common * (parameter.Length * parameter.Length);
//    InertiaMass(5,5) = (1.0 / 12.0) * common * (parameter.Length * parameter.Length);

    MassMatrix = TensorMass + InertiaMass;
}

const Vector6d& UwMotionModel::transition(const Vector6d& x_t, double t, const base::actuators::Status& status)
{
    Eigen::Array<double, 6, 1> Volt;

    for(unsigned i = 0; i < status.states.size(); i++) {
        Volt(i) = status.states[i].pwm * parameter.ThrusterVoltage;
    }
    
    Vector6d tCoefficient = parameter.ThrusterCoefficient 
			      + parameter.LinearThrusterCoefficient.cwiseProduct(Volt.matrix()) 
			      + parameter.SquareThrusterCoefficient.cwiseProduct((Volt * Volt.abs()).matrix());
    
    Vector6d Ctrl = tCoefficient.cwiseProduct((Volt * Volt.abs()).matrix());
    Vector3d Ft = parameter.TCM.transpose() * Ctrl;

    return runga_kutta(x_t, t, Ft);
}

const Vector6d& UwMotionModel::runga_kutta(const Vector6d& x_t, double t, const Vector3d& f_t)
{
    static Vector6d x_t1;

    Vector3d zero = Vector3d::Zero();

    Vector6d f1 = DERIV(x_t, f_t, zero) * t;
    Vector6d f2 = DERIV(x_t + 0.5 * f1, f_t, zero) * t;
    Vector6d f3 = DERIV(x_t + 0.5 * f2, f_t, zero) * t;
    Vector6d f4 = DERIV(x_t + f3, f_t, zero) * t;

    x_t1 = x_t + (1.0/6.0) * (f1 + 2.0 * f2 + 2.0 * f3 + f4);

    return x_t1;
}

const Vector3d& UwMotionModel::GravityBuoyancy(const Vector3d& euler) const
{
    static Vector3d gravitybuoyancy;

    float uwv_weight = parameter.Mass * kGravity;
    float uwv_buoyancy = kWaterDensity * (M_PI * parameter.Radius * parameter.Radius * parameter.Length) * kGravity;

    float e1 = euler(0);
    float e2 = euler(1);
    //float e3 = euler(2);
 
    // currently its is assumed that the vehicle floats.i.e gravity = buoyancy 
    if (parameter.floating == true)
        uwv_buoyancy = uwv_weight;

    gravitybuoyancy(0) =  (uwv_weight-uwv_buoyancy) * sin(e2);
    gravitybuoyancy(1) = -(uwv_weight-uwv_buoyancy) * (cos(e2)*sin(e1));
    gravitybuoyancy(2) = -(uwv_weight-uwv_buoyancy) * (cos(e2)*cos(e1));
//    gravitybuoyancy(3) = -( ( (yg*uwv_weight)-(yb*uwv_buoyancy) ) * ( cos(e2)*cos(e1) ) ) + ( ( (zg*uwv_weight)-(zb*uwv_buoyancy) ) * (cos(e2)*sin(e1) ) );
//    gravitybuoyancy(4) =  ( ( (zg*uwv_weight)-(zb*uwv_buoyancy) ) *   sin(e2) ) + ( ( (xg*uwv_weight)-(xb*uwv_buoyancy) ) *( cos(e2)*cos(e1) ) );
//    gravitybuoyancy(5) =- ( ( (xg*uwv_weight)-(xb*uwv_buoyancy) ) * ( cos(e2)*sin(e1) ) ) - ( ( (yg*uwv_weight)-(yb*uwv_buoyancy) ) *sin(e2) );

    return gravitybuoyancy;
}

/*
void  UwMotionModel::updateOrientation(const base::samples::RigidBodyState& o)
{
    x_t(9) = base::getRoll(o.orientation);
    x_t(10) = base::getPitch(o.orientation);
    x_t(11) = base::getYaw(o.orientation);
    x_t.block<3, 1>(3, 0) = o.angular_velocity;
}
*/

const Vector3d& UwMotionModel::HydroDamping(const Vector3d& velocity) const
{
    static Vector3d D = Vector3d::Zero();

    double v_x = velocity(0);
    double v_y = velocity(1);
    double v_z = velocity(2);

    Vector2d X, Y, Z;
    X << v_x * fabs(v_x), v_x;
    Y << v_y * fabs(v_y), v_y;
    Z << v_z * fabs(v_z), v_z;

    D(0) = parameter.DampingX.transpose() * X;
    D(1) = parameter.DampingY.transpose() * Y;
    D(2) = parameter.DampingZ.transpose() * Z;

    return D;
}

const Vector6d& UwMotionModel::DERIV(const Vector6d& Xt, const Vector3d& Ft, const Vector3d& euler)
{
    static Vector6d Xdott;

    Vector3d velocity = Xt.block<3, 1>(0, 0); 

    Vector3d gravbuoy = GravityBuoyancy(euler);
    Vector3d damping = HydroDamping(velocity);

    Vector3d acceleration = MassMatrix.inverse() * (Ft - damping - gravbuoy);

    Xdott.block<3, 1>(0, 0) = acceleration;
    Xdott.block<3, 1>(3, 0) = velocity;

    return Xdott;    
}

void UwMotionModel::setThrusterVoltage(double voltage){
  parameter.ThrusterVoltage = voltage;
}

}
