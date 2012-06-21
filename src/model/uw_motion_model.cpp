#include "uw_motion_model.hpp"

namespace uw_localization {


UwMotionModel::UwMotionModel(const UwVehicleParameter& params) 
    : parameter(params) 
{
    Matrix6d TensorMass = Matrix6d::Zero();
    Matrix6d InertiaMass = Matrix6d::Zero();
    
    double common = M_PI * kWaterDensity * (parameter.Radius * parameter.Radius) * parameter.Length;
   
    TensorMass.block<3, 3>(0, 0) = params.Mass * Matrix3d::Identity();
    TensorMass.block<3, 3>(3, 3) = params.InertiaTensor;

    InertiaMass(0,0) = 0.1;
    InertiaMass(1,1) = common;
    InertiaMass(2,2) = common;
    InertiaMass(4,4) = (1.0 / 12.0) * common * (parameter.Length * parameter.Length);
    InertiaMass(5,5) = (1.0 / 12.0) * common * (parameter.Length * parameter.Length);

    MassMatrix = TensorMass + InertiaMass;
}

const Vector12d& UwMotionModel::transition(const Vector12d& x_t, double t, const base::actuators::Status& status)
{
    Eigen::Array<double, 6, 1> Volt;

    for(unsigned i = 0; i < status.states.size(); i++) {
        Volt(i) = status.states[i].pwm * parameter.ThrusterVoltage;
    }

    Vector6d Ctrl = parameter.ThrusterCoefficient.cwiseProduct((Volt * Volt.abs()).matrix());
    Vector6d Ft = parameter.TCM.transpose() * Ctrl;

    return runga_kutta(x_t, t, Ft);
}

const Vector12d& UwMotionModel::runga_kutta(const Vector12d& x_t, double t, const Vector6d& f_t)
{
    static Vector12d x_t1;

    Vector12d f1 = DERIV(x_t, f_t) * t;
    Vector12d f2 = DERIV(x_t + 0.5 * f1, f_t) * t;
    Vector12d f3 = DERIV(x_t + 0.5 * f2, f_t) * t;
    Vector12d f4 = DERIV(x_t + f3, f_t) * t;

    x_t1 = x_t + (1.0/6.0) * (f1 + 2.0 * f2 + 2.0 * f3 + f4);

    return x_t1;
}

const Vector6d& UwMotionModel::GravityBuoyancy(const Vector3d& euler) const
{
    static Vector6d gravitybuoyancy;

    float uwv_weight = parameter.Mass * kGravity;
    float uwv_buoyancy = kWaterDensity * (M_PI * parameter.Radius * parameter.Radius * parameter.Length) * kGravity;

    float e1 = euler(0);
    float e2 = euler(1);
    //float e3 = euler(2);
    float xg = 0.0; //param.distance_body2centerofgravity(0);
    float yg = 0.0; //param.distance_body2centerofgravity(1);
    float zg = 0.0; // param.distance_body2centerofgravity(2);
    float xb = 0.0; // param.distance_body2centerofbuoyancy(0);
    float yb = 0.0; //param.distance_body2centerofbuoyancy(1);
    float zb = 0.0; // param.distance_body2centerofbuoyancy(2);

    // currently its is assumed that the vehicle floats.i.e gravity = buoyancy 
    if (parameter.floating == true)
        uwv_buoyancy = uwv_weight;

    gravitybuoyancy(0) =  (uwv_weight-uwv_buoyancy) * sin(e2);
    gravitybuoyancy(1) = -(uwv_weight-uwv_buoyancy) * (cos(e2)*sin(e1));
    gravitybuoyancy(2) = -(uwv_weight-uwv_buoyancy) * (cos(e2)*cos(e1));
    gravitybuoyancy(3) = -( ( (yg*uwv_weight)-(yb*uwv_buoyancy) ) * ( cos(e2)*cos(e1) ) ) + ( ( (zg*uwv_weight)-(zb*uwv_buoyancy) ) * (cos(e2)*sin(e1) ) );
    gravitybuoyancy(4) =  ( ( (zg*uwv_weight)-(zb*uwv_buoyancy) ) *   sin(e2) ) + ( ( (xg*uwv_weight)-(xb*uwv_buoyancy) ) *( cos(e2)*cos(e1) ) );
    gravitybuoyancy(5) =- ( ( (xg*uwv_weight)-(xb*uwv_buoyancy) ) * ( cos(e2)*sin(e1) ) ) - ( ( (yg*uwv_weight)-(yb*uwv_buoyancy) ) *sin(e2) );

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

const Matrix6d& UwMotionModel::HydroDamping(const Vector6d& velocity) const
{
    static Matrix6d D = Matrix6d::Zero();

    double v_x = velocity(0);
    double v_y = velocity(1);
    double v_z = velocity(2);

    Vector4d X, Y, Z;
    X << v_x * v_x * v_x, v_x * v_x, v_x, 1;
    Y << v_y * v_y * v_y, v_y * v_y, v_y, 1;
    Z << v_z * v_z * v_z, v_z * v_z, v_z, 1;

    D(0,0) = parameter.DampingX.transpose() * X;
    D(1,1) = parameter.DampingY.transpose() * Y;
    D(2,2) = parameter.DampingZ.transpose() * Z;

    return D;
}

const Vector12d& UwMotionModel::DERIV(const Vector12d& Xt, const Vector6d& Ft)
{
    static Vector12d Xdott;

    Vector6d velocity = Xt.block<6, 1>(0, 0); 
    Vector3d euler    = Xt.block<3, 1>(9, 0);

    Vector6d gravbuoy = GravityBuoyancy(euler);
    Matrix6d damping = HydroDamping(velocity);

    Vector6d acceleration = MassMatrix.inverse() * (Ft - damping * Vector6d::Constant(1.0) - gravbuoy);

    Xdott.block<6, 1>(0, 0) = acceleration;
    Xdott.block<6, 1>(6, 0) = velocity;

    return Xdott;    
}

}
