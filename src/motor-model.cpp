/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 */

#include <cassert>
#include <cmath>
#include <sot/torque_control/motor-model.hh>
/// i = Kt*tau + Kv*dq + Ka*ddq + sign(dq)Kf
/// tau = i/Kt - (Kv/Kt)*dq - (Ka/Kt)*ddq - sign(dq)(Kf/Kt)
namespace dynamicgraph {
namespace sot {
namespace torque_control {
MotorModel::MotorModel() {}

double MotorModel::getCurrent(double torque, double dq, double ddq, double Kt_p,
                              double Kt_n, double Kf_p, double Kf_n,
                              double Kv_p, double Kv_n, double Ka_p,
                              double Ka_n, unsigned int poly) {
  assert(Kt_p > 0.0 && "Kt_p should be > 0");
  assert(Kt_n > 0.0 && "Kt_n should be > 0");
  assert(Kf_p >= 0.0 && "Kf_p should be >= 0");
  assert(Kf_n >= 0.0 && "Kf_n should be >= 0");
  assert(Kv_p >= 0.0 && "Kv_p should be >= 0");
  assert(Kv_n >= 0.0 && "Kv_n should be >= 0");
  assert(Ka_p >= 0.0 && "Ka_p should be >= 0");
  assert(Ka_n >= 0.0 && "Ka_n should be >= 0");

  double signDq = this->smoothSign(dq, 0.1, poly);  // in [-1;1]
  double current;

  // Smoothly set Coefficients according to velocity sign
  double Kt = 0.5 * (Kt_p * (1 + signDq) + Kt_n * (1 - signDq));
  double Kv = 0.5 * (Kv_p * (1 + signDq) + Kv_n * (1 - signDq));
  double Ka = 0.5 * (Ka_p * (1 + signDq) + Ka_n * (1 - signDq));
  double Kf = 0.5 * (Kf_p * (1 + signDq) + Kf_n * (1 - signDq));

  current = Kt * torque + Kv * dq + Ka * ddq + signDq * Kf;

  return current;
}

double MotorModel::getTorque(double current, double dq, double ddq, double Kt_p,
                             double Kt_n, double Kf_p, double Kf_n, double Kv_p,
                             double Kv_n, double Ka_p, double Ka_n,
                             unsigned int poly) {
  assert(Kt_p > 0.0 && "Kt_p should be > 0");
  assert(Kt_n > 0.0 && "Kt_n should be > 0");
  assert(Kf_p >= 0.0 && "Kf_p should be >= 0");
  assert(Kf_n >= 0.0 && "Kf_n should be >= 0");
  assert(Kv_p >= 0.0 && "Kv_p should be >= 0");
  assert(Kv_n >= 0.0 && "Kv_n should be >= 0");
  assert(Ka_p >= 0.0 && "Ka_p should be >= 0");
  assert(Ka_n >= 0.0 && "Ka_n should be >= 0");

  double signDq = this->smoothSign(dq, 0.1, poly);  // in [-1;1]
  double torque;

  // Smoothly set Coefficients according to velocity sign
  double Kt = 0.5 * (Kt_p * (1 + signDq) + Kt_n * (1 - signDq));
  double Kv = 0.5 * (Kv_p * (1 + signDq) + Kv_n * (1 - signDq));
  double Ka = 0.5 * (Ka_p * (1 + signDq) + Ka_n * (1 - signDq));
  double Kf = 0.5 * (Kf_p * (1 + signDq) + Kf_n * (1 - signDq));

  torque =
      (current / Kt) - (Kv / Kt) * dq - (Ka / Kt) * ddq - signDq * (Kf / Kt);

  return torque;
}

double MotorModel::smoothSign(double value, double threshold,
                              unsigned int poly) {
  if (value > threshold)
    return 1.0;
  else if (value < -threshold)
    return -1.0;
  double a = value / threshold;
  if (poly == 1) return a;
  if (poly == 2 && value > 0) return a * a;
  if (poly == 2 && value <= 0) return -a * a;
  return a * a * a;
}
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph
