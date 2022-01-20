/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 */

#include <fstream>
#include <map>

#include "sot/torque_control/device-torque-ctrl.hh"
#include <pinocchio/algorithm/joint-configuration.hpp>  // integrate
#include <tsid/math/constraint-base.hpp>
#include <tsid/math/utils.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/debug.hh>

using namespace std;
using namespace dynamicgraph;
using namespace sot::torque_control;
using namespace tsid;
using namespace tsid::tasks;
using namespace dynamicgraph::sot;

typedef tsid::math::ConstraintBase ConstraintBase;

const double DeviceTorqueCtrl::TIMESTEP_DEFAULT = 0.001;
const double DeviceTorqueCtrl::FORCE_SENSOR_NOISE_STD_DEV = 1e-10;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DeviceTorqueCtrl, "DeviceTorqueCtrl");

DeviceTorqueCtrl::DeviceTorqueCtrl(std::string RobotName)
    : dgsot::Device(RobotName),
      timestep_(TIMESTEP_DEFAULT),
      m_initSucceeded(false),
      accelerometerSOUT_("DeviceTorqueCtrl(" + RobotName + ")::output(vector)::accelerometer"),
      gyrometerSOUT_("DeviceTorqueCtrl(" + RobotName + ")::output(vector)::gyrometer"),
      robotStateSOUT_("DeviceTorqueCtrl(" + RobotName + ")::output(vector)::robotState"),
      jointsVelocitiesSOUT_("DeviceTorqueCtrl(" + RobotName + ")::output(vector)::jointsVelocities"),
      jointsAccelerationsSOUT_("DeviceTorqueCtrl(" + RobotName + ")::output(vector)::jointsAccelerations"),
      currentSOUT_("DeviceTorqueCtrl(" + RobotName + ")::output(vector)::currents"),
      p_gainsSOUT_("DeviceTorqueCtrl(" + RobotName + ")::output(vector)::p_gains"),
      d_gainsSOUT_("DeviceTorqueCtrl(" + RobotName + ")::output(vector)::d_gains"),
      CONSTRUCT_SIGNAL_IN(kp_constraints, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_constraints, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rotor_inertias, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(gear_ratios, dynamicgraph::Vector),
      accelerometer_(3),
      gyrometer_(3),
      m_isTorqueControlled(true),
      m_numericalDamping(1e-8),
      normalDistribution_(0.0, FORCE_SENSOR_NOISE_STD_DEV),
      normalRandomNumberGenerator_(randomNumberGenerator_, normalDistribution_) {
  forcesSIN_[0] = new SignalPtr<dynamicgraph::Vector, int>(NULL, "DeviceTorqueCtrl::input(vector6)::inputForceRLEG");
  forcesSIN_[1] = new SignalPtr<dynamicgraph::Vector, int>(NULL, "DeviceTorqueCtrl::input(vector6)::inputForceLLEG");
  forcesSIN_[2] = new SignalPtr<dynamicgraph::Vector, int>(NULL, "DeviceTorqueCtrl::input(vector6)::inputForceRARM");
  forcesSIN_[3] = new SignalPtr<dynamicgraph::Vector, int>(NULL, "DeviceTorqueCtrl::input(vector6)::inputForceLARM");
  signalRegistration(accelerometerSOUT_ << gyrometerSOUT_ << robotStateSOUT_ << jointsVelocitiesSOUT_
                                        << jointsAccelerationsSOUT_ << *(forcesSIN_[0]) << *(forcesSIN_[1])
                                        << *(forcesSIN_[2]) << *(forcesSIN_[3]) << currentSOUT_ << p_gainsSOUT_
                                        << d_gainsSOUT_ << m_kp_constraintsSIN << m_kd_constraintsSIN
                                        << m_rotor_inertiasSIN << m_gear_ratiosSIN);

  // set controlInputType to acceleration so that at the end of the increment
  // method the velocity is copied in the velocity output signal (and not the control)
  controlInputType_ = CONTROL_INPUT_TWO_INTEGRATION;

  // initialize gyrometer and accelerometer
  dynamicgraph::Vector data(3);
  data.setZero();
  gyrometerSOUT_.setConstant(data);
  data(2) = 9.81;
  accelerometerSOUT_.setConstant(data);

  // initialize force/torque sensors
  for (int i = 0; i < 4; ++i) {
    withForceSignals[i] = true;
    wrenches_[i].resize(6);
    wrenches_[i].setZero();
    if (i == FORCE_SIGNAL_RLEG || i == FORCE_SIGNAL_LLEG) wrenches_[i](2) = 350.0;
    forcesSOUT[i]->setConstant(wrenches_[i]);
  }

  temp6_.resize(6);
  m_nk = 12;

  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command increment. */
  docstring =
      "\n    Integrate dynamics for time step provided as input\n"
      "\n      take one floating point number as input\n\n";
  addCommand("increment", makeCommandVoid1((Device&)*this, &Device::increment, docstring));
  addCommand("init", makeCommandVoid2(*this, &DeviceTorqueCtrl::init,
                                      docCommandVoid2("Initialize the entity.", "Time period in seconds (double)",
                                                      "Robot reference (string)")));
}

DeviceTorqueCtrl::~DeviceTorqueCtrl() {}

void DeviceTorqueCtrl::init(const double& dt, const std::string& robotRef) {
  if (dt <= 0.0) return SEND_MSG("Init failed: Timestep must be positive", MSG_TYPE_ERROR);

  /* Retrieve m_robot_util  informations */
  std::string localName(robotRef);
  if (isNameInRobotUtil(localName))
    m_robot_util = getRobotUtil(localName);
  else {
    SEND_MSG("You should first initialize the entity control-manager", MSG_TYPE_ERROR);
    return;
  }

  m_nj = static_cast<int>(m_robot_util->m_nbJoints);

  const dynamicgraph::sot::Vector6d& kp_contact = m_kp_constraintsSIN(0);
  const dynamicgraph::sot::Vector6d& kd_contact = m_kd_constraintsSIN(0);
  const Vector rotor_inertias = m_rotor_inertiasSIN(0);
  const Vector gear_ratios = m_gear_ratiosSIN(0);
  const std::string& urdfFile = m_robot_util->m_urdf_filename;

  try {
    vector<string> package_dirs;
    m_robot = new robots::RobotWrapper(urdfFile, package_dirs, pinocchio::JointModelFreeFlyer());
    m_data = new pinocchio::Data(m_robot->model());
    m_robot->rotor_inertias(rotor_inertias);
    m_robot->gear_ratios(gear_ratios);

    assert(m_robot->nq() == m_nj + 7);
    assert(m_robot->nv() == m_nj + 6);

    m_q.setZero(m_robot->nq());
    m_v.setZero(m_robot->nv());
    m_q_sot.setZero(m_nj + 6);
    m_v_sot.setZero(m_nj + 6);
    m_dv_sot.setZero(m_nj + 6);
    m_dvBar.setZero(m_nj + 6);
    m_tau_np6.setZero(m_nj + 6);
    m_K.setZero(m_robot->nv() + m_nk, m_robot->nv() + m_nk);
    m_k.setZero(m_robot->nv() + m_nk);
    m_f.setZero(m_nk);
    m_Jc.setZero(m_nk, m_robot->nv());
    m_dJcv.setZero(m_nk);

    m_contactRF = new TaskSE3Equality("contact_rfoot", *m_robot, m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
    m_contactRF->Kp(kp_contact);
    m_contactRF->Kd(kd_contact);

    m_contactLF = new TaskSE3Equality("contact_lfoot", *m_robot, m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);
    m_contactLF->Kp(kp_contact);
    m_contactLF->Kd(kd_contact);
  } catch (const std::exception& e) {
    std::cout << e.what();
    return SEND_MSG("Init failed: Could load URDF :" + urdfFile, MSG_TYPE_ERROR);
  }
  timestep_ = dt;
  setStateSize(m_nj + 6);
  m_initSucceeded = true;
}

void DeviceTorqueCtrl::setStateSize(const unsigned int& size) {
  assert(size == static_cast<unsigned int>(m_nj + 6));
  Device::setStateSize(size);

  base6d_encoders_.resize(size);
  base6d_encoders_.fill(0.0);
  jointsVelocities_.resize(size - 6);
  jointsVelocities_.fill(0.0);
  jointsAccelerations_ = jointsVelocities_;

  robotStateSOUT_.setConstant(base6d_encoders_);
  jointsVelocitiesSOUT_.setConstant(jointsVelocities_);
  jointsAccelerationsSOUT_.setConstant(jointsAccelerations_);
}

void DeviceTorqueCtrl::setState(const dynamicgraph::Vector& q) {
  assert(q.size() == m_nj + 6);
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot setState before initialization!");
    return;
  }
  Device::setState(q);
  m_q_sot = q;
  m_robot_util->config_sot_to_urdf(m_q_sot, m_q);

  m_robot->computeAllTerms(*m_data, m_q, m_v);
  pinocchio::SE3 H_lf =
      m_robot->position(*m_data, m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
  tsid::trajectories::TrajectorySample s(12, 6);
  Vector vec_H_lf;
  tsid::math::SE3ToVector(H_lf, vec_H_lf);
  s.setValue(vec_H_lf);
  m_contactLF->setReference(s);
  SEND_MSG("Setting left foot reference to " + toString(H_lf), MSG_TYPE_DEBUG);

  pinocchio::SE3 H_rf =
      m_robot->position(*m_data, m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
  Vector vec_H_rf;
  tsid::math::SE3ToVector(H_rf, vec_H_rf);
  s.setValue(vec_H_rf);
  m_contactRF->setReference(s);
  SEND_MSG("Setting right foot reference to " + toString(H_rf), MSG_TYPE_DEBUG);
  setVelocity(m_v_sot);
}

void DeviceTorqueCtrl::setVelocity(const dynamicgraph::Vector& v) {
  assert(v.size() == m_nj + 6);
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot setVelocity before initialization!");
    return;
  }
  Device::setVelocity(v);
  m_v_sot = v;
  m_robot_util->velocity_sot_to_urdf(m_q, m_v_sot, m_v);
}

void DeviceTorqueCtrl::setControlInputType(const std::string& cit) {
  if (cit == "torque") {
    m_isTorqueControlled = true;
    return;
  }
  m_isTorqueControlled = false;
  return Device::setControlInputType(cit);
}

void DeviceTorqueCtrl::computeForwardDynamics() {
  const Vector& tauDes = controlSIN.accessCopy();
  assert(tauDes.size() == m_nj);

  // compute mass matrix
  m_robot->computeAllTerms(*m_data, m_q, m_v);

  const ConstraintBase& constrRF = m_contactRF->compute(0.0, m_q, m_v, *m_data);
  const ConstraintBase& constrLF = m_contactLF->compute(0.0, m_q, m_v, *m_data);
  assert(constrRF.matrix().rows() == 6 && constrRF.matrix().cols() == m_nj + 6);
  assert(constrLF.matrix().rows() == 6 && constrLF.matrix().cols() == m_nj + 6);
  assert(constrRF.vector().size() == 6 && constrLF.vector().size() == 6);
  assert(m_Jc.rows() == 12 && m_Jc.cols() == m_nj + 6);
  assert(m_K.rows() == m_nj + 6 + 12 && m_K.cols() == m_nj + 6 + 12);
  m_Jc.topRows<6>() = constrRF.matrix();
  m_Jc.bottomRows<6>() = constrLF.matrix();
  Matrix JcT = m_Jc.transpose();
  m_dJcv.head<6>() = constrRF.vector();
  m_dJcv.tail<6>() = constrLF.vector();

  // compute constraint solution: ddqBar = - Jc^+ * dJc * dq
  m_Jc_svd.compute(m_Jc, Eigen::ComputeThinU | Eigen::ComputeFullV);
  tsid::math::solveWithDampingFromSvd(m_Jc_svd, m_dJcv, m_dvBar, m_numericalDamping);

  // compute base of null space of constraint Jacobian
  Eigen::VectorXd::Index r = (m_Jc_svd.singularValues().array() > 1e-8).count();
  m_Z = m_Jc_svd.matrixV().rightCols(m_nj + 6 - r);

  // compute constrained accelerations ddq_c = (Z^T*M*Z)^{-1}*Z^T*(S^T*tau - h - M*ddqBar)
  const Matrix& M = m_robot->mass(*m_data);
  const Vector& h = m_robot->nonLinearEffects(*m_data);
  m_ZMZ = m_Z.transpose() * M * m_Z;
  m_robot_util->joints_sot_to_urdf(tauDes, m_tau_np6.tail(m_nj));
  m_dv_c = m_Z.transpose() * (m_tau_np6 - h - M * m_dvBar);
  Vector rhs = m_dv_c;
  //  m_ZMZ_chol.compute(m_ZMZ);
  //  m_ZMZ_chol.solveInPlace(m_dv_c);
  tsid::math::svdSolveWithDamping(m_ZMZ, rhs, m_dv_c, m_numericalDamping);

  if ((m_ZMZ * m_dv_c - rhs).norm() > 1e-10)
    SEND_MSG("ZMZ*dv - ZT*(tau-h-Mdv_bar) = " + toString((m_ZMZ * m_dv_c - rhs).norm()), MSG_TYPE_DEBUG);

  // compute joint accelerations
  m_dv = m_dvBar + m_Z * m_dv_c;

  // compute contact forces
  Vector b = M * m_dv + h - m_tau_np6;
  //  const double d2 = m_numericalDamping*m_numericalDamping;
  //  const unsigned int nzsv = m_Jc_svd.nonzeroSingularValues();
  //  Eigen::VectorXd tmp(m_Jc_svd.rows());
  //  tmp.noalias() = m_Jc_svd.matrixV().leftCols(nzsv).adjoint() * b;
  //  double sv;
  //  for(int i=0; i<nzsv; i++)
  //  {
  //    sv = m_Jc_svd.singularValues()(i);
  //    tmp(i) *= sv/(sv*sv + d2);
  //  }
  //  m_f = m_Jc_svd.matrixU().leftCols(nzsv) * tmp;
  tsid::math::svdSolveWithDamping(JcT, b, m_f, m_numericalDamping);

  //  SEND_MSG("dv = "+toString(m_dv.norm()), MSG_TYPE_DEBUG);
  //  SEND_MSG("f = "+toString(m_f), MSG_TYPE_DEBUG);
  if ((JcT * m_f - b).norm() > 1e-10)
    SEND_MSG("M*dv +h - JT*f - tau = " + toString((JcT * m_f - b).norm()), MSG_TYPE_DEBUG);
  if ((m_Jc * m_dv - m_dJcv).norm() > 1e-10)
    SEND_MSG("Jc*dv - dJc*v = " + toString((m_Jc * m_dv - m_dJcv).norm()), MSG_TYPE_DEBUG);
}

void DeviceTorqueCtrl::integrate(const double& dt) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot integrate before initialization!");
    return;
  }

  if (m_isTorqueControlled) {
    computeForwardDynamics();
    // integrate
    m_q = pinocchio::integrate(m_robot->model(), m_q, dt * m_v);
    m_v += dt * m_dv;

    m_robot_util->config_urdf_to_sot(m_q, m_q_sot);
    m_robot_util->velocity_urdf_to_sot(m_q, m_v, m_v_sot);
    m_robot_util->velocity_urdf_to_sot(m_q, m_dv, m_dv_sot);

    state_ = m_q_sot;
    velocity_ = m_v_sot;
    jointsAccelerations_ = m_dv_sot.tail(m_nj);
  } else {
    Device::integrate(dt);
    if (controlInputType_ == CONTROL_INPUT_TWO_INTEGRATION)
      jointsAccelerations_ = controlSIN.accessCopy().tail(m_nj);
    else
      jointsAccelerations_.setZero(m_nj);
  }

  base6d_encoders_ = state_;
  jointsVelocities_ = velocity_.tail(m_nj);

  // set the value for the encoders, joints velocities and accelerations output signal
  robotStateSOUT_.setConstant(base6d_encoders_);
  jointsVelocitiesSOUT_.setConstant(jointsVelocities_);
  jointsAccelerationsSOUT_.setConstant(jointsAccelerations_);

  int time = robotStateSOUT_.getTime() + 1;
  for (int i = 0; i < 4; i++) {
    // read input force (if any)
    if (forcesSIN_[i]->isPlugged() && forcesSIN_[i]->operator()(time).size() == 6)
      wrenches_[i] = forcesSIN_[i]->accessCopy();
    // add random noise
    for (int j = 0; j < 6; j++) temp6_(j) = wrenches_[i](j) + normalRandomNumberGenerator_();
    // set output force
    forcesSOUT[i]->setConstant(temp6_);
  }

  robotStateSOUT_.setTime(time);
  jointsVelocitiesSOUT_.setTime(time);
  jointsAccelerationsSOUT_.setTime(time);
  accelerometerSOUT_.setTime(time);
  gyrometerSOUT_.setTime(time);
  for (int i = 0; i < 4; ++i) forcesSOUT[i]->setTime(time);
}
