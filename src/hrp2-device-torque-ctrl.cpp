/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <map>

#include <sot/core/debug.hh>

#include "sot/torque_control/hrp2-device-torque-ctrl.hh"
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <pininvdyn/math/constraint-base.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate

using namespace std;
using namespace dynamicgraph;
using namespace sot::torque_control;
using namespace pininvdyn;
using namespace pininvdyn::tasks;

typedef pininvdyn::math::ConstraintBase ConstraintBase;

const double HRP2DeviceTorqueCtrl::TIMESTEP_DEFAULT = 0.001;
const double HRP2DeviceTorqueCtrl::FORCE_SENSOR_NOISE_STD_DEV = 1e-10;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(HRP2DeviceTorqueCtrl,"HRP2DeviceTorqueCtrl");

HRP2DeviceTorqueCtrl::HRP2DeviceTorqueCtrl(std::string RobotName):
  dgsot::Device(RobotName),
  timestep_(TIMESTEP_DEFAULT),
  m_initSucceeded (false),
  accelerometerSOUT_("HRP2DeviceTorqueCtrl(" + RobotName + ")::output(vector)::accelerometer"),
  gyrometerSOUT_ ("HRP2DeviceTorqueCtrl(" + RobotName + ")::output(vector)::gyrometer"),
  robotStateSOUT_ ("HRP2DeviceTorqueCtrl(" + RobotName + ")::output(vector)::robotState"),
  jointsVelocitiesSOUT_ ("HRP2DeviceTorqueCtrl(" + RobotName + ")::output(vector)::jointsVelocities"),
  jointsAccelerationsSOUT_ ("HRP2DeviceTorqueCtrl(" + RobotName + ")::output(vector)::jointsAccelerations"),
  currentSOUT_ ("HRP2DeviceTorqueCtrl(" + RobotName + ")::output(vector)::currents"),
  p_gainsSOUT_ ("HRP2DeviceTorqueCtrl(" + RobotName + ")::output(vector)::p_gains"),
  d_gainsSOUT_ ("HRP2DeviceTorqueCtrl(" + RobotName + ")::output(vector)::d_gains"),
  accelerometer_ (3),
  gyrometer_ (3),
  normalDistribution_(0.0, FORCE_SENSOR_NOISE_STD_DEV),
  normalRandomNumberGenerator_(randomNumberGenerator_,normalDistribution_),
  CONSTRUCT_SIGNAL_IN(kp_constraints,              ml::Vector),
  CONSTRUCT_SIGNAL_IN(kd_constraints,              ml::Vector),
  CONSTRUCT_SIGNAL_IN(rotor_inertias,              ml::Vector),
  CONSTRUCT_SIGNAL_IN(gear_ratios,                 ml::Vector)
{
  forcesSIN_[0] = new SignalPtr<ml::Vector, int>(NULL, "HRP2DeviceTorqueCtrl::input(vector6)::inputForceRLEG");
  forcesSIN_[1] = new SignalPtr<ml::Vector, int>(NULL, "HRP2DeviceTorqueCtrl::input(vector6)::inputForceLLEG");
  forcesSIN_[2] = new SignalPtr<ml::Vector, int>(NULL, "HRP2DeviceTorqueCtrl::input(vector6)::inputForceRARM");
  forcesSIN_[3] = new SignalPtr<ml::Vector, int>(NULL, "HRP2DeviceTorqueCtrl::input(vector6)::inputForceLARM");
  signalRegistration(accelerometerSOUT_<<gyrometerSOUT_<<
                     robotStateSOUT_<<jointsVelocitiesSOUT_<<jointsAccelerationsSOUT_<<
                     *(forcesSIN_[0])<<*(forcesSIN_[1])<<*(forcesSIN_[2])<<*(forcesSIN_[3])<<
                     currentSOUT_ << p_gainsSOUT_ << d_gainsSOUT_<<
                     m_kp_constraintsSIN << m_kd_constraintsSIN <<
                     m_rotor_inertiasSIN << m_gear_ratiosSIN);

  // set controlInputType to acceleration so that at the end of the increment
  // method the velocity is copied in the velocity output signal (and not the control)
  controlInputType_=CONTROL_INPUT_ACCELERATION;

  // initialize gyrometer and accelerometer
  ml::Vector data(3); data.setZero();
  gyrometerSOUT_.setConstant(data);
  data(2) = 9.81;
  accelerometerSOUT_.setConstant(data);

  // initialize force/torque sensors
  for( int i=0;i<4;++i )
  {
    withForceSignals[i] = true;
    wrenches_[i].resize(6);
    wrenches_[i].setZero();
    if(i==FORCE_SIGNAL_RLEG || i==FORCE_SIGNAL_LLEG)
      wrenches_[i](2) = 350.0;
    forcesSOUT[i]->setConstant(wrenches_[i]);
  }

  temp6_.resize(6);
  m_nk = 12;


  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command increment. */
  docstring = "\n    Integrate dynamics for time step provided as input\n"
              "\n      take one floating point number as input\n\n";
  addCommand("increment", makeCommandVoid1((Device&)*this,
                                           &Device::increment, docstring));
  addCommand("init",
             makeCommandVoid2(*this, &HRP2DeviceTorqueCtrl::init,
                              docCommandVoid2("Initialize the entity.",
                                              "Time period in seconds (double)",
                                              "URDF file path (string)")));
}

HRP2DeviceTorqueCtrl::~HRP2DeviceTorqueCtrl()
{ }

void HRP2DeviceTorqueCtrl::init(const double& dt, const std::string& urdfFile)
{
  if(dt<=0.0)
    return SEND_MSG("Init failed: Timestep must be positive", MSG_TYPE_ERROR);

  EIGEN_CONST_VECTOR_FROM_SIGNAL(kp_contact, m_kp_constraintsSIN(0));
  assert(kp_contact.size()==6);
  EIGEN_CONST_VECTOR_FROM_SIGNAL(kd_contact, m_kd_constraintsSIN(0));
  assert(kd_contact.size()==6);
  EIGEN_CONST_VECTOR_FROM_SIGNAL(rotor_inertias, m_rotor_inertiasSIN(0));
  assert(rotor_inertias.size()==N_JOINTS);
  EIGEN_CONST_VECTOR_FROM_SIGNAL(gear_ratios, m_gear_ratiosSIN(0));
  assert(gear_ratios.size()==N_JOINTS);

  try
  {
    vector<string> package_dirs;
    m_robot = new RobotWrapper(urdfFile, package_dirs, se3::JointModelFreeFlyer());
    m_data = new se3::Data(m_robot->model());
    m_robot->rotor_inertias(rotor_inertias);
    m_robot->gear_ratios(gear_ratios);

    assert(m_robot->nq()==N_JOINTS+7);
    assert(m_robot->nv()==N_JOINTS+6);

    m_q.setZero(m_robot->nq());
    m_v.setZero(m_robot->nv());
    m_q_sot.setZero(N_JOINTS+6);
    m_v_sot.setZero(N_JOINTS+6);
    m_dv_sot.setZero(N_JOINTS+6);
    m_K.setZero(m_robot->nv()+m_nk, m_robot->nv()+m_nk);
    m_k.setZero(m_robot->nv()+m_nk);
    m_Jc.setZero(m_nk, m_robot->nv());

    m_contactRF = new TaskSE3Equality("contact_rfoot", *m_robot, RIGHT_FOOT_FRAME_NAME);
    m_contactRF->Kp(kp_contact);
    m_contactRF->Kd(kd_contact);

    m_contactLF = new TaskSE3Equality("contact_lfoot", *m_robot, LEFT_FOOT_FRAME_NAME);
    m_contactLF->Kp(kp_contact);
    m_contactLF->Kd(kd_contact);
  }
  catch (const std::exception& e)
  {
    std::cout << e.what();
    return SEND_MSG("Init failed: Could load URDF :" + urdfFile, MSG_TYPE_ERROR);
  }
  timestep_ = dt;
  setStateSize(N_JOINTS+6);
  m_initSucceeded = true;
}

void HRP2DeviceTorqueCtrl::setStateSize(const unsigned int& size)
{
  assert(size==N_JOINTS+6);
  Device::setStateSize(size);

  base6d_encoders_.resize(size);
  base6d_encoders_.fill(0.0);
  jointsVelocities_.resize(size-6);
  jointsVelocities_.fill(0.0);
  jointsAccelerations_ = jointsVelocities_;

  robotStateSOUT_.setConstant(base6d_encoders_);
  jointsVelocitiesSOUT_.setConstant(jointsVelocities_);
  jointsAccelerationsSOUT_.setConstant(jointsAccelerations_);
}

void HRP2DeviceTorqueCtrl::setState( const ml::Vector& q )
{
  assert(q.size()==N_JOINTS+6);
  if(!m_initSucceeded)
  {
    SEND_WARNING_STREAM_MSG("Cannot setState before initialization!");
    return;
  }
  Device::setState(q);
  COPY_VECTOR_TO_VECTOR(q, m_q_sot);
  config_sot_to_urdf(m_q_sot, m_q);

  m_robot->computeAllTerms(*m_data, m_q, m_v);
  se3::SE3 H_lf = m_robot->position(*m_data,
                                    m_robot->model().getJointId(LEFT_FOOT_FRAME_NAME));
  pininvdyn::trajectories::TrajectorySample s(12, 6);
  pininvdyn::math::se3ToVector(H_lf, s.pos);
  m_contactLF->setReference(s);
  SEND_MSG("Setting left foot reference to "+toString(H_lf), MSG_TYPE_DEBUG);

  se3::SE3 H_rf = m_robot->position(*m_data,
                                    m_robot->model().getJointId(RIGHT_FOOT_FRAME_NAME));
  pininvdyn::math::se3ToVector(H_rf, s.pos);
  m_contactRF->setReference(s);
  SEND_MSG("Setting right foot reference to "+toString(H_rf), MSG_TYPE_DEBUG);
}

void HRP2DeviceTorqueCtrl::setVelocity( const ml::Vector& v )
{
  assert(v.size()==N_JOINTS+6);
  if(!m_initSucceeded)
  {
    SEND_WARNING_STREAM_MSG("Cannot setVelocity before initialization!");
    return;
  }
  Device::setVelocity(v);
  COPY_VECTOR_TO_VECTOR(v, m_v_sot);
  velocity_sot_to_urdf(m_v_sot, m_v);
}

void HRP2DeviceTorqueCtrl::integrate( const double & dt )
{
  if(!m_initSucceeded)
  {
    SEND_WARNING_STREAM_MSG("Cannot integrate before initialization!");
    return;
  }
  EIGEN_CONST_VECTOR_FROM_SIGNAL(tauDes, controlSIN.accessCopy());
  assert(tauDes.size()==N_JOINTS);

  // compute mass matrix
  m_robot->computeAllTerms(*m_data, m_q, m_v);
  m_K.topLeftCorner<N_JOINTS+6,N_JOINTS+6>() = m_robot->mass(*m_data);
  m_k.head<N_JOINTS+6>() = - m_robot->nonLinearEffects(*m_data);
  m_k.segment<N_JOINTS>(6) += tauDes;

  const ConstraintBase & constrRF = m_contactRF->compute(0.0, m_q, m_v, *m_data);
  const ConstraintBase & constrLF = m_contactLF->compute(0.0, m_q, m_v, *m_data);
  assert(constrRF.matrix().rows()==6);
  assert(constrLF.matrix().rows()==6);
  assert(constrRF.matrix().cols()==N_JOINTS+6);
  assert(constrLF.matrix().cols()==N_JOINTS+6);
  assert(constrRF.vector().size()==6);
  assert(constrLF.vector().size()==6);
  assert(m_Jc.rows()==12);
  assert(m_Jc.cols()==N_JOINTS+6);
  assert(m_K.rows()==N_JOINTS+6+12);
  assert(m_K.cols()==N_JOINTS+6+12);
  m_Jc.topRows<6>()                       = constrRF.matrix();
  m_Jc.bottomRows<6>()                    = constrLF.matrix();
  m_K.topRightCorner(N_JOINTS+6, m_nk)    = -m_Jc.transpose();
  m_K.bottomLeftCorner(m_nk, N_JOINTS+6)  = m_Jc;
  m_k.segment<6>(N_JOINTS+6)              = constrRF.vector();
  m_k.tail<6>()                           = constrLF.vector();


  // decompose K matrix to compute accelerations and forces
  m_K_chol.compute(m_K);
  m_K_chol.solveInPlace(m_k);
  m_dv = m_k.head<N_JOINTS+6>();
  m_f  = m_k.tail<12>();

  // integrate
  m_q = se3::integrate(m_robot->model(), m_q, dt*m_v);
  m_v += dt*m_dv;

  config_urdf_to_sot(m_q, m_q_sot);
  velocity_urdf_to_sot(m_v, m_v_sot);
  velocity_urdf_to_sot(m_dv, m_dv_sot);

  EIGEN_VECTOR_TO_VECTOR(m_q_sot, state_);
  EIGEN_VECTOR_TO_VECTOR(m_q_sot, base6d_encoders_);
  EIGEN_VECTOR_TO_VECTOR(m_v_sot, velocity_);
  EIGEN_VECTOR_TO_VECTOR(m_v_sot.tail<N_JOINTS>(), jointsVelocities_);
  EIGEN_VECTOR_TO_VECTOR(m_dv_sot.tail<N_JOINTS>(), jointsAccelerations_);

  // set the value for the encoders, joints velocities and accelerations output signal
  robotStateSOUT_.setConstant(base6d_encoders_);
  jointsVelocitiesSOUT_.setConstant(jointsVelocities_);
  jointsAccelerationsSOUT_.setConstant(jointsAccelerations_);

  int time = robotStateSOUT_.getTime()+1;
  for(int i=0; i<4; i++)
  {
    // read input force (if any)
    if(forcesSIN_[i]->isPlugged() && forcesSIN_[i]->operator()(time).size()==6)
      wrenches_[i] = forcesSIN_[i]->accessCopy();
    // add random noise
    for(int j=0; j<6; j++)
      temp6_(j) = wrenches_[i](j) + normalRandomNumberGenerator_();
    // set output force
    forcesSOUT[i]->setConstant(temp6_);
  }

  robotStateSOUT_.setTime(time);
  jointsVelocitiesSOUT_.setTime(time);
  jointsAccelerationsSOUT_.setTime(time);
  accelerometerSOUT_.setTime(time);
  gyrometerSOUT_.setTime(time);
  for( int i=0;i<4;++i )
    forcesSOUT[i]->setTime(time);
}
