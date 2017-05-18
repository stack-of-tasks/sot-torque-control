/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-dyninv is free software: you can redistribute it and/or
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

#include "sot/torque_control/hrp2-device-pos-ctrl.hh"
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace std;
using namespace dynamicgraph;
using namespace sot::torque_control;
using namespace metapod;

const double HRP2DevicePosCtrl::TIMESTEP_DEFAULT = 0.001;
const double HRP2DevicePosCtrl::FORCE_SENSOR_NOISE_STD_DEV = 1e-10;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(HRP2DevicePosCtrl,"HRP2DevicePosCtrl");

HRP2DevicePosCtrl::HRP2DevicePosCtrl(std::string RobotName):
    dynamicgraphsot::Device(RobotName),
    timestep_(TIMESTEP_DEFAULT),
    kpSIN_(NULL,"HRP2DevicePosCtrl(" + RobotName + ")::input(vector)::kp"),
    kdSIN_(NULL,"HRP2DevicePosCtrl(" + RobotName + ")::input(vector)::kd"),
    accelerometerSOUT_("HRP2DevicePosCtrl(" + RobotName + ")::output(vector)::accelerometer"),
    gyrometerSOUT_ ("HRP2DevicePosCtrl(" + RobotName + ")::output(vector)::gyrometer"),
    robotStateSOUT_ ("HRP2DevicePosCtrl(" + RobotName + ")::output(vector)::robotState"),
    jointsVelocitiesSOUT_ ("HRP2DevicePosCtrl(" + RobotName + ")::output(vector)::jointsVelocities"),
    jointsAccelerationsSOUT_ ("HRP2DevicePosCtrl(" + RobotName + ")::output(vector)::jointsAccelerations"),
    currentSOUT_ ("HRP2DevicePosCtrl(" + RobotName + ")::output(vector)::currents"),
    p_gainsSOUT_ ("HRP2DevicePosCtrl(" + RobotName + ")::output(vector)::p_gains"),
    d_gainsSOUT_ ("HRP2DevicePosCtrl(" + RobotName + ")::output(vector)::d_gains"),
    accelerometer_ (3),
    gyrometer_ (3),
    normalDistribution_(0.0, FORCE_SENSOR_NOISE_STD_DEV),
    normalRandomNumberGenerator_(randomNumberGenerator_,normalDistribution_)
{
    forcesSIN_[0] = new SignalPtr<dynamicgraph::Vector, int>(NULL, "HRP2DevicePosCtrl::input(vector6)::inputForceRLEG");
    forcesSIN_[1] = new SignalPtr<dynamicgraph::Vector, int>(NULL, "HRP2DevicePosCtrl::input(vector6)::inputForceLLEG");
    forcesSIN_[2] = new SignalPtr<dynamicgraph::Vector, int>(NULL, "HRP2DevicePosCtrl::input(vector6)::inputForceRARM");
    forcesSIN_[3] = new SignalPtr<dynamicgraph::Vector, int>(NULL, "HRP2DevicePosCtrl::input(vector6)::inputForceLARM");
    signalRegistration(kpSIN_<<kdSIN_<<
                       accelerometerSOUT_<<gyrometerSOUT_<<
                       robotStateSOUT_<<jointsVelocitiesSOUT_<<jointsAccelerationsSOUT_<<
                       *(forcesSIN_[0])<<*(forcesSIN_[1])<<*(forcesSIN_[2])<<*(forcesSIN_[3])<<
                       currentSOUT_ << p_gainsSOUT_ << d_gainsSOUT_);

    // initialize gyrometer and accelerometer
    dynamicgraph::Vector data(3); data.setZero();
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
    m_torques.setZero();
    m_q.setZero();
    m_dq.setZero();
    m_ddq_zero.setZero();
    m_h.setZero();
    m_ddq.setZero(N_JOINTS);


    using namespace dynamicgraph::command;
    std::string docstring;
    /* Command increment. */
    docstring = "\n    Integrate dynamics for time step provided as input\n"
                "\n      take one floating point number as input\n\n";
    addCommand("increment", makeCommandVoid1((Device&)*this,
                                &Device::increment, docstring));
}

HRP2DevicePosCtrl::~HRP2DevicePosCtrl()
{ }

void HRP2DevicePosCtrl::setStateSize(const unsigned int& size)
{
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

void HRP2DevicePosCtrl::integrate( const double & dt )
{
    const dynamicgraph::Vector & qDes = controlSIN.accessCopy();
    const dynamicgraph::Vector & kp   = kpSIN_.accessCopy();
    const dynamicgraph::Vector & kd   = kdSIN_.accessCopy();

    // compute PD control law to get joints' torques
    for(unsigned int i=0; i<state_.size()-6; ++i)
    {
      m_q(i+6)        = state_(i+6);
      m_dq(i+6)       = velocity_(i+6);
      m_torques(i+6)  = kp(i)*(qDes(i)-state_(i+6)) - kd(i)*velocity_(i+6);
    }

//    SEND_DEBUG_STREAM_MSG("q    = "+toString(m_q.transpose()));
//    SEND_DEBUG_STREAM_MSG("qDes = "+toString(qDes));
//    SEND_DEBUG_STREAM_MSG("dq   = "+toString(m_dq.transpose()));

    // compute mass matrix
    crba<Hrp2_14, true>::run(m_robot, m_q);
    // decompose mass matrix (only right bottom corner)
    m_robot.H.diagonal() += Hrp2_14::confVector::Constant(1e-3); // regularize mass matrix
    m_H_chol.compute(m_robot.H.bottomRightCorner<N_JOINTS,N_JOINTS>());
    // compute bias forces
    rnea< Hrp2_14, true>::run(m_robot, m_q, m_dq, m_ddq_zero);
    getTorques(m_robot, m_h);

    m_ddq = m_torques.tail<N_JOINTS>() - m_h.tail<N_JOINTS>();
//    SEND_MSG("tau-h = "+toString(m_ddq.transpose()),MSG_TYPE_DEBUG_STREAM);
    m_H_chol.solveInPlace(m_ddq);

//    const int JID = 1;
//    SEND_MSG(toString(robotStateSOUT_.getTime())+" tau  = "+toString(m_torques[6+JID]),MSG_TYPE_DEBUG);
//    SEND_MSG("qDes= "+toString(qDes(JID)),MSG_TYPE_DEBUG);
//    SEND_MSG("q   = "+toString(m_q[6+JID]),MSG_TYPE_DEBUG);
//    SEND_MSG("dq  = "+toString(m_dq[6+JID]),MSG_TYPE_DEBUG);
//    SEND_MSG("ddq = "+toString(m_ddq(JID)),MSG_TYPE_DEBUG);

//    SEND_MSG("diag(H) = "+toString(m_robot.H.diagonal().transpose()),MSG_TYPE_DEBUG_STREAM);
//    SEND_MSG("h = "+toString(m_h.transpose()),MSG_TYPE_DEBUG_STREAM);
//    SEND_DEBUG_STREAM_MSG("ddq = "+toString(m_ddq.transpose()));

    // integrate
    double dts2 = 0.5*dt*dt;
    for(unsigned int i=0; i<state_.size()-6; ++i)
    {
        // integrate joints' accelerations to get new vel and pos
        state_(i+6)     += dt*velocity_(i+6) + dts2*m_ddq(i)*m_ddq(i);
        velocity_(i+6)  += dt*m_ddq(i);
        jointsAccelerations_(i) = m_ddq(i);

        base6d_encoders_(i+6) = state_(i+6);
        jointsVelocities_(i)  = velocity_(i+6);
    }
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

//void HRP2DevicePosCtrl::integrate( const double & dt )
//{
//    const dynamicgraph::Vector & qDes = controlSIN.accessCopy();
//    const dynamicgraph::Vector & kp   = kpSIN_.accessCopy();
//    const dynamicgraph::Vector & kd   = kdSIN_.accessCopy();

//    double dts2 = 0.5*dt*dt;
//    for(unsigned int i=0; i<state_.size()-6; ++i)
//    {
//        // compute PD control law to get joints' accelerations
//        jointsAccelerations_(i) = kp(i)*(qDes(i)-state_(i+6)) - kd(i)*velocity_(i+6);
//        // integrate joints' accelerations to get new vel and pos
//        state_(i+6)           += dt*velocity_(i+6) + dts2*jointsAccelerations_(i)*jointsAccelerations_(i);
//        velocity_(i+6)        += dt*jointsAccelerations_(i);

//        base6d_encoders_(i+6) = state_(i+6);
//        jointsVelocities_(i)  = velocity_(i+6);

//        // temp
////        base6d_encoders_(i+6) = qDes(i);
//        //end temp
//    }
//    // set the value for the encoders, joints velocities and accelerations output signal
//    robotStateSOUT_.setConstant(base6d_encoders_);
//    jointsVelocitiesSOUT_.setConstant(jointsVelocities_);
//    jointsAccelerationsSOUT_.setConstant(jointsAccelerations_);

//    int time = robotStateSOUT_.getTime()+1;
//    for(int i=0; i<4; i++)
//    {
//      // read input force (if any)
//      if(forcesSIN_[i]->isPlugged() && forcesSIN_[i]->operator()(time).size()==6)
//        wrenches_[i] = forcesSIN_[i]->accessCopy();
//      // add random noise
//      for(int j=0; j<6; j++)
//        temp6_(j) = wrenches_[i](j) + normalRandomNumberGenerator_();
//      // set output force
//      forcesSOUT[i]->setConstant(temp6_);
//    }

//    robotStateSOUT_.setTime(time);
//    jointsVelocitiesSOUT_.setTime(time);
//    jointsAccelerationsSOUT_.setTime(time);
//    accelerometerSOUT_.setTime(time);
//    gyrometerSOUT_.setTime(time);
//    for( int i=0;i<4;++i )
//      forcesSOUT[i]->setTime(time);
//}

