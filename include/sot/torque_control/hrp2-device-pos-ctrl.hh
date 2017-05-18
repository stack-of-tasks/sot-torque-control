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

#ifndef _HRP2DevicePosCtrl_H_
#define _HRP2DevicePosCtrl_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>
//#include <sot/core/matrix-geometry.hh>

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/hrp2-common.hh>
#include <sot/torque_control/utils/logger.hh>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

/* Metapod */
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <metapod/algos/crba.hh>
#include <metapod/algos/rnea.hh>
#include <metapod/tools/bcalc.hh>
#include <metapod/tools/print.hh>
#include <metapod/tools/initconf.hh>

#include <Eigen/Cholesky>

namespace dynamicgraphsot=dynamicgraph::sot;

namespace dynamicgraph
{
namespace sot
{
namespace torque_control
{

/** Version of HRP2 device for testing the code without the
  * real robot. This version of the device assumes that the robot
  * is position controlled (i.e. the control input are the desired
  * joints' angles). These desired joints' angles are used to compute
  * the joints' accelerations by means of a PD control law.
  * The proportional and derivative gains are input signals of the device.
  * The joints' accelerations are then integrated to get the
  * measured joints' angles.
  *
  * The base is supposed to be fixed and the user can set its pose
  * with the command "setRoot". The accelerometer
  * and the gyrometer output a constant value.
  *
  * A white Gaussian noise is added to the force/torque sensor
  * measurements.
  *
  * TODO: The original Device class should be a clean abstraction for
  * the concept of device, but currently it is not. It defines a lot
  * of input/output signals that are specific of HRP-2 (e.g. zmpSIN,
  * attitudeSIN, forcesSOUT) and some design choices (e.g. secondOrderIntegration).
  * It would be nice to clean the original Device from all this stuff
  * and move it in the specific subclasses.
  */
class HRP2DevicePosCtrl: public
        dynamicgraphsot::Device
{
public:

    static const std::string CLASS_NAME;
    static const double TIMESTEP_DEFAULT;
    static const double FORCE_SENSOR_NOISE_STD_DEV;

    virtual const std::string& getClassName () const
    {
        return CLASS_NAME;
    }

    HRP2DevicePosCtrl(std::string RobotName);
    virtual ~HRP2DevicePosCtrl();

    virtual void setStateSize(const unsigned int& size);

protected:
    virtual void integrate(const double & dt);

    void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
    {
      getLogger().sendMsg("[HRP2DevicePosCtrl] "+msg, t, file, line);
    }

    /// \brief Current integration step.
    double timestep_;

    /// proportional gains
    dynamicgraph::SignalPtr <dynamicgraph::Vector, int> kpSIN_;
    /// derivative gains
    dynamicgraph::SignalPtr <dynamicgraph::Vector, int> kdSIN_;
    /// input force sensor values
    dynamicgraph::SignalPtr <dynamicgraph::Vector, int>* forcesSIN_[4];

    /// Accelerations measured by accelerometers
    dynamicgraph::Signal <dynamicgraph::Vector, int> accelerometerSOUT_;
    /// Rotation velocity measured by gyrometers
    dynamicgraph::Signal <dynamicgraph::Vector, int> gyrometerSOUT_;
    /// base 6d pose + joints' angles measured by encoders
    dynamicgraph::Signal <dynamicgraph::Vector, int> robotStateSOUT_;
    /// joints' velocities computed by the integrator
    dynamicgraph::Signal <dynamicgraph::Vector, int> jointsVelocitiesSOUT_;
    /// joints' accelerations computed by the integrator
    dynamicgraph::Signal <dynamicgraph::Vector, int> jointsAccelerationsSOUT_;
    /// motor currents
    dynamicgraph::Signal <dynamicgraph::Vector, int> currentSOUT_;
    /// proportional and derivative position-control gains
    dynamicgraph::Signal <dynamicgraph::Vector, int> p_gainsSOUT_;
    dynamicgraph::Signal <dynamicgraph::Vector, int> d_gainsSOUT_;

    /// Intermediate variables to avoid allocation during control
    dynamicgraph::Vector accelerometer_;
    dynamicgraph::Vector gyrometer_;

    dynamicgraph::Vector base6d_encoders_;      /// base 6d pose + joints' angles
    dynamicgraph::Vector jointsVelocities_;     /// joints' velocities
    dynamicgraph::Vector jointsAccelerations_;  /// joints' accelerations

    dynamicgraph::Vector wrenches_[4];
    dynamicgraph::Vector temp6_;

    /// robot geometric/inertial data
    typedef metapod::hrp2_14<double> Hrp2_14;
    Hrp2_14 m_robot;
    Hrp2_14::confVector m_q, m_dq, m_ddq_zero;
    Hrp2_14::confVector m_torques;
    Hrp2_14::confVector m_h;

    typedef Eigen::LDLT<Eigen::MatrixXd> Cholesky;
    Cholesky        m_H_chol; /// cholesky decomposition of mass matrix
    Eigen::VectorXd m_ddq;

    typedef boost::mt11213b                    ENG;    // uniform random number generator
    typedef boost::normal_distribution<double> DIST;   // Normal Distribution
    typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator
    ENG  randomNumberGenerator_;
    DIST normalDistribution_;
    GEN  normalRandomNumberGenerator_;
};

}   // end namespace torque_control
}   // end namespace sot
}   // end namespace dynamicgraph
#endif /* HRP2DevicePosCtrl*/
