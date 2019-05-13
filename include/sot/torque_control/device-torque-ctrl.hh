/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
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

#ifndef _DeviceTorqueCtrl_H_
#define _DeviceTorqueCtrl_H_

#include <Eigen/Cholesky>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/tasks/task-se3-equality.hpp>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>

/* HELPER */
#include <dynamic-graph/signal-helper.h>
#include <sot/core/robot-utils.hh>


namespace dgsot=dynamicgraph::sot;

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {

      /** Version of device for testing the code without the
      * real robot. This version of the device assumes that the robot
      * is torque controlled (i.e. the control input are the desired
      * joint torques). These desired joint torques are used to compute
      * the joint accelerations and the contact forces.
      * The joint accelerations are then integrated to get the
      * measured joint angles.
      *
      * The feet are supposed to be fixed to the ground. The accelerometer
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
      class DeviceTorqueCtrl: public
          dgsot::Device
      {
      public:

        static const std::string CLASS_NAME;
        static const double TIMESTEP_DEFAULT;
        static const double FORCE_SENSOR_NOISE_STD_DEV;

        virtual const std::string& getClassName () const
        {
          return CLASS_NAME;
        }

        DeviceTorqueCtrl(std::string RobotName);
        virtual ~DeviceTorqueCtrl();

        virtual void setStateSize(const unsigned int& size);
        virtual void setState(const dynamicgraph::Vector& st);
        virtual void setVelocity(const dynamicgraph::Vector & vel);
        virtual void setControlInputType(const std::string& cit);

        virtual void init(const double& dt, const std::string& urdfFile);

      protected:
        virtual void integrate(const double & dt);
        void computeForwardDynamics();

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
	  Entity::sendMsg("[DeviceTorqueCtrl] "+msg, t, file, line);
        }

        /// \brief Current integration step.
        double timestep_;
        bool m_initSucceeded;

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

        DECLARE_SIGNAL_IN(kp_constraints,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_constraints,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(rotor_inertias,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(gear_ratios,                dynamicgraph::Vector);

        /// Intermediate variables to avoid allocation during control
        dynamicgraph::Vector accelerometer_;
        dynamicgraph::Vector gyrometer_;

        dynamicgraph::Vector base6d_encoders_;      /// base 6d pose + joints' angles
        dynamicgraph::Vector jointsVelocities_;     /// joints' velocities
        dynamicgraph::Vector jointsAccelerations_;  /// joints' accelerations

        dynamicgraph::Vector wrenches_[4];
        dynamicgraph::Vector temp6_;

        bool m_isTorqueControlled;

        /// robot geometric/inertial data
        tsid::robots::RobotWrapper *               m_robot;
        pinocchio::Data *                          m_data;
        tsid::tasks::TaskSE3Equality *             m_contactRF;
        tsid::tasks::TaskSE3Equality *             m_contactLF;
        unsigned int                               m_nk; // number of contact forces

        tsid::math::Vector m_q, m_v, m_dv, m_f;
        tsid::math::Vector m_q_sot, m_v_sot, m_dv_sot;

        typedef Eigen::LDLT<Eigen::MatrixXd> Cholesky;
        Cholesky        m_K_chol; /// cholesky decomposition of the K matrix
        Eigen::MatrixXd m_K;
        Eigen::VectorXd m_k;
        Eigen::MatrixXd m_Jc; /// constraint Jacobian

        double                    m_numericalDamping; /// numerical damping to regularize constraint resolution
        Eigen::JacobiSVD<Matrix>  m_Jc_svd;     /// svd of the constraint matrix
        Vector                    m_dJcv;
        Matrix                    m_Z;          /// base of constraint null space
        Matrix                    m_ZMZ;        /// projected mass matrix: Z_c^T*M*Z_c
        Eigen::LDLT<Matrix>       m_ZMZ_chol;   /// Cholesky decomposition of _ZMZ
        Vector                    m_dv_c;       /// constrained accelerations
        Vector                    m_dvBar;      /// solution of Jc*dv=-dJc*v
        Vector                    m_tau_np6;
        int                       m_nj;         /// number of joints

        typedef boost::mt11213b                    ENG;    // uniform random number generator
        typedef boost::normal_distribution<double> DIST;   // Normal Distribution
        typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator
        ENG  randomNumberGenerator_;
        DIST normalDistribution_;
        GEN  normalRandomNumberGenerator_;

        RobotUtilShrPtr m_robot_util;
      };

    }   // end namespace torque_control
  }   // end namespace sot
}   // end namespace dynamicgraph
#endif /* DevicePosCtrl*/
