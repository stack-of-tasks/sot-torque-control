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

#ifndef __sot_torque_control_NumericalDifference_H__
#define __sot_torque_control_NumericalDifference_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (numerical_difference_EXPORTS)
#    define SOTNUMERICALDIFFERENCE_EXPORT __declspec(dllexport)
#  else
#    define SOTNUMERICALDIFFERENCE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTNUMERICALDIFFERENCE_EXPORT
#endif

//#define VP_DEBUG 1        /// enable debug output
//#define VP_DEBUG_MODE 20

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* HELPER */
#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/metapod-helper.hh>
#include <sot/torque_control/utils/stop-watch.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/hrp2-common.hh>
#include <boost/circular_buffer.hpp>

/* Polynomial estimators */
#include <sot/torque_control/utils/lin-estimator.hh>
#include <sot/torque_control/utils/quad-estimator.hh>

/* Metapod */
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <metapod/algos/rnea.hh>
#include <metapod/tools/bcalc.hh>
#include <metapod/tools/print.hh>
#include <metapod/tools/initconf.hh>

/*Motor model*/
#include <sot/torque_control/motor-model.hh>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {




      /**
        * This Entity takes as inputs the joints' encoders,
        * the IMU and the force/torque sensors' measurements and it computes
        * as output estimates of the joints' torques, the contact
        * forces, the joints' positions, velocities and accelerations.
        * It also takes current measurment if any available, to compute 
        * a second estimate of the joints' torques from the motor model.
        * The two torques estimations can be mixed with signal wCurrentTrust
        * (value from 0 to 1)
        * As current measurment can saturate to a lower value than the 
        * real motor current, an input signal saturationCurrent must be provide.
        * If a current sensor is not available for some joint, this value 
        * should be set to zero.
        * 
        * QUICK START
        * Create the entity, plug all the input signals, call the init method
        * specifying the control-loop time step and the desired delay introduced
        * by the estimation (at least 1.5 times the time step). For instance:
        *   estimator = NumericalDifference("estimator");
        *   plug(device.robotState,     estimator.base6d_encoders);
        *   plug(device.accelerometer,  estimator.accelerometer);
        *   plug(device.gyrometer,      estimator.gyroscope);
        *   plug(device.forceRLEG,      estimator.ftSensRightFoot);
        *   plug(device.forceLLEG,      estimator.ftSensLeftFoot);
        *   plug(device.forceRARM,      estimator.ftSensRightHand);
        *   plug(device.forceLARM,      estimator.ftSensLeftHand);
        *   estimator.init(dt, estimationDelay);
        * Note that the input signals must be plugged before calling init, otherwise
        * the estimation cannot compute the force/torque sensors offsets and
        * the initialization will fail.
        *
        * After this you can read the estimates on the output signals:
        * - jointsPositions: in rad (30d)
        * - jointsVelocities: in rad/s (30d)
        * - jointsAccelerations: in rad/s^2 (30d)
        * - torsoAcceleration: linear + angular in torso's reference frame (6d)
        * - torsoAngularVelocity: in the torso's reference frame (3d)
        * - contactWrenchLeftFoot: equivalent wrench at the frame of the left foot in local coordinate (6d)
        * - contactWrenchRighttFoot: equivalent wrench at the frame of the right foot in local coordinate (6d)
        * - contactWrenchLeftSole: equivalent wrench at the sole of the left foot in local coordinate (6d)
        * - contactWrenchRighttSole: equivalent wrench at the sole of the right foot in local coordinate (6d)
        * - contactWrenchLeftHand: equivalent wrench at the frame of the left hand in local coordinate (6d)
        * - contactWrenchRightHand: equivalent wrench at the frame of the right hand in local coordinate (6d)
        * - contactWrenchBody: equivalent wrench at the base in local coordinate (6d)
        * - jointsTorques: in N*m (30d)
        *
        * DETAILS
        * Joints' velocities and accelerations are computed by fitting
        * a 2-nd order polynomial to a fixed-length window of the joints'
        * positions (as measured by the encoders). The position and velocity
        * are taken in the middle of the window (the acceleration is constant
        * over the whole window since the polynomial is only 2-nd order),
        * so that the delay is half of the window's length.
        *
        * The base's acceleration (lin+ang) and angular velocity are
        * estimated from the IMU's measurements (i.e. linear acceleration
        * and angular velocity). We use again a polynomial fitting over
        * a fixed-length window, but this time the polynomial is of 1-st
        * order because we only need to derivate once. The window's length
        * is the same used for the joints so that the estimation's delay
        * is consistent for all the quantities. Note that we do not need
        * to estimate the position, orientation and linear velocity of the
        * base because they do not affect the dynamics of the robot. Actually
        * the base's orientation indirectly affects the dynamics because it
        * is related to the gravity fields, but since the linear acceleration
        * measured by the IMU includes the gravity accelerations we do not
        * need to know the base's orientation to estimate the joints' torques
        * and the contact forces.
        *
        * The contact forces (actually they are 6d wrench, but in the following
        * we keep using force as a synonim of wrench) are always assumed to
        * be at the four end-effectors (hands and feet) and on the base of the
        * robot. Given that the robot is equipped with 4 F/T sensors, only 5
        * contact forces are observable. The assumption on the
        * position of 4 contact forces at the end-effectors is dictated by
        * the location of the F/T sensors. On the contrary, the 5-th contact
        * force could be assumed anywhere on the robot's body; currently
        * we assume it to be on the base, so that we do not have to modify
        * the order in which forces are propagated in the RNEA.
        * Given their locations, contact forces can be estimated by running
        * the Recursive Newton-Euler Algorithm (RNEA). We first propagates
        * pos, vel and acc from the robot's base to the end-effectors. Then
        * we propagate the measurements of the F/T sensors towards the locations
        * of the contacts. This procedure estimates at the same time the contact
        * forces and the joint torques.
        */
      class SOTNUMERICALDIFFERENCE_EXPORT NumericalDifference
          :public ::dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:  /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(x,                 dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(x_filtered,       dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(dx,               dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(ddx,              dynamicgraph::Vector);

        /// The following inner signals are used because this entity has some output signals
        /// whose related quantities are computed at the same time by the same algorithm
        /// (e.g. torques and contact wrenches are computed by the RNEA). to avoid the risk
        /// or recomputing the same things twice, we create an inner signal that groups together
        /// all the quantities that are computed together. Then the single output signals will depend
        /// on this inner signal, which is the one triggering the computations.
        /// Inner signals are not exposed, so that nobody can access them.

        /// This signal contains the estimated joints positions, velocities and accelerations.
        DECLARE_SIGNAL_INNER(x_dx_ddx,              dynamicgraph::Vector);
        
      protected:
      
        double m_dt;              /// timestep of the controller
        double m_delay;   /// delay introduced by the estimation of joints pos/vel/acc
        int x_size;
        /// std::vector to use with the filters
        /// All the variables whose name contains 'filter' are outputs of the filters
        std::vector<double> m_ddx_filter_std;  /// 2nd derivative
        std::vector<double> m_dx_filter_std;   /// 1st derivative
        std::vector<double> m_x_filter_std;    /// filtered output
        std::vector<double> m_x_std;           /// x signal


        /// polynomial-fitting filters
        PolyEstimator* m_filter;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** --- CONSTRUCTOR ---- */
        NumericalDifference( const std::string & name );

        /** Initialize the NumericalDifference.
         * @param timestep Period (in seconds) after which the sensors' data are updated.
         * @param delay    Delay (in seconds) introduced by the estimation of joints pos/vel/acc.
         *                        This should be a multiple of timestep.
         * @param delayAcc Delay (in seconds) introduced by the low-pass filtering of the accelerometer.
         *                        This should be a multiple of timestep.
         * @param delayGyro Delay (in seconds) introduced by the low-pass filtering of the gyroscope.
         *                        This should be a multiple of timestep.
         * @note The estimationDelay is half of the length of the window used for the
         * polynomial fitting. The larger the delay, the smoother the estimations.
         */
        void init(const double &timestep, const int& sigSize, const double &delay);

      protected:
        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("["+name+"] "+msg, t, file, line);
        }

      public: /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

      }; // class NumericalDifference

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_NumericalDifference_H__
