/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
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

#ifndef __sot_torque_control_position_controller_H__
#define __sot_torque_control_position_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (position_controller_EXPORTS)
#    define SOTPOSITIONCONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define SOTPOSITIONCONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTPOSITIONCONTROLLER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/common.hh>
#include <map>
#include "boost/assign.hpp"

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTPOSITIONCONTROLLER_EXPORT PositionController
	:public::dynamicgraph::Entity
      {
        typedef PositionController EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        PositionController( const std::string & name );

        void init(const double& dt,const std::string &robotRef);

        void resetIntegral();

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(base6d_encoders,  dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(jointsVelocities, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(qRef,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(dqRef,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(Kp,               dynamicgraph::Vector);  /// joint proportional gains
        DECLARE_SIGNAL_IN(Kd,               dynamicgraph::Vector);  /// joint derivative gains
        DECLARE_SIGNAL_IN(Ki,               dynamicgraph::Vector);  /// joint integral gains

        DECLARE_SIGNAL_OUT(pwmDes,      dynamicgraph::Vector);  /// Kp*e_q + Kd*de_q + Ki*int(e_q)
        // DEBUG SIGNALS
        DECLARE_SIGNAL_OUT(qError,      dynamicgraph::Vector);  /// qRef-q


        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[PositionController-"+name+"] "+msg, t, file, line);
        }

      protected:
	RobotUtil *       m_robot_util;        /// Robot Util
        Eigen::VectorXd   m_pwmDes;
        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        double            m_dt;               /// control loop time period

        /// Integral of the joint tracking errors
        Eigen::VectorXd   m_e_integral;

        Eigen::VectorXd m_q, m_dq;

      }; // class PositionController

    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_position_controller_H__
