/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
 *
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

#include <map>
#include "boost/assign.hpp"

#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>

#include <sot/torque_control/utils/vector-conversions.hh>

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
	  Entity::sendMsg("[PositionController-"+name+"] "+msg, t, file, line);
        }

      protected:
	RobotUtilShrPtr       m_robot_util;        /// Robot Util
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
