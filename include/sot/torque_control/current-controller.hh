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

#ifndef __sot_torque_control_current_controller_H__
#define __sot_torque_control_current_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (__sot_torque_control_current_controller_H__)
#    define SOTCURRENTCONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define SOTCURRENTCONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTCURRENTCONTROLLER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/common.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"


#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <tsid/robots/robot-wrapper.hpp>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTCURRENTCONTROLLER_EXPORT CurrentController
        :public::dynamicgraph::Entity
      {
        typedef CurrentController EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        /* --- CONSTRUCTOR ---- */
        CurrentController( const std::string & name);

	/// Initialize
	/// @param dt: control interval
	/// @param currentOffsetIters: number of iterations while control is disabled to calibrate current sensors. 
	/// The recommended way is to use the signal max_current.
        void init(const double & dt,
                  const std::string & robotRef,
                  const unsigned int & currentOffsetIters);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(i_des,                                 dynamicgraph::Vector);  /// desired motor currents
        DECLARE_SIGNAL_IN(i_measured,                            dynamicgraph::Vector);  /// motor currents
        DECLARE_SIGNAL_IN(i_sens_gains,                          dynamicgraph::Vector);  /// gains of current sensors
        DECLARE_SIGNAL_IN(kp_current,                            dynamicgraph::Vector);  /// proportional current feedback gain
        DECLARE_SIGNAL_IN(ki_current,                            dynamicgraph::Vector);  /// proportional current feedback gain

        DECLARE_SIGNAL_IN(i_max,                                 dynamicgraph::Vector);  /// max current allowed before stopping the controller (in Ampers)
        DECLARE_SIGNAL_IN(u_max,                                 dynamicgraph::Vector);  /// max desired current allowed before stopping the controller (in Ampers)
        DECLARE_SIGNAL_IN(u_saturation,                          dynamicgraph::Vector);  /// values at which to saturate the control (in bits)

        DECLARE_SIGNAL_IN(in_out_gain,                           dynamicgraph::Vector);  /// gain from input to output control values
        DECLARE_SIGNAL_IN(dq,                                    dynamicgraph::Vector);  /// Joint velocities; used to compensate for BEMF effect on low level current loop
        DECLARE_SIGNAL_IN(bemf_factor,                           dynamicgraph::Vector);  /// Link between velocity and current; to compensate for BEMF effect on low level current loop (in A/rad.s-1)
        DECLARE_SIGNAL_IN(percentage_bemf_compensation,          dynamicgraph::Vector);  /// percentage in [0;1] of the motor back-EMF that we should compensate 0 is none, 1 is all of it
        DECLARE_SIGNAL_IN(dead_zone_offsets,                     dynamicgraph::Vector);  /// current control dead zone offsets
        DECLARE_SIGNAL_IN(percentage_dead_zone_compensation,     dynamicgraph::Vector);  /// percentage in [0;1] of the motor driver dead zone that we should compensate 0 is none, 1 is all of it
        DECLARE_SIGNAL_IN(i_max_dead_zone_compensation,          dynamicgraph::Vector);  /// value of current tracking error at which deadzone is completely compensated
        DECLARE_SIGNAL_IN(i_sensor_offsets_low_level,            dynamicgraph::Vector);  /// offset of the current sensors seen by the low level
        DECLARE_SIGNAL_IN(i_sensor_offsets_real_in,              dynamicgraph::Vector);  /// real offset of the current sensors

        DECLARE_SIGNAL_OUT(u,                                    dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(u_safe,                               dynamicgraph::Vector);  /// same as u when everything is fine, 0 otherwise
        DECLARE_SIGNAL_OUT(i_real,                               dynamicgraph::Vector);  /// current measurements after gain and offset compensation
        DECLARE_SIGNAL_OUT(i_low_level,                          dynamicgraph::Vector);  /// current measurements as seen by low-level ctrl
        DECLARE_SIGNAL_OUT(i_sensor_offsets_real_out,            dynamicgraph::Vector);  /// real offset of the current sensors
        DECLARE_SIGNAL_OUT(dead_zone_compensation,               dynamicgraph::Vector);  /// dead-zone compensation current applied by the controller
        DECLARE_SIGNAL_OUT(i_errors,                             dynamicgraph::Vector);  /// current tracking error
        DECLARE_SIGNAL_OUT(i_errors_ll_wo_bemf,                  dynamicgraph::Vector);  /// current tracking error without BEMF effect


        /* --- COMMANDS --- */

        void reset_integral();

        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[CurrentController-"+name+"] "+msg, t, file, line);
        }

      protected:
	RobotUtil * m_robot_util;
        bool    m_initSucceeded;    /// true if the entity has been successfully initialized
        bool    m_emergency_stop_triggered;
        double  m_dt;               /// control loop time period
        bool    m_is_first_iter;    /// true at the first iteration, false otherwise
        int     m_iter;
        double  m_sleep_time;       /// time to sleep at every iteration (to slow down simulation)

        unsigned int m_currentOffsetIters;
        dynamicgraph::Vector m_i_offsets_real;
        dynamicgraph::Vector m_i_err_integr;

        dynamicgraph::Vector m_dz_coeff;

        dynamicgraph::Vector m_avg_i_err_pos;
        dynamicgraph::Vector m_avg_i_err_neg;

      }; // class CurrentController

    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_current_controller_H__
