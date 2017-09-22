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

#ifndef __sot_torque_control_control_manager_H__
#define __sot_torque_control_control_manager_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (__sot_torque_control_control_manager_H__)
#    define SOTCONTROLMANAGER_EXPORT __declspec(dllexport)
#  else
#    define SOTCONTROLMANAGER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTCONTROLMANAGER_EXPORT
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

/// Number of time step to transition from one ctrl mode to another
#define CTRL_MODE_TRANSITION_TIME_STEP 1000.0

// max motor current
#define DEFAULT_MAX_CURRENT 8

// number of iterations used to compute current offset at the beginning
#define CURRENT_OFFSET_ITERS 200

      class CtrlMode
      {
      public:
        int         id;
        std::string name;

        CtrlMode(): id(-1), name("None"){}
        CtrlMode(int id, const std::string& name):
          id(id), name(name) {}
      };

      std::ostream& operator<<( std::ostream& os, const CtrlMode& s )
      {
        os<<s.id<<"_"<<s.name;
        return os;
      }

      class SOTCONTROLMANAGER_EXPORT ControlManager
        :public::dynamicgraph::Entity
      {
        typedef ControlManager EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        /* --- CONSTRUCTOR ---- */
        ControlManager( const std::string & name);

	/// Initialize
	/// @param dt: control interval
	/// @param urdfFile: path to the URDF model of the robot
	/// @param maxCurrent: default maximum current for each motor. 
	/// @param currentOffsetIters: number of iterations while control is disabled to calibrate current sensors. 
	/// The recommended way is to use the signal max_current.
        void init(const double & dt,
                  const std::string & urdfFile,
                  const double & maxCurrent,
                  const std::string & robotRef,
                  const unsigned int & currentOffsetIters);

        /* --- SIGNALS --- */
        std::vector<dynamicgraph::SignalPtr<dynamicgraph::Vector,int>*> m_ctrlInputsSIN;
        std::vector< dynamicgraph::SignalPtr<bool,int>* >               m_emergencyStopSIN; /// emergency stop inputs. If one is true, control is set to zero forever
        std::vector<dynamicgraph::Signal<dynamicgraph::Vector,int>*>    m_jointsCtrlModesSOUT;

        DECLARE_SIGNAL_IN(base6d_encoders,                       dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(dq,                                    dynamicgraph::Vector);  /// Joint velocities; used to compensate for BEMF effect on low level current loop
        DECLARE_SIGNAL_IN(bemfFactor,                            dynamicgraph::Vector);  /// Link between velocity and current; to compensate for BEMF effect on low level current loop (in A/rad.s-1)
        DECLARE_SIGNAL_IN(in_out_gain,                           dynamicgraph::Vector);  /// gain from input to output control values
        DECLARE_SIGNAL_IN(cur_sens_gains,                        dynamicgraph::Vector);  /// gains of current sensors
        DECLARE_SIGNAL_IN(dead_zone_offsets,                     dynamicgraph::Vector);  /// current control dead zone offsets
        DECLARE_SIGNAL_IN(currents,                              dynamicgraph::Vector);  /// motor currents
        DECLARE_SIGNAL_IN(tau,                                   dynamicgraph::Vector);  /// estimated joint torques (using dynamic robot model + F/T sensors)
        DECLARE_SIGNAL_IN(tau_predicted,                         dynamicgraph::Vector);  /// predicted joint torques (using motor model)
        DECLARE_SIGNAL_IN(max_current,                           dynamicgraph::Vector);  /// max current allowed before stopping the controller (in Ampers)
        DECLARE_SIGNAL_IN(max_tau,                               dynamicgraph::Vector);  /// max torque allowed before stopping the controller
        DECLARE_SIGNAL_IN(percentageDriverDeadZoneCompensation,  dynamicgraph::Vector);  /// percentatge in [0;1] of the motor driver dead zone that we should compensate 0 is none, 1 is all of it
        DECLARE_SIGNAL_IN(percentage_bemf_compensation,          dynamicgraph::Vector);  /// percentatge in [0;1] of the motor back-EMF that we should compensate 0 is none, 1 is all of it
        DECLARE_SIGNAL_IN(iMaxDeadZoneCompensation,              dynamicgraph::Vector);  /// value of current tracking error at which deadzone is completely compensated
        DECLARE_SIGNAL_IN(current_sensor_offsets_low_level,      dynamicgraph::Vector);  /// offset of the current sensors seen by the low level
        DECLARE_SIGNAL_IN(current_sensor_offsets_real_in,        dynamicgraph::Vector);  /// real offset of the current sensors
        DECLARE_SIGNAL_IN(kp_current,                            dynamicgraph::Vector);  /// proportional current feedback gain
        DECLARE_SIGNAL_IN(ki_current,                            dynamicgraph::Vector);  /// proportional current feedback gain

        DECLARE_SIGNAL_OUT(pwmDes,                               dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(pwmDesSafe,                           dynamicgraph::Vector);  /// same as pwmDes when everything is fine, 0 otherwise //TODO change since pwmDes is now the desired current and pwmDesSafe is the DAC 
        DECLARE_SIGNAL_OUT(currents_real,                        dynamicgraph::Vector);  /// current measurements after gain and offset compensation
        DECLARE_SIGNAL_OUT(currents_low_level,                   dynamicgraph::Vector);  /// current measurements as seen by low-level ctrl
        DECLARE_SIGNAL_OUT(current_sensor_offsets_real_out,      dynamicgraph::Vector);  /// real offset of the current sensors
        DECLARE_SIGNAL_OUT(dead_zone_compensation,               dynamicgraph::Vector);  /// dead-zone compensation current applied by the controller
        DECLARE_SIGNAL_OUT(current_errors,                       dynamicgraph::Vector);  /// current tracking error
        DECLARE_SIGNAL_OUT(current_errors_ll_wo_bemf,            dynamicgraph::Vector);  /// current tracking error without BEMF effect


        /* --- COMMANDS --- */

	/// Commands related to the control mode.
        void addCtrlMode(const std::string& name);
        void ctrlModes();
        void getCtrlMode(const std::string& jointName);
        void setCtrlMode(const std::string& jointName, const std::string& ctrlMode);
        void setCtrlMode(const int jid, const CtrlMode& cm);
	
        void resetProfiler();

        void setDefaultMaxCurrent(const double &lDefaultMaxCurrent);
	
	/// Commands related to joint name and joint id
	void setNameToId(const std::string& jointName, const double & jointId);
	void setJointLimitsFromId(const double &jointId, 
				const double &lq, const double &uq);

	/// Command related to ForceUtil
	void setForceLimitsFromId(const double &jointId, 
				  const dynamicgraph::Vector &lq, 
				  const dynamicgraph::Vector &uq);
	void setForceNameToForceId(const std::string& forceName, 
				   const double & forceId);
	
	/// Commands related to FootUtil
	void setRightFootSoleXYZ(const dynamicgraph::Vector &);
        void setRightFootForceSensorXYZ(const dynamicgraph::Vector &);
	void setFootFrameName(const std::string &, const std::string &);
    void setImuJointName(const std::string &);
	void displayRobotUtil();
	/// Set the mapping between urdf and sot.
	void setJoints(const dynamicgraph::Vector &);

        void setStreamPrintPeriod(const double & s);
        void setSleepTime(const double &seconds);
        void addEmergencyStopSIN(const std::string& name);

        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[ControlManager-"+name+"] "+msg, t, file, line);
        }

      protected:
	RobotUtil * m_robot_util;
	tsid::robots::RobotWrapper *                       m_robot;
        bool    m_initSucceeded;    /// true if the entity has been successfully initialized
        double  m_dt;               /// control loop time period
        double  m_maxCurrent;       /// control limit in Ampers
        bool    m_emergency_stop_triggered;  /// true if an emergency condition as been triggered either by an other entity, or by control limit violation
        bool    m_is_first_iter;    /// true at the first iteration, false otherwise
        int     m_iter;
        double  m_sleep_time;       /// time to sleep at every iteration (to slow down simulation)

        unsigned int m_currentOffsetIters;
//        dynamicgraph::Vector m_currents;
//        dynamicgraph::Vector m_current_offsets;
        dynamicgraph::Vector m_cur_offsets_real;
        dynamicgraph::Vector m_cur_err_integr;

        dynamicgraph::Vector m_dz_coeff;

        dynamicgraph::Vector m_avg_cur_err_pos;
        dynamicgraph::Vector m_avg_cur_err_neg;

        std::vector<std::string>  m_ctrlModes;                /// existing control modes
        std::vector<CtrlMode>     m_jointCtrlModes_current;   /// control mode of the joints
        std::vector<CtrlMode>     m_jointCtrlModes_previous;  /// previous control mode of the joints
        std::vector<int>          m_jointCtrlModesCountDown;  /// counters used for the transition between two ctrl modes

        bool convertStringToCtrlMode(const std::string& name, CtrlMode& cm);
        bool convertJointNameToJointId(const std::string& name, unsigned int& id);
        bool isJointInRange(unsigned int id, double q);
        void updateJointCtrlModesOutputSignal();

      }; // class ControlManager

    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_control_manager_H__
