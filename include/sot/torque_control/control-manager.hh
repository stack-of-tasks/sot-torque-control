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
#include <sot/torque_control/hrp2-common.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"


namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

/// Number of time step to transition from one ctrl mode to another
#define CTRL_MODE_TRANSITION_TIME_STEP 1000.0

///factor to go from a [-20.0 ; 20.0] Ampers value 
///             to the [-2048 ; 2048] 12bit DAC register
#define FROM_CURRENT_TO_12_BIT_CTRL 102.4

///offset to apply to compensate motor driver dead-zone (+-0.2V -> +-0.4A -> "+-(int)40.96) 
#define DEAD_ZONE_OFFSET 40

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
        ControlManager( const std::string & name );

        void init(const double& dt);

        /* --- SIGNALS --- */
        std::vector<dynamicgraph::SignalPtr<dynamicgraph::Vector,int>*> m_ctrlInputsSIN;
        std::vector<dynamicgraph::Signal<dynamicgraph::Vector,int>*> m_jointsCtrlModesSOUT;

        std::vector<dynamicgraph::SignalPtr<bool,int>*> m_emergencyStopSIN;/// emergency stop inputs. If one is true, control is set to zero forever

        DECLARE_SIGNAL_IN(base6d_encoders,                       dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(dq,                                    dynamicgraph::Vector);  /// Joint velocities; used to compensate for BEMF effect on low level current loop
        DECLARE_SIGNAL_IN(bemfFactor,                            dynamicgraph::Vector);  /// Link betwin velocity and current; to compensate for BEMF effect on low level current loop (in A/rad.s-1)
        DECLARE_SIGNAL_IN(tau,                                   dynamicgraph::Vector);  /// estimated joint torques (using dynamic robot model + F/T sensors)
        DECLARE_SIGNAL_IN(tau_predicted,                         dynamicgraph::Vector);  /// predicted joint torques (using motor model)
        DECLARE_SIGNAL_IN(max_current,                           dynamicgraph::Vector);  /// max current allowed before stopping the controller (in Ampers)
        DECLARE_SIGNAL_IN(max_tau,                               dynamicgraph::Vector);  /// max torque allowed before stopping the controller
        DECLARE_SIGNAL_IN(percentageDriverDeadZoneCompensation,  dynamicgraph::Vector);  /// percentatge in [0;1] of the motor driver dead zone that we should compensate 0 is none, 1 is all of it
        DECLARE_SIGNAL_IN(signWindowsFilterSize,                 dynamicgraph::Vector);  /// windows size to detect changing of control sign (to then apply motor driver dead zone compensation) 0 is no filter. 1,2,3...
        DECLARE_SIGNAL_IN(emergencyStop,                         bool      );  /// emergency stop input. If true, control is set to zero forever 
        DECLARE_SIGNAL_OUT(pwmDes,                               dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(pwmDesSafe,                           dynamicgraph::Vector);  /// same as pwmDes when everything is fine, 0 otherwise //TODO change since pwmDes is now the desired current and pwmDesSafe is the DAC 
        DECLARE_SIGNAL_OUT(signOfControlFiltered,                dynamicgraph::Vector);  /// sign of control filtered (indicating dead zone compensation applyed)
        DECLARE_SIGNAL_OUT(signOfControl,                        dynamicgraph::Vector);  /// sign of control without filtered (indicating what would be the dead zone compensation applyed if no filtering on sign)



        /* --- COMMANDS --- */
        void addCtrlMode(const std::string& name);
        void ctrlModes();
        void getCtrlMode(const std::string& jointName);
        void setCtrlMode(const std::string& jointName, const std::string& ctrlMode);
        void setCtrlMode(const int jid, const CtrlMode& cm);
        void resetProfiler();
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
        bool    m_initSucceeded;    /// true if the entity has been successfully initialized
        double  m_dt;               /// control loop time period
        double  m_maxCurrent;       /// control limit in Ampers
        bool    m_emergency_stop_triggered;  /// true if an emergency condition as been triggered either by an other entity, or by control limit violation
        bool    m_is_first_iter;    /// true at the first iteration, false otherwise

        std::vector<bool>         m_signIsPos;      /// Control sign filtered for deadzone compensation
        std::vector<unsigned int> m_changeSignCpt;  /// Cpt to filter the control sign
        std::vector<unsigned int> m_winSizeAdapt;   /// Variable windows filter size used to be more reactibe if last changing sign event is dating a bit (see graph)
        /*
                        _    _   _________________________    _
           input ______| |__| |_|                         |__| |________
                        __________________________________
           output______|                                  |_____________

        */

        std::vector<std::string>  m_ctrlModes;                /// existing control modes
        std::vector<CtrlMode>     m_jointCtrlModes_current;   /// control mode of the joints
        std::vector<CtrlMode>     m_jointCtrlModes_previous;  /// previous control mode of the joints
        std::vector<int>          m_jointCtrlModesCountDown;  /// counters used for the transition between two ctrl modes

        std::vector<bool>         m_currentWarningZone;      /// true if desired current > 0.8*maxCurrent

        bool convertStringToCtrlMode(const std::string& name, CtrlMode& cm);
        bool convertJointNameToJointId(const std::string& name, unsigned int& id);
        bool isJointInRange(unsigned int id, double q);
        void updateJointCtrlModesOutputSignal();

      }; // class ControlManager

    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_control_manager_H__
