/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
 *
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


#include <map>
#include "boost/assign.hpp"


#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <tsid/robots/robot-wrapper.hpp>
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

/// Number of time step to transition from one ctrl mode to another
#define CTRL_MODE_TRANSITION_TIME_STEP 1000.0

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
        void init(const double & dt,
                  const std::string & urdfFile,
                  const std::string & robotRef);

        /* --- SIGNALS --- */
        std::vector<dynamicgraph::SignalPtr<dynamicgraph::Vector,int>*> m_ctrlInputsSIN;
        std::vector< dynamicgraph::SignalPtr<bool,int>* >               m_emergencyStopSIN; /// emergency stop inputs. If one is true, control is set to zero forever
        std::vector<dynamicgraph::Signal<dynamicgraph::Vector,int>*>    m_jointsCtrlModesSOUT;

        DECLARE_SIGNAL_IN(i_measured,                            dynamicgraph::Vector);  /// motor currents
        DECLARE_SIGNAL_IN(tau,                                   dynamicgraph::Vector);  /// estimated joint torques (using dynamic robot model + F/T sensors)
        DECLARE_SIGNAL_IN(tau_predicted,                         dynamicgraph::Vector);  /// predicted joint torques (using motor model)
        DECLARE_SIGNAL_IN(i_max,                                 dynamicgraph::Vector);  /// max current allowed before stopping the controller (in Ampers)
        DECLARE_SIGNAL_IN(u_max,                                 dynamicgraph::Vector);  /// max desired current allowed before stopping the controller (in Ampers)
        DECLARE_SIGNAL_IN(tau_max,                               dynamicgraph::Vector);  /// max torque allowed before stopping the controller

        DECLARE_SIGNAL_OUT(u,                                    dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(u_safe,                               dynamicgraph::Vector);  /// same as u when everything is fine, 0 otherwise

        /* --- COMMANDS --- */

        /// Commands related to the control mode.
        void addCtrlMode(const std::string& name);
        void ctrlModes();
        void getCtrlMode(const std::string& jointName);
        void setCtrlMode(const std::string& jointName, const std::string& ctrlMode);
        void setCtrlMode(const int jid, const CtrlMode& cm);

        void resetProfiler();

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

        /// Commands related to HandUtil
        void setHandFrameName(const std::string &, const std::string &);

        /// Set the mapping between urdf and sot.
        void setJoints(const dynamicgraph::Vector &);

        void setStreamPrintPeriod(const double & s);
        void setSleepTime(const double &seconds);
        void addEmergencyStopSIN(const std::string& name);

        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
	  Entity::sendMsg("[ControlManager-"+name+"] "+msg, t, file, line);
        }

      protected:
        RobotUtilShrPtr               m_robot_util;
        tsid::robots::RobotWrapper *  m_robot;
        bool    m_initSucceeded;    /// true if the entity has been successfully initialized
        double  m_dt;               /// control loop time period
        bool    m_emergency_stop_triggered;  /// true if an emergency condition as been triggered either by an other entity, or by control limit violation
        bool    m_is_first_iter;    /// true at the first iteration, false otherwise
        int     m_iter;
        double  m_sleep_time;       /// time to sleep at every iteration (to slow down simulation)

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
