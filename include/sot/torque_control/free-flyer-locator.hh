/*
 * Copyright 2017, Thomas Flayols, LAAS-CNRS
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

#ifndef __sot_torque_control_free_flyer_locator_H__
#define __sot_torque_control_free_flyer_locator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (free_flyer_locator_EXPORTS)
#    define SOTFREEFLYERLOCATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTFREEFLYERLOCATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFREEFLYERLOCATOR_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"

/* Metapod */
//~ #include <metapod/models/hrp2_14/hrp2_14.hh>
//~ #include <metapod/algos/rnea.hh>
//~ #include <metapod/algos/jac.hh>
//~ #include <metapod/tools/jcalc.hh>
//~ #include <metapod/tools/bcalc.hh>
//~ #include <metapod/tools/print.hh>
//~ #include <metapod/tools/initconf.hh>

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
//~ #include <pinocchio/algorithm/rnea.hpp>
//~ #include "pinocchio/algorithm/crba.hpp"
namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTFREEFLYERLOCATOR_EXPORT FreeFlyerLocator
	:public::dynamicgraph::Entity
      {
        typedef FreeFlyerLocator EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();
        
      public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        FreeFlyerLocator( const std::string & name );

        void init(const std::string& urdfFile,
		  const std::string & leftFootFrameName,
		  const std::string & rightFootFrameName);

        void resetIntegral();

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(base6d_encoders,            ml::Vector);
        DECLARE_SIGNAL_IN(joint_velocities,           ml::Vector);
        DECLARE_SIGNAL_INNER(kinematics_computations, ml::Vector);
        DECLARE_SIGNAL_OUT(freeflyer_aa,              ml::Vector);  /// freeflyer position with angle axis format
        DECLARE_SIGNAL_OUT(base6dFromFoot_encoders,   ml::Vector);  /// base6d_encoders with base6d in RPY
        DECLARE_SIGNAL_OUT(v,                         ml::Vector);  /// n+6 robot velocities

        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[FreeFlyerLocator-"+name+"] "+msg, t, file, line);
        }

      protected:
        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        se3::Model        m_model;            /// Pinocchio robot model
        se3::Data         *m_data;            /// Pinocchio robot data 
        se3::SE3          m_Mff;               /// SE3 Transform from center of feet to base
        se3::SE3          m_w_M_lf;
        se3::SE3          m_w_M_rf;
        unsigned int      m_right_foot_id;
        unsigned int      m_left_foot_id;
        Eigen::VectorXd   m_q_pin;            /// robot configuration according to pinocchio convention
        Eigen::VectorXd   m_q_sot;            /// robot configuration according to SoT convention
        Eigen::VectorXd   m_v_pin;            /// robot velocities according to pinocchio convention
        Eigen::VectorXd   m_v_sot;            /// robot velocities according to SoT convention

	std::string m_Left_Foot_Frame_Name;
	std::string m_Right_Foot_Frame_Name;
	unsigned int m_nbJoints;
	double m_Right_Foot_Sole_XYZ[3];
      }; // class FreeFlyerLocator
      
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_free_flyer_locator_H__
