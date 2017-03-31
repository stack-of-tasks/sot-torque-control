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

#ifndef __sot_torque_control_inverse_dynamics_controller_H__
#define __sot_torque_control_inverse_dynamics_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (joint_position_controller_EXPORTS)
#    define SOTINVERSEDYNAMICSCONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define SOTINVERSEDYNAMICSCONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTINVERSEDYNAMICSCONTROLLER_EXPORT
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

/* Metapod */
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <metapod/algos/rnea.hh>
#include <metapod/algos/jac.hh>
#include <metapod/tools/jcalc.hh>
#include <metapod/tools/bcalc.hh>
#include <metapod/tools/print.hh>
#include <metapod/tools/initconf.hh>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTINVERSEDYNAMICSCONTROLLER_EXPORT InverseDynamicsController
	:public::dynamicgraph::Entity
      {
        typedef InverseDynamicsController EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();
        
      public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        InverseDynamicsController( const std::string & name );

        void init(const double& dt);

        void resetForceIntegral();

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(base6d_encoders,  ml::Vector);
        DECLARE_SIGNAL_IN(jointsVelocities, ml::Vector);
        DECLARE_SIGNAL_IN(baseAngularVelocity, ml::Vector);
        DECLARE_SIGNAL_IN(baseAcceleration, ml::Vector);
        DECLARE_SIGNAL_IN(qRef,             ml::Vector);
        DECLARE_SIGNAL_IN(dqRef,            ml::Vector);
        DECLARE_SIGNAL_IN(ddqRef,           ml::Vector);
        DECLARE_SIGNAL_IN(Kp,               ml::Vector);  /// joint proportional gains
        DECLARE_SIGNAL_IN(Kd,               ml::Vector);  /// joint derivative gains
        DECLARE_SIGNAL_IN(Kf,               ml::Vector);  /// force proportional gains
        DECLARE_SIGNAL_IN(Ki,               ml::Vector);  /// force integral gains
        DECLARE_SIGNAL_IN(fRightFootRef,    ml::Vector); /// right sole 6d reference force
        DECLARE_SIGNAL_IN(fLeftFootRef,     ml::Vector); /// left sole 6d reference force
        DECLARE_SIGNAL_IN(fRightHandRef,    ml::Vector); /// right gripper 6d reference force
        DECLARE_SIGNAL_IN(fLeftHandRef,     ml::Vector); /// left gripper 6d reference force
        DECLARE_SIGNAL_IN(fRightFoot,       ml::Vector); /// right sole 6d estimated force
        DECLARE_SIGNAL_IN(fLeftFoot,        ml::Vector); /// left sole 6d estimated force
        DECLARE_SIGNAL_IN(fRightHand,       ml::Vector); /// right gripper 6d estimated force
        DECLARE_SIGNAL_IN(fLeftHand,        ml::Vector); /// left gripper 6d estimated force
        DECLARE_SIGNAL_IN(controlledJoints, ml::Vector); /// mask with 1 for controlled joints, 0 otherwise
        DECLARE_SIGNAL_IN(dynamicsError,    ml::Vector); /// estimated error of the robot dynamic model (n+6)
        DECLARE_SIGNAL_IN(dynamicsErrorGain,ml::Vector); /// gain multiplying the dynamics error (n+6)

        DECLARE_SIGNAL_OUT(tauDes,      ml::Vector);  /// M*ddqRef + h - J^T*(fRef+Kf*e_f) + Kp*e_q + Kd*de_q
        // DEBUG SIGNALS
        DECLARE_SIGNAL_OUT(tauFF,       ml::Vector);  /// M*ddqRef + h - J^T*fRef
        DECLARE_SIGNAL_OUT(tauFB,       ml::Vector);  /// Kp*(qRef-q) + Kd*(dqRef-dq) - J^T*Kf*e_f
        DECLARE_SIGNAL_OUT(tauFB2,      ml::Vector);  /// same thing but computed differently (just for debug)
        DECLARE_SIGNAL_OUT(ddqDes,      ml::Vector);  /// ddqRef + Kp*(qRef-q) + Kd*(dqRef-dq)
        DECLARE_SIGNAL_OUT(qError,      ml::Vector);  /// qRef-q


        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[InverseDynamicsController-"+name+"] "+msg, t, file, line);
        }
        
      protected:
        Eigen::VectorXd   m_ddqDes;
        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        bool              m_useFeedforward;   /// if true it uses the feedforward, otherwise it does not
        double            m_dt;               /// control loop time period

        /// Integral of the force tracking errors
        Eigen::VectorXd   m_f_RF_integral;
        Eigen::VectorXd   m_f_LF_integral;
        Eigen::VectorXd   m_f_RH_integral;
        Eigen::VectorXd   m_f_LH_integral;


        /// robot geometric/inertial data
        typedef metapod::hrp2_14<double> Hrp2_14;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::r_ankle>::type      RightFootNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::l_ankle>::type      LeftFootNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::r_wrist>::type      RightHandNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::l_wrist>::type      LeftHandNode;
        typedef Eigen::Matrix<double, 6*Hrp2_14::NBBODIES, Hrp2_14::NBDOF>  AllJacobian;
        typedef Eigen::Matrix<double, 6, Hrp2_14::NBDOF>                    Jacobian;
        typedef metapod::Spatial::TransformT<double, metapod::Spatial::RotationMatrixIdentityTpl<double> > Transform;
        // nodes corresponding to the four end-effectors
        RightFootNode&  m_node_right_foot;
        LeftFootNode&   m_node_left_foot;
        RightHandNode&  m_node_right_hand;
        LeftHandNode&   m_node_left_hand;

        AllJacobian m_J_all;
        Jacobian m_J_right_foot;
        Jacobian m_J_left_foot;
        Jacobian m_J_right_hand;
        Jacobian m_J_left_hand;

        /// Transformation from foot frame to sole frame
        Transform m_sole_X_RF;
        Transform m_sole_X_LF;
        /// Transformation from hand frame to gripper frame
        Transform m_gripper_X_RH;
        Transform m_gripper_X_LH;

        Hrp2_14 m_robot;
        Hrp2_14::confVector m_q, m_dq, m_ddq;
        Hrp2_14::confVector m_torques;

      }; // class InverseDynamicsController
      
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_inverse_dynamics_controller_H__
