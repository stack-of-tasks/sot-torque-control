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

#ifndef __sot_torque_control_admittance_controller_H__
#define __sot_torque_control_admittance_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (sot_admittance_controller_EXPORTS)
#    define SOTADMITTANCECONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define SOTADMITTANCECONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTADMITTANCECONTROLLER_EXPORT
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

      Eigen::VectorXd svdSolveWithDamping(const Eigen::JacobiSVD<Eigen::MatrixXd>& A, const Eigen::VectorXd &b, double damping);

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTADMITTANCECONTROLLER_EXPORT AdmittanceController
	:public::dynamicgraph::Entity
      {
        typedef AdmittanceController EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();
        
      public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        AdmittanceController( const std::string & name );

        void init(const double& dt);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(base6d_encoders,  ml::Vector);
        DECLARE_SIGNAL_IN(jointsVelocities, ml::Vector);
        DECLARE_SIGNAL_IN(Kd,               ml::Vector);
        DECLARE_SIGNAL_IN(Kf,               ml::Vector);
        DECLARE_SIGNAL_IN(fRightFootRef,    ml::Vector); /// 6d reference force
        DECLARE_SIGNAL_IN(fLeftFootRef,     ml::Vector); /// 6d reference force
        DECLARE_SIGNAL_IN(fRightHandRef,    ml::Vector); /// 6d reference force
        DECLARE_SIGNAL_IN(fLeftHandRef,     ml::Vector); /// 6d reference force
        DECLARE_SIGNAL_IN(fRightFoot,       ml::Vector); /// 6d estimated force
        DECLARE_SIGNAL_IN(fLeftFoot,        ml::Vector); /// 6d estimated force
        DECLARE_SIGNAL_IN(fRightHand,       ml::Vector); /// 6d estimated force
        DECLARE_SIGNAL_IN(fLeftHand,        ml::Vector); /// 6d estimated force
        DECLARE_SIGNAL_IN(controlledJoints, ml::Vector); /// mask with 1 for controlled joints, 0 otherwise
        DECLARE_SIGNAL_IN(damping,          ml::Vector); /// damping factors used for the 4 end-effectors

        DECLARE_SIGNAL_OUT(qDes,              ml::Vector);  /// integral of dqDes
        // DEBUG SIGNALS
        DECLARE_SIGNAL_OUT(dqDes,             ml::Vector);  /// dqDes = J^+ * Kf * (fRef-f)
        DECLARE_SIGNAL_OUT(fRightFootError,   ml::Vector);  /// fRef-f
        DECLARE_SIGNAL_OUT(fLeftFootError,    ml::Vector);  /// fRef-f
        DECLARE_SIGNAL_OUT(fRightHandError,   ml::Vector);  /// fRef-f
        DECLARE_SIGNAL_OUT(fLeftHandError,    ml::Vector);  /// fRef-f


        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[AdmittanceController-"+name+"] "+msg, t, file, line);
        }
        
      protected:
        Eigen::VectorXd   m_qDes;             /// desired joint positions
        Eigen::VectorXd   m_dqDes;            /// desired joint velocities
        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        bool              m_firstIter;        /// true only at the first iteration
        bool              m_useJacobianTranspose; /// if true it uses the Jacobian transpose rather than the pseudoinverse
        double            m_dt;               /// control loop time period

        /// robot geometric/inertial data
        typedef metapod::hrp2_14<double> Hrp2_14;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::r_ankle>::type      RightFootNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::l_ankle>::type      LeftFootNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::r_wrist>::type      RightHandNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::l_wrist>::type      LeftHandNode;
        typedef Eigen::Matrix<double, 6*Hrp2_14::NBBODIES, Hrp2_14::NBDOF>  AllJacobian;
        typedef Eigen::Matrix<double, 6, 6>                    FootJacobian;
        typedef Eigen::Matrix<double, 6, 9>                    HandJacobian;
        typedef metapod::Spatial::TransformT<double, metapod::Spatial::RotationMatrixIdentityTpl<double> > Transform;
        // nodes corresponding to the four end-effectors
        RightFootNode&  m_node_right_foot;
        LeftFootNode&   m_node_left_foot;
        RightHandNode&  m_node_right_hand;
        LeftHandNode&   m_node_left_hand;

        AllJacobian m_J_all;
        FootJacobian m_J_right_foot;
        FootJacobian m_J_left_foot;
        HandJacobian m_J_right_hand;
        HandJacobian m_J_left_hand;

        Eigen::JacobiSVD<Eigen::MatrixXd> m_J_right_foot_svd; /// svd of the jacobian matrix
        Eigen::JacobiSVD<Eigen::MatrixXd> m_J_left_foot_svd;  /// svd of the jacobian matrix
        Eigen::JacobiSVD<Eigen::MatrixXd> m_J_right_hand_svd; /// svd of the jacobian matrix
        Eigen::JacobiSVD<Eigen::MatrixXd> m_J_left_hand_svd;  /// svd of the jacobian matrix

        /// Transformation from foot frame to sole frame
        Transform m_sole_X_RF;
        Transform m_sole_X_LF;
        /// Transformation from hand frame to gripper frame
        Transform m_gripper_X_RH;
        Transform m_gripper_X_LH;

        Hrp2_14 m_robot;
        Hrp2_14::confVector m_q, m_dq;

      }; // class AdmittanceController
      
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_admittance_controller_H__
