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

#ifndef __sot_torque_control_JointTorqueController_H__
#define __sot_torque_control_JointTorqueController_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (joint_torque_controller_EXPORTS)
#    define SOTJOINTTORQUECONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define SOTJOINTTORQUECONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTJOINTTORQUECONTROLLER_EXPORT
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

/*Motor model*/
#include <sot/torque_control/motor-model.hh>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /**
        * This Entity takes as inputs the estimated joints' positions,
        * velocities, accelerations and torques and it computes the desired
        * current to send to the motors board in order to track the
        * desired joints torques. Most of the input of this entity are
        * computed by the entity ForceTorqueEstimator.
        *
        * QUICK START
        * Create the entity, plug all the input signals, call the init method
        * specifying the control-loop time step. For instance:
        *   jtc = JointTorqueController("jtc");
        *   plug(estimator.jointsPositions,     jtc.jointsPositions);
        *   plug(estimator.jointsVelocities,    jtc.jointsVelocities);
        *   plug(estimator.jointsAccelerations, jtc.jointsAccelerations);
        *   plug(estimator.jointsTorques,       jtc.jointsTorques);
        *   jtc.KpTorque.value = N_DOF*(10.0,);
        *   jtc.KiTorque.value = N_DOF*(0.01,);
        *   jtc.k_tau.value = ...
        *   jtc.k_v.value = ...
        *   jtc.init(dt);
        *
        * DETAILS
        * To do...
        */
      class SOTJOINTTORQUECONTROLLER_EXPORT JointTorqueController
          :public ::dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:  /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(base6d_encoders,        dynamicgraph::Vector);      /// base position  + joints positions
        DECLARE_SIGNAL_IN(pwm,                    dynamicgraph::Vector);      /// pwm commanded to the motors (used for debug only)
        DECLARE_SIGNAL_IN(jointsVelocities,       dynamicgraph::Vector);      /// dq
        DECLARE_SIGNAL_IN(jointsAccelerations,    dynamicgraph::Vector);      /// ddq
        DECLARE_SIGNAL_IN(jointsTorques,          dynamicgraph::Vector);      /// estimated joints torques tau
        DECLARE_SIGNAL_IN(jointsTorquesDesired,   dynamicgraph::Vector);      /// desired joints torques tauDes
        DECLARE_SIGNAL_IN(measuredCurrent,        dynamicgraph::Vector);      /// measured current in amps
        DECLARE_SIGNAL_IN(KpTorque,               dynamicgraph::Vector);      /// proportional gain for torque feedback controller
        DECLARE_SIGNAL_IN(KiTorque,               dynamicgraph::Vector);      /// integral gain for torque feedback controller
        DECLARE_SIGNAL_IN(KpCurrent,              dynamicgraph::Vector);      /// proportional gain for current feedback controller
        DECLARE_SIGNAL_IN(KiCurrent,              dynamicgraph::Vector);      /// integral gain for current feedback controller
 
//        DECLARE_SIGNAL_IN(dq_threshold,           dynamicgraph::Vector);      /// velocity sign threshold
//        DECLARE_SIGNAL_IN(ddq_threshold,          dynamicgraph::Vector);      /// acceleration sign threshold
//        DECLARE_SIGNAL_IN(activeJoints,           dynamicgraph::Vector);      /// mask with 1 for (torque) controlled joints and 0 for position controlled joints

        /// parameters for the linear model
        DECLARE_SIGNAL_IN(k_tau,                dynamicgraph::Vector); //to be del
        DECLARE_SIGNAL_IN(k_v,                  dynamicgraph::Vector); //to be del
        DECLARE_SIGNAL_IN(frictionCompensationPercentage, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(motorParameterKt_p, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(motorParameterKt_n, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(motorParameterKf_p, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(motorParameterKf_n, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(motorParameterKv_p, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(motorParameterKv_n, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(motorParameterKa_p, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(motorParameterKa_n, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(polySignDq,         dynamicgraph::Vector);
        /// input from inverse dynamics controller for computing
        /// monitoring signals deltaQ_ff, deltaQ_fb
        DECLARE_SIGNAL_IN(tauFF,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(tauFB,                dynamicgraph::Vector);

        // Parameters of the piece-wise linear function f, which
        // describes the relationship between joints torques tau and
        // position error deltaQ
//        DECLARE_SIGNAL_IN(f_k1p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_k2p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_k3p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_k1n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_k2n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_k3n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_q1p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_q2p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_q3p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_q1n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_q2n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_q3n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_tau1p,              dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_tau2p,              dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_tau1n,              dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(f_tau2n,              dynamicgraph::Vector);

        // Parameters of the piece-wise linear function g, which
        // describes the relationship between joints velocities dq and
        // position error deltaQ
//        DECLARE_SIGNAL_IN(g_k1p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_k2p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_k3p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_k1n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_k2n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_k3n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_q1p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_q2p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_q3p,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_q1n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_q2n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_q3n,                dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_dq1p,               dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_dq2p,               dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_dq1n,               dynamicgraph::Vector);
//        DECLARE_SIGNAL_IN(g_dq2n,               dynamicgraph::Vector);

        DECLARE_SIGNAL_OUT(desiredCurrent,  dynamicgraph::Vector);  /// Desired current 
        DECLARE_SIGNAL_OUT(controlCurrent,  dynamicgraph::Vector);  /// current command to send to the motor drivers

        /// Signals for monitoring the behavior of the entity
        DECLARE_SIGNAL_OUT(predictedJointsTorques,  dynamicgraph::Vector);  /// k_tau^{-1}*(delta_q - k_v*dq)
        DECLARE_SIGNAL_OUT(predictedPwm,            dynamicgraph::Vector);  /// k_tau*tau + k_v*dq
        DECLARE_SIGNAL_OUT(pwm,                     dynamicgraph::Vector);  /// pwm, just for comparison with predictedPwm
        DECLARE_SIGNAL_OUT(predictedPwm_tau,        dynamicgraph::Vector);  /// k_tau*tau
/// pwm = k_tau*(tauFF+tauFB + kp*(tauFF+tauFB-tauEst)) + k_v*dq
/// pwm = k_tau*(tauFF+tauFB + kp*(M*ddqRef+h-JT*fRef + Ks*e_q+Kd*de_q-JT*Kf*e_f - M*ddqRef-h+JT*f)) + k_v*dq
/// pwm = k_tau*(tauFF+tauFB + kp*(-JT*e_f + tauFB)) + k_v*dq
/// pwm = k_tau*tauFF + k_tau*(tauFB +k_p*tauFB - kp*JT*e_f) + k_v*dq
/// pwm = k_tau*tauFF + k_tau*(tauFB +k_p*(tauDes - tau)) + k_v*dq
        DECLARE_SIGNAL_OUT(pwm_ff,               dynamicgraph::Vector);  /// k_tau*tauFF,                       part of pwm due to tauFF
        DECLARE_SIGNAL_OUT(pwm_fb,               dynamicgraph::Vector);  /// k_tau*(tauFB +k_p*(tauDes - tau)), part of pwm due to tauFB
        DECLARE_SIGNAL_OUT(pwm_friction,         dynamicgraph::Vector);  /// k_v*dq,                            part of pwm due to friction compensation
        DECLARE_SIGNAL_OUT(smoothSignDq,         dynamicgraph::Vector); /// smooth approximation of sign(dq)
//        DECLARE_SIGNAL_OUT(smoothSignDdq,           dynamicgraph::Vector);  /// smooth approximation of sign(ddq)

      protected:
        MotorModel motorModel;
        bool m_firstIter;
        double m_dt; /// timestep of the controller
        Eigen::VectorXd m_tau_star;
        Eigen::VectorXd m_current_star;
        Eigen::VectorXd m_f;
        Eigen::VectorXd m_g;
        Eigen::VectorXd m_current_des;
        Eigen::VectorXd m_tauErrIntegral; /// integral of the torque error
        Eigen::VectorXd m_currentErrIntegral; /// integral of the current error
        Eigen::VectorXd m_qDes_for_position_controlled_joints;
        Eigen::Array<int, N_JOINTS, 1> m_activeJoints;
        std::string m_activeJointsString;

//        void compute_f(const Eigen::VectorXd &tau, Eigen::const_SigVectorXd &dq, Eigen::const_SigVectorXd &dq_thr, int iter, Eigen::VectorXd &f);
//        void compute_g(Eigen::const_SigVectorXd &dq, Eigen::const_SigVectorXd &ddq, Eigen::const_SigVectorXd &ddq_thr, int iter, Eigen::VectorXd &g);

        /** Compute a piece-wise linear function composed by three segments:
          * if x<x1,    y = a1*x+b1
          * if x1<x<x2, y = a2*x+b2
          * if x>x2,    y = a3*x+b3
          */
        double compute_piecewise_linear(const double &x, const double &a1, const double &a2, const double &a3, const double &b1,
                                        const double &b2, const double &b3, const double &x1, const double &x2) const;

        bool convertJointNameToJointId(const std::string& name, unsigned int& id);

        void updateActiveJointsString()
        {
          std::stringstream ss;
          unsigned int i;
          for(i=0; i<N_JOINTS-1; i++)
            ss << JointUtil::get_name_from_id(i)<<" "<<toString(m_activeJoints[i])<<", ";
          ss << JointUtil::get_name_from_id(i)<<" "<<toString(m_activeJoints[i]);
          m_activeJointsString = ss.str();
        }

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[JointTorqueController-"+name+"] "+msg, t, file, line);
        }

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** --- CONSTRUCTOR ---- */
        JointTorqueController( const std::string & name );

        /** Initialize the JointTorqueController.
         * @param timestep Period (in seconds) after which the sensors' data are updated.
         */
        void init(const double &timestep);

        void activate(const std::string& jointName);
        void deactivate(const std::string& jointName);

      public: /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

      }; // class JointTorqueController

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_JointTorqueController_H__
