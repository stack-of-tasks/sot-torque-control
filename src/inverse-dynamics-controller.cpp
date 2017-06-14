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

#include <sot/torque_control/inverse-dynamics-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/utils/metapod-helper.hh>
#include <sot/torque_control/utils/stop-watch.hh>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {
      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace std;
      using namespace metapod;

#define PROFILE_TAU_DES_COMPUTATION "InverseDynamicsController: Desired torques computation "
#define PROFILE_DDQ_DES_COMPUTATION "InverseDynamicsController: Desired ddq computation     "

#define REF_FORCE_SIGNALS m_fRightFootRefSIN << m_fLeftFootRefSIN << \
                          m_fRightHandRefSIN << m_fLeftHandRefSIN
#define FORCE_SIGNALS     m_fRightFootSIN << m_fLeftFootSIN << \
                          m_fRightHandSIN << m_fLeftHandSIN
#define GAIN_SIGNALS      m_KpSIN << m_KdSIN << m_KfSIN << m_KiSIN << m_dynamicsErrorGainSIN
#define REF_JOINT_SIGNALS m_qRefSIN << m_dqRefSIN << m_ddqRefSIN
#define STATE_SIGNALS     m_base6d_encodersSIN << m_jointsVelocitiesSIN << \
                          m_baseAngularVelocitySIN << m_baseAccelerationSIN

#define INPUT_SIGNALS     STATE_SIGNALS << REF_JOINT_SIGNALS << REF_FORCE_SIGNALS << \
                          FORCE_SIGNALS << GAIN_SIGNALS << m_controlledJointsSIN << \
                          m_dynamicsErrorSIN

#define OUTPUT_SIGNALS m_tauDesSOUT << m_ddqDesSOUT << m_qErrorSOUT << \
                       m_tauFFSOUT << m_tauFBSOUT << m_tauFB2SOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef InverseDynamicsController EntityClassName;
      typedef Eigen::Matrix<double,N_JOINTS,1>                     VectorN;
      typedef Eigen::Matrix<double,N_JOINTS+6,1>                   VectorN6;
      typedef Eigen::Matrix<double,3,1>                            Vector3;
      typedef Eigen::Matrix<double,6,1>                            Vector6;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(InverseDynamicsController,
                                         "InverseDynamicsController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      InverseDynamicsController::
          InverseDynamicsController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(base6d_encoders,     dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(jointsVelocities,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(baseAngularVelocity, dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(baseAcceleration,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(qRef,                dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(dqRef,               dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(ddqRef,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kp,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kd,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kf,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Ki,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightFootRef,       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftFootRef,        dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightHandRef,       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftHandRef,        dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightFoot,          dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftFoot,           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightHand,          dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftHand,           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(controlledJoints,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(dynamicsError,       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(dynamicsErrorGain,   dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_OUT(tauDes,             dynamicgraph::Vector, m_tauFBSOUT<<
                                                                  m_tauFFSOUT)
            ,CONSTRUCT_SIGNAL_OUT(tauFF,            dynamicgraph::Vector, STATE_SIGNALS<<
                                                                REF_JOINT_SIGNALS<<
                                                                REF_FORCE_SIGNALS<<
                                                                m_controlledJointsSIN)
            ,CONSTRUCT_SIGNAL_OUT(tauFB,            dynamicgraph::Vector,  STATE_SIGNALS<<
                                                                 REF_JOINT_SIGNALS<<
                                                                 FORCE_SIGNALS<<
                                                                 REF_FORCE_SIGNALS<<
                                                                 GAIN_SIGNALS<<
                                                                 m_controlledJointsSIN)
            ,CONSTRUCT_SIGNAL_OUT(tauFB2,           dynamicgraph::Vector,  STATE_SIGNALS<<
                                                                 REF_JOINT_SIGNALS<<
                                                                 FORCE_SIGNALS<<
                                                                 REF_FORCE_SIGNALS<<
                                                                 GAIN_SIGNALS<<
                                                                 m_controlledJointsSIN)
            ,CONSTRUCT_SIGNAL_OUT(ddqDes,           dynamicgraph::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(qError,           dynamicgraph::Vector, m_base6d_encodersSIN <<
                                                                m_qRefSIN)
            ,m_initSucceeded(false)
            ,m_useFeedforward(true)
            ,m_node_right_foot(boost::fusion::at_c<Hrp2_14::r_ankle>(m_robot.nodes))
            ,m_node_left_foot(boost::fusion::at_c<Hrp2_14::l_ankle>(m_robot.nodes))
            ,m_node_right_hand(boost::fusion::at_c<Hrp2_14::r_wrist>(m_robot.nodes))
            ,m_node_left_hand(boost::fusion::at_c<Hrp2_14::l_wrist>(m_robot.nodes))
      {
        m_J_all = AllJacobian::Zero();

        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init",
                   makeCommandVoid1(*this, &InverseDynamicsController::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Time period in seconds (double)")));
        addCommand("resetForceIntegral",
                   makeCommandVoid0(*this, &InverseDynamicsController::resetForceIntegral,
                                    docCommandVoid0("Reset the force integral.")));

        addCommand("getUseFeedforward",
                   makeDirectGetter(*this,&m_useFeedforward,
                                    docDirectGetter("Whether to use the feedforward terms or not","bool")));
        addCommand("setUseFeedforward",
                   makeDirectSetter(*this, &m_useFeedforward,
                                    docDirectSetter("flag specifying whether Whether to use the feedforward terms or not",
                                                    "bool")));
      }

      void InverseDynamicsController::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(!m_base6d_encodersSIN.isPlugged())
          return SEND_MSG("Init failed: signal base6d_encoders is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_baseAngularVelocitySIN.isPlugged())
          return SEND_MSG("Init failed: signal baseAngularVelocity is not plugged", MSG_TYPE_ERROR);
        if(!m_baseAccelerationSIN.isPlugged())
          return SEND_MSG("Init failed: signal baseAcceleration is not plugged", MSG_TYPE_ERROR);
        if(!m_qRefSIN.isPlugged())
          return SEND_MSG("Init failed: signal qRef is not plugged", MSG_TYPE_ERROR);
        if(!m_dqRefSIN.isPlugged())
          return SEND_MSG("Init failed: signal dqRef is not plugged", MSG_TYPE_ERROR);
        if(!m_ddqRefSIN.isPlugged())
          return SEND_MSG("Init failed: signal ddqRef is not plugged", MSG_TYPE_ERROR);
        if(!m_KpSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
        if(!m_KdSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kd is not plugged", MSG_TYPE_ERROR);
        if(!m_KfSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kf is not plugged", MSG_TYPE_ERROR);
        if(!m_KiSIN.isPlugged())
          return SEND_MSG("Init failed: signal Ki is not plugged", MSG_TYPE_ERROR);
        if(!m_controlledJointsSIN.isPlugged())
          return SEND_MSG("Init failed: signal controlledJoints is not plugged", MSG_TYPE_ERROR);

        m_dt = dt;
        m_ddqDes.setZero(N_JOINTS);
        m_q.setZero();
        m_dq.setZero();
        m_ddq.setZero();
        m_torques.setZero();

        resetForceIntegral();

        // compute transformation from foot frame to sole frame
        // The second argument of the Transform constructor has to be the position
        // of the sole w.r.t. the local frame in local coordinates
        Spatial::RotationMatrixIdentityTpl<double> eye;
        typedef Eigen::Map<const Eigen::Vector3d> CMap3d;
        m_sole_X_RF = Transform(eye, CMap3d(RIGHT_FOOT_SOLE_XYZ));
        m_sole_X_LF = Transform(eye, CMap3d(LEFT_FOOT_SOLE_XYZ));
        m_gripper_X_RH = Transform(eye, CMap3d(RIGHT_HAND_GRIPPER_XYZ));
        m_gripper_X_LH = Transform(eye, CMap3d(LEFT_HAND_GRIPPER_XYZ));

        m_initSucceeded = true;
      }

      void InverseDynamicsController::resetForceIntegral()
      {
        m_f_RF_integral.setZero(6);
        m_f_LF_integral.setZero(6);
        m_f_RH_integral.setZero(6);
        m_f_LH_integral.setZero(6);
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(tauDes, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tauDes before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_TAU_DES_COMPUTATION);
        {
          const VectorN& tauFB  = m_tauFBSOUT(iter);                          // n
          const VectorN& tauFF  = m_tauFFSOUT(iter);                          // n
          const VectorN6& dynamicsError  = m_dynamicsErrorSIN(iter);          // n+6
          const VectorN6& dynamicsErrorGain  = m_dynamicsErrorGainSIN(iter);  // n+6

          if(s.size()!=N_JOINTS)
            s.resize(N_JOINTS);
          if(m_useFeedforward)
	    s = tauFB +
	      tauFF +
	      dynamicsErrorGain.tail<N_JOINTS>().cwiseProduct(dynamicsError.tail<N_JOINTS>());
          else
	    s = tauFB +
	      tauFF +
	      dynamicsErrorGain.tail<N_JOINTS>().cwiseProduct(dynamicsError.tail<N_JOINTS>());
        }
        getProfiler().stop(PROFILE_TAU_DES_COMPUTATION);

        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(ddqDes,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal ddqDes before initialization!");
          return s;
        }

        const VectorN& Kp =m_KpSIN(iter);                   // n
        const VectorN& Kd =m_KdSIN(iter);                   // n
        const VectorN6& q = m_base6d_encodersSIN(iter);     //n+6
        const VectorN& dq =m_jointsVelocitiesSIN(iter);     // n
        const VectorN& qRef = m_qRefSIN(iter);              // n
        const VectorN& dqRef =     m_dqRefSIN(iter);        // n
        const VectorN& ddqRef =    m_ddqRefSIN(iter);       // n

        getProfiler().start(PROFILE_DDQ_DES_COMPUTATION);
        {

          m_ddqDes = ddqRef + Kp.cwiseProduct(qRef-q.tail<N_JOINTS>()) +
                               Kd.cwiseProduct(dqRef-dq);

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
	s = m_ddqDes;
        }
        getProfiler().stop(PROFILE_DDQ_DES_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(qError,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal qError before initialization!",MSG_TYPE_WARNING_STREAM);
          return s;
        }

        const VectorN6& q = m_base6d_encodersSIN(iter);     //n+6
        const VectorN&  qRef = m_qRefSIN(iter);   // n

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
	s = qRef-q.tail<N_JOINTS>();

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(tauFF,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tauFF before initialization!");
          return s;
        }

        const VectorN6& q = m_base6d_encodersSIN(iter);         // n+6
        const VectorN& dq = m_jointsVelocitiesSIN(iter);        // n
        const Vector3& w_b =  m_baseAngularVelocitySIN(iter);   // 3
        const Vector6& dw_b = m_baseAccelerationSIN(iter);      // 6
        const VectorN& ddqRef =    m_ddqRefSIN(iter);           // n
        const VectorN& qMask =     m_controlledJointsSIN(iter); // n

        m_q.head<6>().setZero();
        m_q.tail<N_JOINTS>()   = q.tail<N_JOINTS>();
        m_dq.segment<3>(3)     = w_b;
        m_dq.tail<N_JOINTS>()  = dq;
        m_ddq.head<6>()        = dw_b;
        m_ddq(2)              -= 9.81; // remove gravity acceleration from IMU's measurement
        m_ddq.tail<N_JOINTS>() = ddqRef.cwiseProduct(qMask);

        // compute homogeneous transformations (iX0 and sXp)
        // The iX0's are needed to transform the external wrenches to the base frame
        bcalc< Hrp2_14>::run(m_robot, m_q);

        // if ref force signals are plugged use them
        if(m_fRightFootRefSIN.isPlugged())
        {
          METAPOD_FORCE_FROM_SIGNAL(f_i, m_fRightFootRefSIN(iter));
          f_i = m_sole_X_RF.applyInv(f_i);
          m_node_right_foot.body.Fext = m_node_right_foot.body.iX0.applyInv(f_i);
        }
        if(m_fLeftFootRefSIN.isPlugged())
        {
          METAPOD_FORCE_FROM_SIGNAL(f_i, m_fLeftFootRefSIN(iter));
          f_i = m_sole_X_LF.applyInv(f_i);
          m_node_left_foot.body.Fext = m_node_left_foot.body.iX0.applyInv(f_i);
        }
        if(m_fRightHandRefSIN.isPlugged())
        {
          METAPOD_FORCE_FROM_SIGNAL(f_i, m_fRightHandRefSIN(iter));
          f_i = m_gripper_X_RH.applyInv(f_i);
          m_node_right_hand.body.Fext = m_node_right_hand.body.iX0.applyInv(f_i);
        }
        if(m_fLeftHandRefSIN.isPlugged())
        {
          METAPOD_FORCE_FROM_SIGNAL(f_i, m_fLeftHandRefSIN(iter));
          f_i = m_gripper_X_LH.applyInv(f_i);
          m_node_left_hand.body.Fext = m_node_left_hand.body.iX0.applyInv(f_i);
        }

        rnea< Hrp2_14, true>::run(m_robot, m_q, m_dq, m_ddq);
        getTorques(m_robot, m_torques);

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
	s = m_torques.tail<N_JOINTS>();  // s[i] = m_torques[i+6]

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(tauFB, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tauFB before initialization!");
          return s;
        }

        const VectorN6& q = m_base6d_encodersSIN(iter);         // n+6
        const VectorN& dq =m_jointsVelocitiesSIN(iter);         // n
        const VectorN& Kp =m_KpSIN(iter);                       // n
        const VectorN& Kd =m_KdSIN(iter);                       // n
        const Eigen::Matrix<double, 24, 1>& Kf =m_KfSIN(iter);  // 6*4
        const Eigen::Matrix<double, 24, 1>& Ki =m_KiSIN(iter);  // 6*4
        const VectorN& qRef = m_qRefSIN(iter);                  // n
        const VectorN& dqRef =     m_dqRefSIN(iter);            // n

        VectorN tauFB = Kp.cwiseProduct(qRef-q.tail<N_JOINTS>()) +
	  Kd.cwiseProduct(dqRef-dq);

        /// *** Compute all Jacobians ***
        // jcalc computes homogeneous transformations (sXp) and local velocities (vj)
        jcalc< Hrp2_14 >::run(m_robot, m_q, m_dq);
        // compute Jacobians
        jac< Hrp2_14 >::run(m_robot, m_J_all);
        // extract Jacobians of bodies of interest
        m_J_right_foot = m_J_all.middleRows<6>(6*Hrp2_14::r_ankle);
        m_J_left_foot  = m_J_all.middleRows<6>(6*Hrp2_14::l_ankle);
        m_J_right_hand = m_J_all.middleRows<6>(6*Hrp2_14::r_wrist);
        m_J_left_hand  = m_J_all.middleRows<6>(6*Hrp2_14::l_wrist);
        // map foot Jacobians to soles and hand Jacobians to grippers
        applyTransformToMatrix(m_sole_X_RF, m_J_right_foot);
        applyTransformToMatrix(m_sole_X_LF, m_J_left_foot);
        applyTransformToMatrix(m_gripper_X_RH, m_J_right_hand);
        applyTransformToMatrix(m_gripper_X_LH, m_J_left_hand);

        // if ref force signals are plugged use them
        Eigen::Vector6d e_f;
        m_torques.setZero();
        if(m_fRightFootRefSIN.isPlugged())
        {
          // both f and fRef are expressed in body local coordinates
          // and so is the Jacobian
          const Vector6& fRef = m_fRightFootRefSIN(iter);  // 6
          const Vector6& f =  m_fRightFootSIN(iter);     // 6
          m_f_RF_integral += Ki.head<6>().cwiseProduct(fRef-f);
//          SEND_DEBUG_STREAM_MSG("Force err RF integral: "+toString(m_f_RF_integral.transpose()));
          e_f = Kf.head<6>().cwiseProduct(fRef-f) + m_f_RF_integral;
          m_torques -= m_J_right_foot.topRows<3>().transpose()*e_f.tail<3>();
          m_torques -= m_J_right_foot.bottomRows<3>().transpose()*e_f.head<3>();
        }
        if(m_fLeftFootRefSIN.isPlugged())
        {
          const Vector6& fRef = m_fLeftFootRefSIN(iter);  // 6
          const Vector6& f =  m_fLeftFootSIN(iter);     // 6
          m_f_LF_integral += Ki.segment<6>(6).cwiseProduct(fRef-f);
          e_f = Kf.segment<6>(6).cwiseProduct(fRef-f) + m_f_LF_integral;
          m_torques -= m_J_left_foot.topRows<3>().transpose()*e_f.tail<3>();
          m_torques -= m_J_left_foot.bottomRows<3>().transpose()*e_f.head<3>();
        }
        if(m_fRightHandRefSIN.isPlugged())
        {
          const Vector6& fRef = m_fRightHandRefSIN(iter);  // 6
          const Vector6& f =  m_fRightHandSIN(iter);     // 6
          m_f_RH_integral += Ki.segment<6>(12).cwiseProduct(fRef-f);
          e_f = Kf.segment<6>(12).cwiseProduct(fRef-f) + m_f_RH_integral;
          m_torques -= m_J_right_hand.topRows<3>().transpose()*e_f.tail<3>();
          m_torques -= m_J_right_hand.bottomRows<3>().transpose()*e_f.head<3>();
        }
        if(m_fLeftHandRefSIN.isPlugged())
        {
          const Vector6& fRef = m_fLeftHandRefSIN(iter);  // 6
          const Vector6& f =  m_fLeftHandSIN(iter);     // 6
          m_f_LH_integral += Ki.tail<6>().cwiseProduct(fRef-f);
          e_f = Kf.tail<6>().cwiseProduct(fRef-f) + m_f_LH_integral;
          m_torques -= m_J_left_hand.topRows<3>().transpose()*e_f.tail<3>();
          m_torques -= m_J_left_hand.bottomRows<3>().transpose()*e_f.head<3>();
        }

        tauFB += m_torques.tail<N_JOINTS>();

	if(s.size()!=N_JOINTS)
	  s.resize(N_JOINTS);
	s = tauFB;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(tauFB2,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tauFB2 before initialization!");
          return s;
        }

        const VectorN6& q = m_base6d_encodersSIN(iter);    // n+6
        const VectorN& dq = m_jointsVelocitiesSIN(iter);   // n
        const VectorN& Kp = m_KpSIN(iter); // n
        const VectorN& Kd = m_KdSIN(iter); // n
        const Eigen::Matrix<double,24,1>& Kf = m_KfSIN(iter); // 6*4
        const VectorN& qRef = m_qRefSIN(iter);   // n
        const VectorN& dqRef =     m_dqRefSIN(iter);  // n

        VectorN tauFB = Kp.cwiseProduct(qRef-q.tail<N_JOINTS>()) + Kd.cwiseProduct(dqRef-dq);

        m_q.head<6>().setZero();
        m_q.tail<N_JOINTS>()   = q.tail<N_JOINTS>();
        m_dq.setZero();
        m_ddq.setZero();
        m_ddq(2)              -= 9.81; // remove gravity acceleration

        // compute homogeneous transformations (iX0 and sXp)
        // The iX0's are needed to transform the external wrenches to the base frame
        bcalc< Hrp2_14>::run(m_robot, m_q);
        Spatial::ForceTpl<double> e_f;
        Eigen::Vector6d e_f_vector;

        // if ref force signals are plugged use them
        if(m_fRightFootRefSIN.isPlugged())
        {
          const Vector6& fRef = m_fRightFootRefSIN(iter);  // 6
          const Vector6& f =  m_fRightFootSIN(iter);     // 6
          e_f_vector = Kf.head<6>().cwiseProduct(fRef-f) + m_f_RF_integral;
          e_f = Spatial::ForceTpl<double>(e_f_vector.tail<3>(), e_f_vector.head<3>());
          e_f = m_sole_X_RF.applyInv(e_f);
          m_node_right_foot.body.Fext = m_node_right_foot.body.iX0.applyInv(e_f);
        }
        if(m_fLeftFootRefSIN.isPlugged())
        {
          const Vector6& fRef = m_fLeftFootRefSIN(iter);
          const Vector6& f =  m_fLeftFootSIN(iter);
          e_f_vector = Kf.segment<6>(6).cwiseProduct(fRef-f) + m_f_LF_integral;
          e_f = Spatial::ForceTpl<double>(e_f_vector.tail<3>(), e_f_vector.head<3>());
          e_f = m_sole_X_LF.applyInv(e_f);
          m_node_left_foot.body.Fext = m_node_left_foot.body.iX0.applyInv(e_f);
        }
        if(m_fRightHandRefSIN.isPlugged())
        {
          const Vector6& fRef = m_fRightHandRefSIN(iter);
          const Vector6& f =  m_fRightHandSIN(iter);
          e_f_vector = Kf.segment<6>(12).cwiseProduct(fRef-f) + m_f_RH_integral;
          e_f = Spatial::ForceTpl<double>(e_f_vector.tail<3>(), e_f_vector.head<3>());
          e_f = m_gripper_X_RH.applyInv(e_f);
          m_node_right_hand.body.Fext = m_node_right_hand.body.iX0.applyInv(e_f);
        }
        if(m_fLeftHandRefSIN.isPlugged())
        {
          const Vector6& fRef = m_fLeftHandRefSIN(iter);
          const Vector6& f =  m_fLeftHandSIN(iter);
          e_f_vector = Kf.tail<6>().cwiseProduct(fRef-f) + m_f_LH_integral;
          e_f = Spatial::ForceTpl<double>(e_f_vector.tail<3>(), e_f_vector.head<3>());
          e_f = m_gripper_X_LH.applyInv(e_f);
          m_node_left_hand.body.Fext = m_node_left_hand.body.iX0.applyInv(e_f);
        }

        rnea< Hrp2_14, true>::run(m_robot, m_q, m_dq, m_ddq);
        getTorques(m_robot, m_torques);

        tauFB += m_torques.tail<N_JOINTS>();

	if(s.size()!=N_JOINTS)
	  s.resize(N_JOINTS);
	s = tauFB;

        return s;
      }
      


      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void InverseDynamicsController::display(std::ostream& os) const
      {
        os << "InverseDynamicsController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }


      void InverseDynamicsController::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "sotInverseDynamicsController:\n"
              << "\t -." << std::endl;
          Entity::commandLine(cmdLine, cmdArgs, os);
        }
        else
        {
          Entity::commandLine(cmdLine,cmdArgs,os);
        }
      }
      
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

