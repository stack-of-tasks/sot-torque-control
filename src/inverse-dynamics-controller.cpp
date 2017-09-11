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
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;
      using namespace std;
      using namespace metapod;
//Size to be aligned                "-------------------------------------------------------"
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

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(InverseDynamicsController,
                                         "InverseDynamicsController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      InverseDynamicsController::
          InverseDynamicsController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(base6d_encoders,     ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(jointsVelocities,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(baseAngularVelocity, ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(baseAcceleration,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(qRef,                ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(dqRef,               ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(ddqRef,              ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kp,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kd,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kf,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(Ki,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightFootRef,       ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftFootRef,        ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightHandRef,       ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftHandRef,        ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightFoot,          ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftFoot,           ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightHand,          ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftHand,           ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(controlledJoints,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(dynamicsError,       ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(dynamicsErrorGain,   ml::Vector)
            ,CONSTRUCT_SIGNAL_OUT(tauDes,             ml::Vector, m_tauFBSOUT<<
                                                                  m_tauFFSOUT)
            ,CONSTRUCT_SIGNAL_OUT(tauFF,            ml::Vector, STATE_SIGNALS<<
                                                                REF_JOINT_SIGNALS<<
                                                                REF_FORCE_SIGNALS<<
                                                                m_controlledJointsSIN)
            ,CONSTRUCT_SIGNAL_OUT(tauFB,            ml::Vector,  STATE_SIGNALS<<
                                                                 REF_JOINT_SIGNALS<<
                                                                 FORCE_SIGNALS<<
                                                                 REF_FORCE_SIGNALS<<
                                                                 GAIN_SIGNALS<<
                                                                 m_controlledJointsSIN)
            ,CONSTRUCT_SIGNAL_OUT(tauFB2,           ml::Vector,  STATE_SIGNALS<<
                                                                 REF_JOINT_SIGNALS<<
                                                                 FORCE_SIGNALS<<
                                                                 REF_FORCE_SIGNALS<<
                                                                 GAIN_SIGNALS<<
                                                                 m_controlledJointsSIN)
            ,CONSTRUCT_SIGNAL_OUT(ddqDes,           ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(qError,           ml::Vector, m_base6d_encodersSIN <<
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

      DEFINE_SIGNAL_OUT_FUNCTION(tauDes, ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tauDes before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_TAU_DES_COMPUTATION);
        {
          const ml::Vector& tauFB  = m_tauFBSOUT(iter); // n
          const ml::Vector& tauFF  = m_tauFFSOUT(iter); // n
          const ml::Vector& dynamicsError  = m_dynamicsErrorSIN(iter); // n+6
          const ml::Vector& dynamicsErrorGain  = m_dynamicsErrorGainSIN(iter); // n+6

          if(s.size()!=N_JOINTS)
            s.resize(N_JOINTS);
          if(m_useFeedforward)
          {
            for(unsigned int i=0; i<N_JOINTS; i++)
              s(i) = tauFB(i) + tauFF(i) + dynamicsErrorGain(6+i)*dynamicsError(6+i);
          }
          else
          {
            for(unsigned int i=0; i<N_JOINTS; i++)
              s(i) = tauFB(i) + dynamicsErrorGain(6+i)*dynamicsError(6+i);
          }
        }
        getProfiler().stop(PROFILE_TAU_DES_COMPUTATION);

        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(ddqDes,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal ddqDes before initialization!");
          return s;
        }

        EIGEN_CONST_VECTOR_FROM_SIGNAL(Kp,          m_KpSIN(iter)); // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(Kd,          m_KdSIN(iter)); // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(q,           m_base6d_encodersSIN(iter));     //n+6
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,          m_jointsVelocitiesSIN(iter));     // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(qRef,        m_qRefSIN(iter));   // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dqRef,       m_dqRefSIN(iter));  // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(ddqRef,      m_ddqRefSIN(iter)); // n

        getProfiler().start(PROFILE_DDQ_DES_COMPUTATION);
        {
          assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
          assert(dq.size()==N_JOINTS      && "Unexpected size of signal dq");
          assert(qRef.size()==N_JOINTS    && "Unexpected size of signal qRef");
          assert(dqRef.size()==N_JOINTS   && "Unexpected size of signal dqRef");
          assert(ddqRef.size()==N_JOINTS  && "Unexpected size of signal ddqRef");
          assert(Kp.size()==N_JOINTS      && "Unexpected size of signal Kd");
          assert(Kd.size()==N_JOINTS      && "Unexpected size of signal Kd");

          m_ddqDes = ddqRef + Kp.cwiseProduct(qRef-q.tail<N_JOINTS>()) +
                               Kd.cwiseProduct(dqRef-dq);

          EIGEN_VECTOR_TO_VECTOR(m_ddqDes,s);
        }
        getProfiler().stop(PROFILE_DDQ_DES_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(qError,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal qError before initialization!",MSG_TYPE_WARNING_STREAM);
          return s;
        }

        EIGEN_CONST_VECTOR_FROM_SIGNAL(q,           m_base6d_encodersSIN(iter));     //n+6
        EIGEN_CONST_VECTOR_FROM_SIGNAL(qRef,        m_qRefSIN(iter));   // n
        assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
        assert(qRef.size()==N_JOINTS    && "Unexpected size of signal qRef");

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(unsigned int i=0; i<N_JOINTS; i++)
          s(i)= qRef[i]-q(6+i);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(tauFF,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tauFF before initialization!");
          return s;
        }

        EIGEN_CONST_VECTOR_FROM_SIGNAL(q,           m_base6d_encodersSIN(iter));    // n+6
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,          m_jointsVelocitiesSIN(iter));   // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(w_b,         m_baseAngularVelocitySIN(iter));   // 3
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dw_b,        m_baseAccelerationSIN(iter));   // 6
        EIGEN_CONST_VECTOR_FROM_SIGNAL(ddqRef,      m_ddqRefSIN(iter));             // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(qMask,       m_controlledJointsSIN(iter));   // n
        assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
        assert(dq.size()==N_JOINTS      && "Unexpected size of signal dq");
        assert(w_b.size()==3            && "Unexpected size of signal base angular velocity");
        assert(dw_b.size()==6           && "Unexpected size of signal base acceleration");
        assert(ddqRef.size()==N_JOINTS  && "Unexpected size of signal ddqRef");
        assert(qMask.size()==N_JOINTS   && "Unexpected size of signal controlledJoints");

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
        COPY_SHIFTED_VECTOR_TO_VECTOR(m_torques,s,6); // s[i] = m_torques[i+6]

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(tauFB,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tauFB before initialization!");
          return s;
        }

        EIGEN_CONST_VECTOR_FROM_SIGNAL(q,           m_base6d_encodersSIN(iter));    // n+6
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,          m_jointsVelocitiesSIN(iter));   // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(Kp,          m_KpSIN(iter)); // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(Kd,          m_KdSIN(iter)); // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(Kf,          m_KfSIN(iter)); // 6*4
        EIGEN_CONST_VECTOR_FROM_SIGNAL(Ki,          m_KiSIN(iter)); // 6*4
        EIGEN_CONST_VECTOR_FROM_SIGNAL(qRef,        m_qRefSIN(iter));   // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dqRef,       m_dqRefSIN(iter));  // n

        assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
        assert(dq.size()==N_JOINTS      && "Unexpected size of signal dq");
        assert(Kf.size()==6*4           && "Unexpected size of signal Kf");
        assert(Kp.size()==N_JOINTS      && "Unexpected size of signal Kp");
        assert(Kd.size()==N_JOINTS      && "Unexpected size of signal Kd");
        assert(qRef.size()==N_JOINTS    && "Unexpected size of signal qRef");
        assert(dqRef.size()==N_JOINTS   && "Unexpected size of signal dqRef");

        Eigen::Matrix<double,N_JOINTS,1> tauFB = Kp.cwiseProduct(qRef-q.tail<N_JOINTS>()) +
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
          EIGEN_CONST_VECTOR_FROM_SIGNAL(fRef, m_fRightFootRefSIN(iter));  // 6
          EIGEN_CONST_VECTOR_FROM_SIGNAL(f,    m_fRightFootSIN(iter));     // 6
          m_f_RF_integral += Ki.head<6>().cwiseProduct(fRef-f);
//          SEND_DEBUG_STREAM_MSG("Force err RF integral: "+toString(m_f_RF_integral.transpose()));
          e_f = Kf.head<6>().cwiseProduct(fRef-f) + m_f_RF_integral;
          m_torques -= m_J_right_foot.topRows<3>().transpose()*e_f.tail<3>();
          m_torques -= m_J_right_foot.bottomRows<3>().transpose()*e_f.head<3>();
        }
        if(m_fLeftFootRefSIN.isPlugged())
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(fRef, m_fLeftFootRefSIN(iter));  // 6
          EIGEN_CONST_VECTOR_FROM_SIGNAL(f,    m_fLeftFootSIN(iter));     // 6
          m_f_LF_integral += Ki.segment<6>(6).cwiseProduct(fRef-f);
          e_f = Kf.segment<6>(6).cwiseProduct(fRef-f) + m_f_LF_integral;
          m_torques -= m_J_left_foot.topRows<3>().transpose()*e_f.tail<3>();
          m_torques -= m_J_left_foot.bottomRows<3>().transpose()*e_f.head<3>();
        }
        if(m_fRightHandRefSIN.isPlugged())
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(fRef, m_fRightHandRefSIN(iter));  // 6
          EIGEN_CONST_VECTOR_FROM_SIGNAL(f,    m_fRightHandSIN(iter));     // 6
          m_f_RH_integral += Ki.segment<6>(12).cwiseProduct(fRef-f);
          e_f = Kf.segment<6>(12).cwiseProduct(fRef-f) + m_f_RH_integral;
          m_torques -= m_J_right_hand.topRows<3>().transpose()*e_f.tail<3>();
          m_torques -= m_J_right_hand.bottomRows<3>().transpose()*e_f.head<3>();
        }
        if(m_fLeftHandRefSIN.isPlugged())
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(fRef, m_fLeftHandRefSIN(iter));  // 6
          EIGEN_CONST_VECTOR_FROM_SIGNAL(f,    m_fLeftHandSIN(iter));     // 6
          m_f_LH_integral += Ki.tail<6>().cwiseProduct(fRef-f);
          e_f = Kf.tail<6>().cwiseProduct(fRef-f) + m_f_LH_integral;
          m_torques -= m_J_left_hand.topRows<3>().transpose()*e_f.tail<3>();
          m_torques -= m_J_left_hand.bottomRows<3>().transpose()*e_f.head<3>();
        }

        tauFB += m_torques.tail<N_JOINTS>();

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        COPY_VECTOR_TO_VECTOR(tauFB,s);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(tauFB2,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tauFB2 before initialization!");
          return s;
        }

        EIGEN_CONST_VECTOR_FROM_SIGNAL(q,           m_base6d_encodersSIN(iter));    // n+6
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,          m_jointsVelocitiesSIN(iter));   // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(Kp,          m_KpSIN(iter)); // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(Kd,          m_KdSIN(iter)); // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(Kf,          m_KfSIN(iter)); // 6*4
        EIGEN_CONST_VECTOR_FROM_SIGNAL(qRef,        m_qRefSIN(iter));   // n
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dqRef,       m_dqRefSIN(iter));  // n

        assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
        assert(dq.size()==N_JOINTS      && "Unexpected size of signal dq");
        assert(Kf.size()==6*4           && "Unexpected size of signal Kf");
        assert(Kp.size()==N_JOINTS      && "Unexpected size of signal Kp");
        assert(Kd.size()==N_JOINTS      && "Unexpected size of signal Kd");
        assert(qRef.size()==N_JOINTS    && "Unexpected size of signal qRef");
        assert(dqRef.size()==N_JOINTS   && "Unexpected size of signal dqRef");

        Eigen::Matrix<double,N_JOINTS,1> tauFB = Kp.cwiseProduct(qRef-q.tail<N_JOINTS>()) +
                                                 Kd.cwiseProduct(dqRef-dq);

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
          EIGEN_CONST_VECTOR_FROM_SIGNAL(fRef, m_fRightFootRefSIN(iter));  // 6
          EIGEN_CONST_VECTOR_FROM_SIGNAL(f,    m_fRightFootSIN(iter));     // 6
          e_f_vector = Kf.head<6>().cwiseProduct(fRef-f) + m_f_RF_integral;
          e_f = Spatial::ForceTpl<double>(e_f_vector.tail<3>(), e_f_vector.head<3>());
          e_f = m_sole_X_RF.applyInv(e_f);
          m_node_right_foot.body.Fext = m_node_right_foot.body.iX0.applyInv(e_f);
        }
        if(m_fLeftFootRefSIN.isPlugged())
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(fRef, m_fLeftFootRefSIN(iter));
          EIGEN_CONST_VECTOR_FROM_SIGNAL(f,    m_fLeftFootSIN(iter));
          e_f_vector = Kf.segment<6>(6).cwiseProduct(fRef-f) + m_f_LF_integral;
          e_f = Spatial::ForceTpl<double>(e_f_vector.tail<3>(), e_f_vector.head<3>());
          e_f = m_sole_X_LF.applyInv(e_f);
          m_node_left_foot.body.Fext = m_node_left_foot.body.iX0.applyInv(e_f);
        }
        if(m_fRightHandRefSIN.isPlugged())
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(fRef, m_fRightHandRefSIN(iter));
          EIGEN_CONST_VECTOR_FROM_SIGNAL(f,    m_fRightHandSIN(iter));
          e_f_vector = Kf.segment<6>(12).cwiseProduct(fRef-f) + m_f_RH_integral;
          e_f = Spatial::ForceTpl<double>(e_f_vector.tail<3>(), e_f_vector.head<3>());
          e_f = m_gripper_X_RH.applyInv(e_f);
          m_node_right_hand.body.Fext = m_node_right_hand.body.iX0.applyInv(e_f);
        }
        if(m_fLeftHandRefSIN.isPlugged())
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(fRef, m_fLeftHandRefSIN(iter));
          EIGEN_CONST_VECTOR_FROM_SIGNAL(f,    m_fLeftHandSIN(iter));
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
        COPY_VECTOR_TO_VECTOR(tauFB,s);

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

