/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-dyninv is free software: you can redistribute it and/or
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

#include <sot/torque_control/joint-torque-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <Eigen/Dense>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {

#define MODEL_INPUT_SIGNALS     m_k_tauSIN << m_k_vSIN \
                             << m_motorParameterKt_pSIN << m_motorParameterKt_nSIN \
                             << m_motorParameterKf_pSIN << m_motorParameterKf_nSIN \
                             << m_motorParameterKv_pSIN << m_motorParameterKv_nSIN \
                             << m_motorParameterKa_pSIN << m_motorParameterKa_nSIN <<  m_polySignDqSIN
//                              <<m_f_k1pSIN << m_f_k2pSIN << m_f_k3pSIN << m_f_k1nSIN << m_f_k2nSIN << m_f_k3nSIN << \
//                                m_f_q1pSIN << m_f_q2pSIN << m_f_q3pSIN << m_f_q1nSIN << m_f_q2nSIN << m_f_q3nSIN << \
//                                m_f_tau1pSIN << m_f_tau2pSIN << m_f_tau1nSIN << m_f_tau2nSIN << \
//                                m_g_k1pSIN << m_g_k2pSIN << m_g_k3pSIN << m_g_k1nSIN << m_g_k2nSIN << m_g_k3nSIN << \
//                                m_g_q1pSIN << m_g_q2pSIN << m_g_q3pSIN << m_g_q1nSIN << m_g_q2nSIN << m_g_q3nSIN << \
//                                m_g_dq1pSIN << m_g_dq2pSIN << m_g_dq1nSIN << m_g_dq2nSIN << \
//                                m_dq_thresholdSIN << m_ddq_thresholdSIN

#define ESTIMATOR_INPUT_SIGNALS m_base6d_encodersSIN << m_jointsVelocitiesSIN << m_jointsAccelerationsSIN << \
                                m_jointsTorquesSIN

#define TORQUE_CONTROL_INPUT_SIGNALS    m_jointsTorquesDesiredSIN << m_KpTorqueSIN  << m_KiTorqueSIN  << m_frictionCompensationPercentageSIN//<< m_activeJointsSIN
#define CURRENT_CONTROL_INPUT_SIGNALS   m_measuredCurrentSIN      << m_KpCurrentSIN << m_KiCurrentSIN 
#define ALL_INPUT_SIGNALS       m_pwmSIN << m_tauFFSIN << m_tauFBSIN << \
                                ESTIMATOR_INPUT_SIGNALS << TORQUE_CONTROL_INPUT_SIGNALS << CURRENT_CONTROL_INPUT_SIGNALS << MODEL_INPUT_SIGNALS

#define ALL_OUTPUT_SIGNALS      m_desiredCurrentSOUT << m_controlCurrentSOUT << m_predictedJointsTorquesSOUT << \
                                m_predictedPwmSOUT << m_predictedPwm_tauSOUT << \
                                m_pwm_ffSOUT << m_pwm_fbSOUT << m_pwm_frictionSOUT << m_smoothSignDqSOUT

      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace std;
      using namespace Eigen;

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef JointTorqueController EntityClassName;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(JointTorqueController,"JointTorqueController");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      JointTorqueController::
      JointTorqueController( const std::string & name )
        : Entity(name),
         CONSTRUCT_SIGNAL_IN(base6d_encoders,        dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(pwm,                    dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsVelocities,       dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsAccelerations,    dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsTorques,          dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsTorquesDesired,   dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(measuredCurrent,        dynamicgraph::Vector) 
        ,CONSTRUCT_SIGNAL_IN(KpTorque,                     dynamicgraph::Vector)   // proportional gain for torque feedback controller
        ,CONSTRUCT_SIGNAL_IN(KiTorque,                     dynamicgraph::Vector)   // integral gain for torque feedback controller
        ,CONSTRUCT_SIGNAL_IN(KpCurrent,                     dynamicgraph::Vector)  // proportional gain for current feedback controller
        ,CONSTRUCT_SIGNAL_IN(KiCurrent,                     dynamicgraph::Vector)  // integral gain for current feedback controller  
        ,CONSTRUCT_SIGNAL_IN(k_tau,                  dynamicgraph::Vector)// to be del
        ,CONSTRUCT_SIGNAL_IN(k_v,                    dynamicgraph::Vector)// to be del
        ,CONSTRUCT_SIGNAL_IN(frictionCompensationPercentage, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKt_p, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKt_n, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKf_p, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKf_n, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKv_p, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKv_n, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKa_p, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKa_n, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(polySignDq        , dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(tauFF,                  dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(tauFB,                  dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(desiredCurrent,        dynamicgraph::Vector,   ESTIMATOR_INPUT_SIGNALS <<
                                                                   TORQUE_CONTROL_INPUT_SIGNALS <<
                                                                   MODEL_INPUT_SIGNALS )

        ,CONSTRUCT_SIGNAL_OUT(controlCurrent,        dynamicgraph::Vector,   m_desiredCurrentSOUT <<
                                                                   CURRENT_CONTROL_INPUT_SIGNALS )
        ,CONSTRUCT_SIGNAL_OUT(predictedJointsTorques,  dynamicgraph::Vector, m_pwmSIN<<
                                                                   m_jointsVelocitiesSIN<<
                                                                   m_k_tauSIN<<
                                                                   m_k_vSIN)
        ,CONSTRUCT_SIGNAL_OUT(predictedPwm,            dynamicgraph::Vector, ESTIMATOR_INPUT_SIGNALS <<
                                                                   MODEL_INPUT_SIGNALS)
        ,CONSTRUCT_SIGNAL_OUT(predictedPwm_tau,        dynamicgraph::Vector, ESTIMATOR_INPUT_SIGNALS <<
                                                                   MODEL_INPUT_SIGNALS)
        ,CONSTRUCT_SIGNAL_OUT(pwm_ff,              dynamicgraph::Vector, m_tauFFSIN <<
                                                                  m_k_tauSIN)
        ,CONSTRUCT_SIGNAL_OUT(pwm_fb,              dynamicgraph::Vector, m_tauFBSIN <<
                                                                  m_jointsTorquesSIN <<
                                                                  m_jointsTorquesDesiredSIN <<
                                                                  m_k_tauSIN <<
                                                                  m_KpTorqueSIN)
        ,CONSTRUCT_SIGNAL_OUT(pwm_friction,        dynamicgraph::Vector, m_jointsVelocitiesSIN <<
                                                                  m_k_vSIN)
        ,CONSTRUCT_SIGNAL_OUT(smoothSignDq,        dynamicgraph::Vector, m_jointsVelocitiesSIN )


      {
        Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);
        m_firstIter = true;

        /* Commands. */
        addCommand("getTimestep", makeDirectGetter(*this,&m_dt,
                                  docDirectGetter("Control timestep","double")));
        addCommand("getActiveJoints", makeDirectGetter(*this,&m_activeJointsString,
                                      docDirectGetter("Active joints","bool")));

        addCommand("init", makeCommandVoid2(*this, &JointTorqueController::init,
                              docCommandVoid2("Initialize the controller.",
                                              "Control timestep [s].",
					      "Robot reference (string)")));
        addCommand("activate",
                   makeCommandVoid1(*this, &JointTorqueController::activate,
                                    docCommandVoid1("Activate torque control of the specified joint.",
                                                    "Joint name (string)")));
        addCommand("deactivate",
                   makeCommandVoid1(*this, &JointTorqueController::deactivate,
                                    docCommandVoid1("Deactivate torque control of the specified joint.",
                                                    "Joint name (string)")));
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      void JointTorqueController::init(const double &timestep,
				       const std::string &robot_ref)
      {
        assert(timestep>0.0 && "Timestep should be > 0");
        if(!m_base6d_encodersSIN.isPlugged())
          return SEND_MSG("Init failed: signal base6d_encoders is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsTorquesSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_jointsTorquesSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsTorquesDesiredSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_jointsTorquesDesiredSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_k_tauSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_k_tauSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_k_vSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_k_vSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KpTorqueSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KpTorqueSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KiTorqueSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KiTorqueSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KpCurrentSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KpCurrentSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KiCurrentSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KiCurrentSIN is not plugged", MSG_TYPE_ERROR);

	/* Retrieve m_robot_util  informations */
	std::string localName(robot_ref);
	if (isNameInRobotUtil(localName))
	  {	      
	    m_robot_util = getRobotUtil(localName);
	  }
	else 
	  {
	    SEND_MSG("You should have an entity controller manager initialized before",MSG_TYPE_ERROR);
	    return;
	  }
	
        m_dt = timestep;
        m_firstIter = true;
        m_tau_star.setZero(m_robot_util->m_nbJoints);
        m_current_star.setZero(m_robot_util->m_nbJoints);
        m_f.setZero(m_robot_util->m_nbJoints);
        m_g.setZero(m_robot_util->m_nbJoints);
        m_current_des.setZero(m_robot_util->m_nbJoints);
        m_tauErrIntegral.setZero(m_robot_util->m_nbJoints);
        m_currentErrIntegral.setZero(m_robot_util->m_nbJoints);
        m_qDes_for_position_controlled_joints.setZero(m_robot_util->m_nbJoints);
        m_activeJoints.setOnes(m_robot_util->m_nbJoints);
        updateActiveJointsString();
      }

      void JointTorqueController::activate(const string& jointName)
      {
        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;

        if(m_activeJoints[i]==0)
        {
          SEND_MSG("Activate joint "+jointName, MSG_TYPE_INFO);
          m_activeJoints[i] = 1;
          updateActiveJointsString();
        }
        else
          SEND_MSG("Joint "+jointName+" is already active.", MSG_TYPE_WARNING);
      }

      void JointTorqueController::deactivate(const string& jointName)
      {
        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;

        if(m_activeJoints[i]==1)
        {
          SEND_MSG("Deactivate joint "+jointName, MSG_TYPE_INFO);
          const dynamicgraph::Vector& base6d_encoders = m_base6d_encodersSIN.accessCopy();
          m_qDes_for_position_controlled_joints[i] = base6d_encoders(6+i);
          m_activeJoints[i] = 0;
          updateActiveJointsString();
        }
        else
          SEND_MSG("Joint "+jointName+" is already deactivated.", MSG_TYPE_WARNING);
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(desiredCurrent, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& q =             m_base6d_encodersSIN(iter);
        const Eigen::VectorXd& dq =            m_jointsVelocitiesSIN(iter);
        const Eigen::VectorXd& ddq =           m_jointsAccelerationsSIN(iter);
        const Eigen::VectorXd& tau =           m_jointsTorquesSIN(iter);
        const Eigen::VectorXd& tau_d =         m_jointsTorquesDesiredSIN(iter);
        const Eigen::VectorXd& kp =            m_KpTorqueSIN(iter);
        const Eigen::VectorXd& ki =            m_KiTorqueSIN(iter);
        const Eigen::VectorXd& k_tau =         m_k_tauSIN(iter);
        const Eigen::VectorXd& k_v =           m_k_vSIN(iter);
        const Eigen::VectorXd& frictionCompensationPercentage = m_frictionCompensationPercentageSIN(iter);
        const Eigen::VectorXd& motorParameterKt_p = m_motorParameterKt_pSIN(iter);
        const Eigen::VectorXd& motorParameterKt_n = m_motorParameterKt_nSIN(iter);
        const Eigen::VectorXd& motorParameterKf_p = m_motorParameterKf_pSIN(iter);
        const Eigen::VectorXd& motorParameterKf_n = m_motorParameterKf_nSIN(iter);
        const Eigen::VectorXd& motorParameterKv_p = m_motorParameterKv_pSIN(iter);
        const Eigen::VectorXd& motorParameterKv_n = m_motorParameterKv_nSIN(iter);
        const Eigen::VectorXd& motorParameterKa_p = m_motorParameterKa_pSIN(iter);
        const Eigen::VectorXd& motorParameterKa_n = m_motorParameterKa_nSIN(iter);
        const Eigen::VectorXd& polySignDq         = m_polySignDqSIN(iter);


//        const Eigen::VectorXd& activeJoints =  m_activeJointsSIN(iter);
//        const Eigen::VectorXd& ddq =           m_jointsAccelerationsSIN(iter);
//        const Eigen::VectorXd& dq_thr =        m_dq_thresholdSIN(iter);
//        const Eigen::VectorXd& ddq_thr =       m_ddq_thresholdSIN(iter);

        if(m_firstIter)
        {
          m_qDes_for_position_controlled_joints = q.tail(m_robot_util->m_nbJoints);
          m_firstIter = false;
        }

        m_tauErrIntegral += m_dt * ki.cwiseProduct(tau_d-tau);
        m_tau_star = tau_d + kp.cwiseProduct(tau_d - tau) + m_tauErrIntegral;
        if(dq.size()==(int)m_robot_util->m_nbJoints)
	  for(int i=0; i<(int)m_robot_util->m_nbJoints; i++)
            {
                m_current_des(i) = motorModel.getCurrent(m_tau_star(i), dq(i), ddq(i),
                                                         motorParameterKt_p(i), motorParameterKt_n(i),
                                                         motorParameterKf_p(i)*frictionCompensationPercentage(i), motorParameterKf_n(i)*frictionCompensationPercentage(i),
                                                         motorParameterKv_p(i), motorParameterKv_n(i),
                                                         motorParameterKa_p(i), motorParameterKa_n(i) , polySignDq(i));
            }
        else if(dq.size()==(int)(m_robot_util->m_nbJoints+6))
	  for(int i=0; i<(int)m_robot_util->m_nbJoints; i++)
            {
                m_current_des(i) = motorModel.getCurrent(m_tau_star(i), dq(i+6), ddq(i+6),
                                                         motorParameterKt_p(i), motorParameterKt_n(i),
                                                         motorParameterKf_p(i)*frictionCompensationPercentage(i), motorParameterKf_n(i)*frictionCompensationPercentage(i),
                                                         motorParameterKv_p(i), motorParameterKv_n(i),
                                                         motorParameterKa_p(i), motorParameterKa_n(i), polySignDq(i));
            }
            else
          SEND_ERROR_STREAM_MSG("Unexpected size of signal dq: "+toString(dq.size()));


        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	s = m_activeJoints.select(m_current_des,0.0).matrix();
	
//        SEND_MSG("qDes = "+toString(s), MSG_TYPE_DEBUG_STREAM);

//        const int JID = 1;
//        SEND_MSG(toString(iter)+" q   = "+toString(q[6+JID]),MSG_TYPE_DEBUG);
//        SEND_MSG("qDes = "+toString(m_q_des(JID)),MSG_TYPE_DEBUG);
//        SEND_MSG("dq   = "+toString(dq[JID]),MSG_TYPE_DEBUG);
//        SEND_MSG("tau  = "+toString(tau_d(JID)),MSG_TYPE_DEBUG);
        return s;
      }


 DEFINE_SIGNAL_OUT_FUNCTION(controlCurrent, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& current =       m_measuredCurrentSIN(iter);
        const Eigen::VectorXd& current_d =     m_desiredCurrentSOUT(iter);
        const Eigen::VectorXd& kp =            m_KpCurrentSIN(iter);
        const Eigen::VectorXd& ki =            m_KiCurrentSIN(iter);

        m_currentErrIntegral += m_dt * ki.cwiseProduct(current_d-current);
        m_current_star = current_d + kp.cwiseProduct(current_d - current) + m_currentErrIntegral;
        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	s = m_activeJoints.select(m_current_star,0.0).matrix(); //TODO check saturation
        return s;
      }



      DEFINE_SIGNAL_OUT_FUNCTION(predictedPwm, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& dq =      m_jointsVelocitiesSIN(iter);
        const Eigen::VectorXd& tau =     m_jointsTorquesSIN(iter);
        const Eigen::VectorXd& k_tau =         m_k_tauSIN(iter);
        const Eigen::VectorXd& k_v =           m_k_vSIN(iter);

        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	s = k_tau.cwiseProduct(tau) + k_v.cwiseProduct(dq);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(predictedPwm_tau, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& tau =     m_jointsTorquesSIN(iter);
        const Eigen::VectorXd& k_tau =         m_k_tauSIN(iter);

        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	s = k_tau.cwiseProduct(tau);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(pwm_ff, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& tauFF =         m_tauFFSIN(iter);
        const Eigen::VectorXd& k_tau =         m_k_tauSIN(iter);
        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	s = k_tau.cwiseProduct(tauFF);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(pwm_fb, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& tau =           m_jointsTorquesSIN(iter);
        const Eigen::VectorXd& tau_d =         m_jointsTorquesDesiredSIN(iter);
        const Eigen::VectorXd& tauFB =         m_tauFBSIN(iter);
        const Eigen::VectorXd& k_tau =         m_k_tauSIN(iter);
        const Eigen::VectorXd& k_p =           m_KpTorqueSIN(iter);
        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	s = k_tau.cwiseProduct(tauFB + k_p.cwiseProduct(tau_d-tau));
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(pwm_friction, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& k_v =         m_k_vSIN(iter);
        const Eigen::VectorXd& dq =          m_jointsVelocitiesSIN(iter);
        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	s = k_v.cwiseProduct(dq);
        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(smoothSignDq, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& dq =            m_jointsVelocitiesSIN(iter);
        const Eigen::VectorXd& polySignDq =    m_polySignDqSIN(iter);
        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	
        for(int i=0; i<(int)m_robot_util->m_nbJoints; i++)
          s(i) = motorModel.smoothSign(dq[i], 0.1, polySignDq[i]); //TODO Use Eigen binaryexpr
        return s;
      }


//      DEFINE_SIGNAL_OUT_FUNCTION(smoothSignDq, dynamicgraph::Vector)
//      {
//        const Eigen::VectorXd dq =      m_jointsVelocitiesSIN(iter);
//        const Eigen::VectorXd dq_thr =  m_dq_thresholdSIN(iter);

//        if(s.size()!=N_JOINTS)
//          s.resize(N_JOINTS);
//        for(int i=0; i<N_JOINTS; i++)
//        {
//          if(dq[i]>dq_thr[i])
//            s(i) = 1.0;
//          else if(dq[i]<-dq_thr[i])
//            s(i) = -1.0;
//          else
//            s(i) = pow(dq[i]/dq_thr[i],3);
//        }
//        return s;
//      }

//      DEFINE_SIGNAL_OUT_FUNCTION(smoothSignDdq, dynamicgraph::Vector)
//      {
//        const Eigen::VectorXd ddq =      m_jointsAccelerationsSIN(iter);
//        const Eigen::VectorXd ddq_thr =  m_ddq_thresholdSIN(iter);

//        if(s.size()!=N_JOINTS)
//          s.resize(N_JOINTS);
//        for(int i=0; i<N_JOINTS; i++)
//        {
//          if(ddq[i]>ddq_thr[i])
//            s(i) = 1.0;
//          else if(ddq[i]<-ddq_thr[i])
//            s(i) = -1.0;
//          else
//            s(i) = pow(ddq[i]/ddq_thr[i],3);
//        }
//        return s;
//      }

      DEFINE_SIGNAL_OUT_FUNCTION(predictedJointsTorques, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& pwm =       m_pwmSIN(iter);
        const Eigen::VectorXd& dq =        m_jointsVelocitiesSIN(iter);
        const Eigen::VectorXd& k_tau =     m_k_tauSIN(iter);
        const Eigen::VectorXd& k_v =       m_k_vSIN(iter);

        /// k_tau^{-1}*(delta_q - k_v*dq)
        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);

        //TODO: Use Eigen cwise operations
        for(int i=0; i<(int)m_robot_util->m_nbJoints; i++)
          if(k_tau(i)!=0.0)
            s(i) = (pwm(i) - k_v(i)*dq(i))/k_tau(i);
          else
            s(i) = 0.0;

        return s;
      }

//      void JointTorqueController::compute_f(const VectorXd &tau, const_SigVectorXd &dq, const_SigVectorXd &dq_thr, int iter, VectorXd &f)
//      {
//        const Eigen::VectorXd& k1p = m_f_k1pSIN(iter);
//        const Eigen::VectorXd& k2p = m_f_k2pSIN(iter);
//        const Eigen::VectorXd& k3p = m_f_k3pSIN(iter);
//        const Eigen::VectorXd& k1n = m_f_k1nSIN(iter);
//        const Eigen::VectorXd& k2n = m_f_k2nSIN(iter);
//        const Eigen::VectorXd& k3n = m_f_k3nSIN(iter);
//        const Eigen::VectorXd& q1p = m_f_q1pSIN(iter);
//        const Eigen::VectorXd& q2p = m_f_q2pSIN(iter);
//        const Eigen::VectorXd& q3p = m_f_q3pSIN(iter);
//        const Eigen::VectorXd& q1n = m_f_q1nSIN(iter);
//        const Eigen::VectorXd& q2n = m_f_q2nSIN(iter);
//        const Eigen::VectorXd& q3n = m_f_q3nSIN(iter);
//        const Eigen::VectorXd& tau1p = m_f_tau1pSIN(iter);
//        const Eigen::VectorXd& tau2p = m_f_tau2pSIN(iter);
//        const Eigen::VectorXd& tau1n = m_f_tau1nSIN(iter);
//        const Eigen::VectorXd& tau2n = m_f_tau2nSIN(iter);

//        for(int i=0; i<N_JOINTS; i++)
//        {
//          if(dq[i]>dq_thr[i])
//          {
//            f[i] = compute_piecewise_linear(tau[i], k1p[i],k2p[i],k3p[i],q1p[i],q2p[i],q3p[i],tau1p[i],tau2p[i]);
//          }
//          else if(dq[i]<-dq_thr[i])
//          {
//            f[i] = compute_piecewise_linear(tau[i], k1n[i],k2n[i],k3n[i],q1n[i],q2n[i],q3n[i],tau1n[i],tau2n[i]);
//          }
//          else
//          {
//            double fp = compute_piecewise_linear(tau[i], k1p[i],k2p[i],k3p[i],q1p[i],q2p[i],q3p[i],tau1p[i],tau2p[i]);
//            double fn = compute_piecewise_linear(tau[i], k1n[i],k2n[i],k3n[i],q1n[i],q2n[i],q3n[i],tau1n[i],tau2n[i]);
//            double alpha = 0.5*pow(dq[i]/dq_thr[i],3) + 0.5;
//            f[i] = alpha*fp + (1-alpha)*fn;
//          }
//        }
//      }

//      void JointTorqueController::compute_g(const_SigVectorXd &dq, const_SigVectorXd &ddq, const_SigVectorXd &ddq_thr, int iter, VectorXd &g)
//      {
//        const Eigen::VectorXd& k1p = m_g_k1pSIN(iter);
//        const Eigen::VectorXd& k2p = m_g_k2pSIN(iter);
//        const Eigen::VectorXd& k3p = m_g_k3pSIN(iter);
//        const Eigen::VectorXd& k1n = m_g_k1nSIN(iter);
//        const Eigen::VectorXd& k2n = m_g_k2nSIN(iter);
//        const Eigen::VectorXd& k3n = m_g_k3nSIN(iter);
//        const Eigen::VectorXd& q1p = m_g_q1pSIN(iter);
//        const Eigen::VectorXd& q2p = m_g_q2pSIN(iter);
//        const Eigen::VectorXd& q3p = m_g_q3pSIN(iter);
//        const Eigen::VectorXd& q1n = m_g_q1nSIN(iter);
//        const Eigen::VectorXd& q2n = m_g_q2nSIN(iter);
//        const Eigen::VectorXd& q3n = m_g_q3nSIN(iter);
//        const Eigen::VectorXd& dq1p = m_g_dq1pSIN(iter);
//        const Eigen::VectorXd& dq2p = m_g_dq2pSIN(iter);
//        const Eigen::VectorXd& dq1n = m_g_dq1nSIN(iter);
//        const Eigen::VectorXd& dq2n = m_g_dq2nSIN(iter);

//        for(int i=0; i<N_JOINTS; i++)
//        {
//          if(ddq[i]>ddq_thr[i])
//          {
//            g[i] = compute_piecewise_linear(dq[i], k1p[i],k2p[i],k3p[i],q1p[i],q2p[i],q3p[i],dq1p[i],dq2p[i]);
//          }
//          else if(ddq[i]<-ddq_thr[i])
//          {
//            g[i] = compute_piecewise_linear(dq[i], k1n[i],k2n[i],k3n[i],q1n[i],q2n[i],q3n[i],dq1n[i],dq2n[i]);
//          }
//          else
//          {
//            double gp = compute_piecewise_linear(dq[i], k1p[i],k2p[i],k3p[i],q1p[i],q2p[i],q3p[i],dq1p[i],dq2p[i]);
//            double gn = compute_piecewise_linear(dq[i], k1n[i],k2n[i],k3n[i],q1n[i],q2n[i],q3n[i],dq1n[i],dq2n[i]);
//            double alpha = 0.5*pow(ddq[i]/ddq_thr[i],3) + 0.5;
//            g[i] = alpha*gp + (1-alpha)*gn;
//          }
//        }
//      }

      double JointTorqueController::compute_piecewise_linear(const double &x, const double &a1, const double &a2, const double &a3, const double &b1,
                                                             const double &b2, const double &b3, const double &x1, const double &x2) const
      {
        if(x<x1)
          return a1*x+b1;
        if(x<x2)
          return a2*x+b2;
        return a3*x+b3;
      }

      bool JointTorqueController::convertJointNameToJointId(const std::string& name, unsigned int& id)
      {
        // Check if the joint name exists
        int jid = (int)m_robot_util->get_id_from_name(name);
        if (jid<0)
        {
          SEND_MSG("The specified joint name does not exist", MSG_TYPE_ERROR);
          std::stringstream ss;
          for(map<string, Index>::const_iterator 
		it = m_robot_util->m_name_to_id.begin(); 
	      it != m_robot_util->m_name_to_id.end(); it++)
            ss<<it->first<<", ";
          SEND_MSG("Possible joint names are: "+ss.str(), MSG_TYPE_INFO);
          return false;
        }
        id = jid;
        return true;
      }

      void JointTorqueController::display( std::ostream& os ) const
      {
        os << "JointTorqueController "<<getName()<<":\n";
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
