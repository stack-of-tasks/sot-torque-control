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

#include <Eigen/Dense>

#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/joint-torque-controller.hh>
#include <sot/torque_control/commands-helper.hh>


namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {

#define MODEL_INPUT_SIGNALS    m_motorParameterKt_pSIN << m_motorParameterKt_nSIN \
                            << m_motorParameterKf_pSIN << m_motorParameterKf_nSIN \
                            << m_motorParameterKv_pSIN << m_motorParameterKv_nSIN \
                            << m_motorParameterKa_pSIN << m_motorParameterKa_nSIN <<  m_polySignDqSIN

#define ESTIMATOR_INPUT_SIGNALS m_jointsPositionsSIN << m_jointsVelocitiesSIN << m_jointsAccelerationsSIN << \
                                m_jointsTorquesSIN << m_jointsTorquesDerivativeSIN

#define TORQUE_INTEGRAL_INPUT_SIGNALS m_KiTorqueSIN << m_torque_integral_saturationSIN
#define TORQUE_CONTROL_INPUT_SIGNALS  m_jointsTorquesDesiredSIN << m_KpTorqueSIN  << m_KdTorqueSIN \
                                      << m_coulomb_friction_compensation_percentageSIN
#define VEL_CONTROL_INPUT_SIGNALS  m_dq_desSIN << m_KdVelSIN << m_KiVelSIN \

#define ALL_INPUT_SIGNALS       ESTIMATOR_INPUT_SIGNALS << TORQUE_INTEGRAL_INPUT_SIGNALS << \
                                TORQUE_CONTROL_INPUT_SIGNALS << VEL_CONTROL_INPUT_SIGNALS << \
                                MODEL_INPUT_SIGNALS
#define ALL_OUTPUT_SIGNALS      m_uSOUT << m_torque_error_integralSOUT << m_smoothSignDqSOUT

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
        : Entity(name)
        ,CONSTRUCT_SIGNAL_IN(jointsPositions,         dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsVelocities,        dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsAccelerations,     dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsTorques,           dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsTorquesDesired,    dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsTorquesDerivative, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(dq_des,                  dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(KpTorque,                     dynamicgraph::Vector)   // proportional gain for torque feedback controller
        ,CONSTRUCT_SIGNAL_IN(KiTorque,                     dynamicgraph::Vector)   // integral gain for torque feedback controller
        ,CONSTRUCT_SIGNAL_IN(KdTorque,                     dynamicgraph::Vector)   // derivative gain for torque feedback controller
        ,CONSTRUCT_SIGNAL_IN(KdVel,                        dynamicgraph::Vector)   // derivative gain for velocity feedback
        ,CONSTRUCT_SIGNAL_IN(KiVel,                        dynamicgraph::Vector)   // integral gain for velocity feedback
        ,CONSTRUCT_SIGNAL_IN(coulomb_friction_compensation_percentage, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKt_p, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKt_n, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKf_p, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKf_n, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKv_p, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKv_n, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKa_p, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKa_n, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(polySignDq        , dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(torque_integral_saturation, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(u,                     dynamicgraph::Vector, ESTIMATOR_INPUT_SIGNALS <<
                                                                           TORQUE_CONTROL_INPUT_SIGNALS <<
                                                                           VEL_CONTROL_INPUT_SIGNALS <<
                                                                           MODEL_INPUT_SIGNALS <<
                                                                           m_torque_error_integralSOUT)
        ,CONSTRUCT_SIGNAL_OUT(torque_error_integral, dynamicgraph::Vector, m_jointsTorquesSIN <<
                                                                           m_jointsTorquesDesiredSIN <<
                                                                           TORQUE_INTEGRAL_INPUT_SIGNALS )
        ,CONSTRUCT_SIGNAL_OUT(smoothSignDq,          dynamicgraph::Vector, m_jointsVelocitiesSIN )
      {
        Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);

        /* Commands. */
        addCommand("getTimestep", makeDirectGetter(*this,&m_dt,
                                  docDirectGetter("Control timestep","double")));

        addCommand("init", makeCommandVoid2(*this, &JointTorqueController::init,
                              docCommandVoid2("Initialize the controller.",
                                              "Control timestep [s].",
					      "Robot reference (string)")));
        addCommand("reset_integral", makeCommandVoid0(*this, &JointTorqueController::reset_integral,
                                     docCommandVoid0("Reset the integral error.")));
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      void JointTorqueController::init(const double &timestep,
				       const std::string &robot_ref)
      {
        assert(timestep>0.0 && "Timestep should be > 0");
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsTorquesSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_jointsTorquesSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsTorquesDesiredSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_jointsTorquesDesiredSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KpTorqueSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KpTorqueSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KiTorqueSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KiTorqueSIN is not plugged", MSG_TYPE_ERROR);

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
        m_tau_star.setZero(m_robot_util->m_nbJoints);
        m_current_des.setZero(m_robot_util->m_nbJoints);
        m_tauErrIntegral.setZero(m_robot_util->m_nbJoints);
//        m_dqDesIntegral.setZero(m_robot_util->m_nbJoints);
        m_dqErrIntegral.setZero(m_robot_util->m_nbJoints);
      }

      void JointTorqueController::reset_integral()
      {
        m_tauErrIntegral.setZero();
        m_dqErrIntegral.setZero();
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(u, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& q                  = m_jointsPositionsSIN(iter);
        const Eigen::VectorXd& dq                 = m_jointsVelocitiesSIN(iter);
        const Eigen::VectorXd& ddq                = m_jointsAccelerationsSIN(iter);
        const Eigen::VectorXd& tau                = m_jointsTorquesSIN(iter);
        const Eigen::VectorXd& dtau               = m_jointsTorquesDerivativeSIN(iter);
        const Eigen::VectorXd& tau_d              = m_jointsTorquesDesiredSIN(iter);
//        const Eigen::VectorXd& dtau_d             = m_jointsTorquesDesiredDerivativeSIN(iter);
        const Eigen::VectorXd& dq_des             = m_dq_desSIN(iter);
        const Eigen::VectorXd& kp                 = m_KpTorqueSIN(iter);
        const Eigen::VectorXd& kd                 = m_KdTorqueSIN(iter);
        const Eigen::VectorXd& kd_vel             = m_KdVelSIN(iter);
        const Eigen::VectorXd& ki_vel             = m_KiVelSIN(iter);
        const Eigen::VectorXd& tauErrInt          = m_torque_error_integralSOUT(iter);
        const Eigen::VectorXd& colFricCompPerc    = m_coulomb_friction_compensation_percentageSIN(iter);
        const Eigen::VectorXd& motorParameterKt_p = m_motorParameterKt_pSIN(iter);
        const Eigen::VectorXd& motorParameterKt_n = m_motorParameterKt_nSIN(iter);
        const Eigen::VectorXd& motorParameterKf_p = m_motorParameterKf_pSIN(iter);
        const Eigen::VectorXd& motorParameterKf_n = m_motorParameterKf_nSIN(iter);
        const Eigen::VectorXd& motorParameterKv_p = m_motorParameterKv_pSIN(iter);
        const Eigen::VectorXd& motorParameterKv_n = m_motorParameterKv_nSIN(iter);
        const Eigen::VectorXd& motorParameterKa_p = m_motorParameterKa_pSIN(iter);
        const Eigen::VectorXd& motorParameterKa_n = m_motorParameterKa_nSIN(iter);
        const Eigen::VectorXd& polySignDq         = m_polySignDqSIN(iter);
//        const Eigen::VectorXd& dq_thr =        m_dq_thresholdSIN(iter);

        m_tau_star = tau_d + kp.cwiseProduct(tau_d - tau) + tauErrInt - kd.cwiseProduct(dtau);

        int offset = 0;
        if(dq.size()==(int)(m_robot_util->m_nbJoints+6))
          offset = 6;

        m_dqErrIntegral += m_dt * ki_vel.cwiseProduct(dq_des-dq);
        const Eigen::VectorXd& err_int_sat =   m_torque_integral_saturationSIN(iter);
        // saturate
        bool saturating = false;
        for(int i=0; i<(int)m_robot_util->m_nbJoints; i++)
        {
          if(m_dqErrIntegral(i) > err_int_sat(i))
          {
            saturating = true;
            m_dqErrIntegral(i) = err_int_sat(i);
          }
          else if(m_dqErrIntegral(i) < -err_int_sat(i))
          {
            saturating = true;
            m_dqErrIntegral(i) = -err_int_sat(i);
          }
        }
        if(saturating)
          SEND_INFO_STREAM_MSG("Saturate dqErr integral: "+toString(m_dqErrIntegral.head<12>()));

        for(int i=0; i<(int)m_robot_util->m_nbJoints; i++)
        {
          m_current_des(i) = motorModel.getCurrent(m_tau_star(i),
                                                   dq(i+offset)+kd_vel(i)*(dq_des(i)-dq(i+offset)) + m_dqErrIntegral(i), //ki_vel(i)*(m_dqDesIntegral(i)-q(i)),
                                                   ddq(i+offset),
                                                   motorParameterKt_p(i),
                                                   motorParameterKt_n(i),
                                                   motorParameterKf_p(i)*colFricCompPerc(i),
                                                   motorParameterKf_n(i)*colFricCompPerc(i),
                                                   motorParameterKv_p(i),
                                                   motorParameterKv_n(i),
                                                   motorParameterKa_p(i),
                                                   motorParameterKa_n(i),
                                                   polySignDq(i));
        }

        s = m_current_des;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(torque_error_integral, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& tau =           m_jointsTorquesSIN(iter);
        const Eigen::VectorXd& tau_d =         m_jointsTorquesDesiredSIN(iter);
        const Eigen::VectorXd& err_int_sat =   m_torque_integral_saturationSIN(iter);
        const Eigen::VectorXd& ki =            m_KiTorqueSIN(iter);

        // compute torque error integral and saturate
        m_tauErrIntegral += m_dt * ki.cwiseProduct(tau_d-tau);
        for(int i=0; i<(int)m_robot_util->m_nbJoints; i++)
        {
          if(m_tauErrIntegral(i) > err_int_sat(i))
            m_tauErrIntegral(i) = err_int_sat(i);
          else if(m_tauErrIntegral(i) < -err_int_sat(i))
            m_tauErrIntegral(i) = -err_int_sat(i);
        }

        s = m_tauErrIntegral;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(smoothSignDq, dynamicgraph::Vector)
      {
        const Eigen::VectorXd& dq =            m_jointsVelocitiesSIN(iter);
        const Eigen::VectorXd& polySignDq =    m_polySignDqSIN(iter);
        if(s.size()!=(int)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
	
        for(int i=0; i<(int)m_robot_util->m_nbJoints; i++)
          s(i) = motorModel.smoothSign(dq[i], 0.1,
				       static_cast<unsigned int>(polySignDq[i]));
	//TODO Use Eigen binaryexpr
        return s;
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
