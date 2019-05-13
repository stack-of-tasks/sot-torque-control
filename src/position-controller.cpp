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


#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/position-controller.hh>
#include <sot/torque_control/commands-helper.hh>
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
//Size to be aligned                "-------------------------------------------------------"
#define PROFILE_PWM_DES_COMPUTATION "PositionController: desired pwm computation            "

#define GAIN_SIGNALS      m_KpSIN << m_KdSIN << m_KiSIN
#define REF_JOINT_SIGNALS m_qRefSIN << m_dqRefSIN
#define STATE_SIGNALS     m_base6d_encodersSIN << m_jointsVelocitiesSIN

#define INPUT_SIGNALS     STATE_SIGNALS << REF_JOINT_SIGNALS << GAIN_SIGNALS

#define OUTPUT_SIGNALS m_pwmDesSOUT << m_qErrorSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef PositionController EntityClassName;
      typedef Eigen::Matrix<double,Eigen::Dynamic,1> VectorN;
      typedef Eigen::Matrix<double,Eigen::Dynamic,1> VectorN6;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PositionController,
                                         "PositionController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      PositionController::
          PositionController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(base6d_encoders,     dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(jointsVelocities,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(qRef,                dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(dqRef,               dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kp,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kd,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Ki,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_OUT(pwmDes,             dynamicgraph::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(qError,             dynamicgraph::Vector, m_base6d_encodersSIN <<
                                                                  m_qRefSIN)
	    ,m_robot_util(RefVoidRobotUtil())	      
            ,m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init",
                   makeCommandVoid2(*this, &PositionController::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "Time period in seconds (double)",
						    "Reference to the robot (string)")));
        addCommand("resetIntegral",
                   makeCommandVoid0(*this, &PositionController::resetIntegral,
                                    docCommandVoid0("Reset the integral.")));
      }

      void PositionController::init(const double& dt,
				    const std::string& robotRef)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(!m_base6d_encodersSIN.isPlugged())
          return SEND_MSG("Init failed: signal base6d_encoders is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_qRefSIN.isPlugged())
          return SEND_MSG("Init failed: signal qRef is not plugged", MSG_TYPE_ERROR);
        if(!m_dqRefSIN.isPlugged())
          return SEND_MSG("Init failed: signal dqRef is not plugged", MSG_TYPE_ERROR);
        if(!m_KpSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
        if(!m_KdSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kd is not plugged", MSG_TYPE_ERROR);
        if(!m_KiSIN.isPlugged())
          return SEND_MSG("Init failed: signal Ki is not plugged", MSG_TYPE_ERROR);

	/* Retrieve m_robot_util  informations */
	std::string localName(robotRef);
	if (isNameInRobotUtil(localName))
	  m_robot_util = getRobotUtil(localName);
	else
	  {
	    SEND_MSG("You should have an entity controller manager initialized before",MSG_TYPE_ERROR);
	    return;
	  }

        m_dt = dt;

	m_pwmDes.setZero(m_robot_util->m_nbJoints);
        m_q.setZero(m_robot_util->m_nbJoints+6);
        m_dq.setZero(m_robot_util->m_nbJoints);

        resetIntegral();

        m_initSucceeded = true;
      }

      void PositionController::resetIntegral()
      {
        m_e_integral.setZero(m_robot_util->m_nbJoints);
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(pwmDes,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal pwmDes before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_PWM_DES_COMPUTATION);
        {
          const VectorN& Kp =        m_KpSIN(iter); // n
          const VectorN& Kd =        m_KdSIN(iter); // n
          const VectorN6& q =         m_base6d_encodersSIN(iter);     //n+6
          const VectorN& dq =        m_jointsVelocitiesSIN(iter);     // n
          const VectorN& qRef =      m_qRefSIN(iter);   // n
          const VectorN& dqRef =     m_dqRefSIN(iter);  // n

          assert(q.size()==static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints+6)     && "Unexpected size of signal base6d_encoder");
          assert(dq.size()==static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints)      && "Unexpected size of signal dq");
          assert(qRef.size()==static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints)    && "Unexpected size of signal qRef");
          assert(dqRef.size()==static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints)   && "Unexpected size of signal dqRef");
          assert(Kp.size()==static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints)      && "Unexpected size of signal Kd");
          assert(Kd.size()==static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints)      && "Unexpected size of signal Kd");

          m_pwmDes = Kp.cwiseProduct(qRef-q.tail(m_robot_util->m_nbJoints)) + Kd.cwiseProduct(dqRef-dq);

	  if(s.size()!=static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints))
	    s.resize(m_robot_util->m_nbJoints);
	  s = m_pwmDes;
        }
        getProfiler().stop(PROFILE_PWM_DES_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(qError,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal qError before initialization!",MSG_TYPE_WARNING_STREAM);
          return s;
        }

        const VectorN6& q =         m_base6d_encodersSIN(iter);     //n+6
        const VectorN& qRef =      m_qRefSIN(iter);   // n
        assert(q.size()==static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints+6)     && "Unexpected size of signal base6d_encoder");
        assert(qRef.size()==static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints)    && "Unexpected size of signal qRef");

        if(s.size()!=static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints))
          s.resize(m_robot_util->m_nbJoints);
	s = qRef - q.tail(m_robot_util->m_nbJoints);


        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void PositionController::display(std::ostream& os) const
      {
        os << "PositionController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

