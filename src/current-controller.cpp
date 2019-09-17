/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 */


#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>

#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/current-controller.hh>
#include <sot/torque_control/commands-helper.hh>

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
      using namespace dg::sot::torque_control;


#define SAFETY_SIGNALS m_i_maxSIN << m_u_maxSIN << m_u_saturationSIN
#define INPUT_SIGNALS  m_i_desSIN << m_percentage_dead_zone_compensationSIN << \
                       SAFETY_SIGNALS << m_i_max_dead_zone_compensationSIN << m_dqSIN << \
                       m_bemf_factorSIN << m_in_out_gainSIN << m_i_measuredSIN << \
                       m_dead_zone_offsetsSIN << m_i_sens_gainsSIN << m_i_sensor_offsets_low_levelSIN << \
                       m_i_sensor_offsets_real_inSIN << m_kp_currentSIN << m_ki_currentSIN << m_percentage_bemf_compensationSIN
#define OUTPUT_SIGNALS m_uSOUT << m_u_safeSOUT << m_i_realSOUT << m_i_low_levelSOUT << \
                       m_dead_zone_compensationSOUT << m_i_errorsSOUT << m_i_errors_ll_wo_bemfSOUT << \
                       m_i_sensor_offsets_real_outSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef CurrentController EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CurrentController,
                                         "CurrentController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      //to do rename 'pwm' to 'current'
      CurrentController::
      CurrentController(const std::string& name)
        : Entity(name)
        ,CONSTRUCT_SIGNAL_IN(i_des,                             dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(i_measured,                        dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(i_sens_gains,                      dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(kp_current,                        dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(ki_current,                        dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(i_max,                             dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(u_max,                             dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(u_saturation,                      dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(in_out_gain,                       dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(dq,                                dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(bemf_factor,                       dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(percentage_bemf_compensation,      dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(dead_zone_offsets,                 dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(percentage_dead_zone_compensation, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(i_max_dead_zone_compensation,      dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(i_sensor_offsets_low_level,        dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(i_sensor_offsets_real_in,          dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(u,                        dynamicgraph::Vector, m_i_desSIN)
        ,CONSTRUCT_SIGNAL_OUT(u_safe,                   dynamicgraph::Vector, INPUT_SIGNALS <<
                                                                               m_uSOUT <<
                                                                               m_i_realSOUT <<
                                                                               m_i_low_levelSOUT <<
                                                                               m_i_sensor_offsets_real_outSOUT)
        ,CONSTRUCT_SIGNAL_OUT(i_real,                    dynamicgraph::Vector, m_i_measuredSIN <<
                                                                               m_i_sens_gainsSIN <<
                                                                               m_i_sensor_offsets_real_outSOUT)
        ,CONSTRUCT_SIGNAL_OUT(i_low_level,               dynamicgraph::Vector, m_i_measuredSIN <<
                                                                               m_i_sens_gainsSIN <<
                                                                               m_i_sensor_offsets_low_levelSIN)
        ,CONSTRUCT_SIGNAL_OUT(i_sensor_offsets_real_out, dynamicgraph::Vector, m_i_measuredSIN <<
                                                                               m_i_sensor_offsets_real_inSIN)
        ,CONSTRUCT_SIGNAL_OUT(dead_zone_compensation,    dynamicgraph::Vector, m_u_safeSOUT <<
                                                                               m_dead_zone_offsetsSIN)
        ,CONSTRUCT_SIGNAL_OUT(i_errors,                  dynamicgraph::Vector, m_i_realSOUT <<
                                                                               m_uSOUT)
        ,CONSTRUCT_SIGNAL_OUT(i_errors_ll_wo_bemf,       dynamicgraph::Vector, m_i_realSOUT <<
                                                                               m_uSOUT <<
                                                                               m_percentage_bemf_compensationSIN <<
                                                                               m_dqSIN <<
                                                                               m_bemf_factorSIN)
        ,m_robot_util(RefVoidRobotUtil())
        ,m_initSucceeded(false)
        ,m_emergency_stop_triggered(false)
        ,m_is_first_iter(true)
        ,m_iter(0)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS);

        /* Commands. */
        addCommand("init",
                   makeCommandVoid3(*this, &CurrentController::init,
                                    docCommandVoid3("Initialize the entity.",
                                                    "Time period in seconds (double)",
						    "Robot reference (string)",
						    "Number of iterations while control is disabled to calibrate current sensors (int)")));

        addCommand("reset_integral",
                   makeCommandVoid0(*this, &CurrentController::reset_integral,
                                    docCommandVoid0("Reset the integral error.")));
      }

      void CurrentController::init(const double & dt,
                                   const std::string &robotRef,
                                   const unsigned int & currentOffsetIters )
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        m_dt = dt;
        m_initSucceeded = true;
        m_currentOffsetIters = currentOffsetIters;

        std::string localName(robotRef);
        if (!isNameInRobotUtil(localName))
        {
          m_robot_util = createRobotUtil(localName);
        }
        else
        {
          m_robot_util = getRobotUtil(localName);
        }

        m_i_offsets_real.setZero(m_robot_util->m_nbJoints);
        m_i_err_integr.setZero(m_robot_util->m_nbJoints);
        m_dz_coeff.setZero(m_robot_util->m_nbJoints);
        m_avg_i_err_pos.setZero(m_robot_util->m_nbJoints);
        m_avg_i_err_neg.setZero(m_robot_util->m_nbJoints);
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(u, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal u before initialization!");
          return s;
        }

        if(m_is_first_iter)
          m_is_first_iter = false;

        if(s.size()!=(Eigen::VectorXd::Index) m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);

        const dynamicgraph::Vector& i_des                     = m_i_desSIN(iter);
        const dynamicgraph::Vector& i_real                    = m_i_realSOUT(iter);
        const dynamicgraph::Vector& i_ll                      = m_i_low_levelSOUT(iter);
        const dynamicgraph::Vector& cur_sens_gains            = m_i_sens_gainsSIN(iter);
        const dynamicgraph::Vector& i_offset_real             = m_i_sensor_offsets_real_outSOUT(iter);
        const dynamicgraph::Vector& dq                        = m_dqSIN(iter);
        //const dynamicgraph::Vector& in_out_gain               = m_in_out_gainSIN(iter);
        const dynamicgraph::Vector& dead_zone_offsets         = m_dead_zone_offsetsSIN(iter);
        const dynamicgraph::Vector& dead_zone_comp_perc       = m_percentage_dead_zone_compensationSIN(iter);
        const dynamicgraph::Vector& bemf_factor               = m_bemf_factorSIN(iter);
        const dynamicgraph::Vector& bemf_comp_perc            = m_percentage_bemf_compensationSIN(iter);
        const dynamicgraph::Vector& i_max_dz_comp             = m_i_max_dead_zone_compensationSIN(iter);
        const dynamicgraph::Vector& kp                        = m_kp_currentSIN(iter);
        const dynamicgraph::Vector& ki                        = m_ki_currentSIN(iter);

        m_i_err_integr += ki.cwiseProduct(i_des-i_real);

        s = i_des + m_i_err_integr;                                     // feedforward + integral feedback
        s += kp.cwiseProduct(i_des-i_real);                             // proportional feedback
        s += cur_sens_gains.cwiseProduct(i_offset_real);                // sensor offset compensation
        s += bemf_comp_perc.cwiseProduct(bemf_factor.cwiseProduct(dq)); // back-EMF compensation

        for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
        {
          double err = s(i)-i_ll(i);
          if( err > i_max_dz_comp(i) )
            m_dz_coeff(i) = 1.0;
          else if( err < -i_max_dz_comp(i) )
            m_dz_coeff(i) = -1.0;
          else
            m_dz_coeff(i) = err / i_max_dz_comp(i);
        }

        // compensate dead zone
        s += m_dz_coeff.cwiseProduct(dead_zone_comp_perc.cwiseProduct(dead_zone_offsets));
        //s = s.cwiseProduct(in_out_gain);

        // when estimating current offset set ctrl to zero
        if(m_emergency_stop_triggered || m_iter<static_cast<int>(m_currentOffsetIters))
          s.setZero();

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(u_safe,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal u_safe before initialization!");
          return s;
        }

        const dynamicgraph::Vector& u                         = m_uSOUT(iter);
        const dynamicgraph::Vector& u_max                     = m_u_maxSIN(iter);
        const dynamicgraph::Vector& u_saturation              = m_u_saturationSIN(iter);
        const dynamicgraph::Vector& i_real                    = m_i_realSOUT(iter);
        const dynamicgraph::Vector& in_out_gain               = m_in_out_gainSIN(iter);

        if(s.size()!=static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints))
          s.resize(m_robot_util->m_nbJoints);

        for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
        {
          double i_max = m_i_maxSIN(iter)(i);
          if( (fabs(i_real(i)) > i_max))
          {
            m_emergency_stop_triggered = true;
            SEND_MSG("Joint "+m_robot_util->get_name_from_id(i)+" measured current is too large: "+
                     toString(i_real(i))+"A > "+toString(i_max)+"A", MSG_TYPE_ERROR);
          }

          if(fabs(u(i)) > u_max(i))
          {
            m_emergency_stop_triggered = true;
            SEND_MSG("Joint "+m_robot_util->get_name_from_id(i)+" control is too large: "+
                     toString(u(i))+"A > "+toString(u_max(i))+"A", MSG_TYPE_ERROR);
          }

          s(i) = u(i)*in_out_gain(i);

          // saturate control signal
          if(s(i) > u_saturation(i))
            s(i) = u_saturation(i);
          else if(s(i) < - u_saturation(i))
            s(i) = - u_saturation(i);
        }

        // when estimating current offset set ctrl to zero
        if(m_emergency_stop_triggered || m_iter<static_cast<int>(m_currentOffsetIters))
          s.setZero();

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(i_real,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal i_real before initialization!");
          return s;
        }
        s = m_i_measuredSIN(iter);

        // compensate for current sensor offsets
        const dynamicgraph::Vector& offset = m_i_sensor_offsets_real_outSOUT(iter);
        s -= offset;

        // compensate for current sensor gains
        const dynamicgraph::Vector& K = m_i_sens_gainsSIN(iter);
        s = s.cwiseProduct(K);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(i_low_level,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal currents_low_level before initialization!");
          return s;
        }
        s = m_i_measuredSIN(iter);
        // Compensate for current sensor offsets
        const dynamicgraph::Vector& i_offsets_low_level = m_i_sensor_offsets_low_levelSIN(iter);
        s -= i_offsets_low_level;
        // Compensate for the current sensor gains
        const dynamicgraph::Vector& K = m_i_sens_gainsSIN(iter);
        s = s.cwiseProduct(K);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(i_sensor_offsets_real_out, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal i_sensor_offsets_real_out before initialization!");
          return s;
        }
        const dynamicgraph::Vector& currents = m_i_measuredSIN(iter);

        // Compute current sensor offsets
        if (m_currentOffsetIters > 0)
        {
          if(m_iter<static_cast<int>(m_currentOffsetIters))
            m_i_offsets_real += (currents-m_i_offsets_real)/(m_iter+1);
          else if(m_iter==static_cast<int>(m_currentOffsetIters))
          {
            SEND_MSG("Current sensor offsets computed in "+toString(m_iter)+" iterations: "+toString(m_i_offsets_real), MSG_TYPE_INFO);
            for(int i=0; i<s.size(); i++)
              if(fabs(m_i_offsets_real(i))>0.6)
              {
                SEND_MSG("Current offset for joint "+m_robot_util->get_name_from_id(i)+
                         " is too large, suggesting that the sensor may be broken: "+toString(m_i_offsets_real(i)), MSG_TYPE_WARNING);
                m_i_offsets_real(i) = 0.0;
              }
          }
        }
        m_iter++;

        if(m_i_sensor_offsets_real_inSIN.isPlugged())
          s = m_i_sensor_offsets_real_inSIN(iter);
        else
          s = m_i_offsets_real;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dead_zone_compensation, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal dead_zone_compensation before initialization!");
          return s;
        }
        const dynamicgraph::Vector& dead_zone_offsets         = m_dead_zone_offsetsSIN(iter);
        m_u_safeSOUT(iter);
        s = m_dz_coeff.cwiseProduct(dead_zone_offsets);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(i_errors, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal i_errors before initialization!");
          return s;
        }
        const dynamicgraph::Vector& u                         = m_uSOUT(iter);
        const dynamicgraph::Vector& currents                  = m_i_realSOUT(iter);
        s = u-currents;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(i_errors_ll_wo_bemf, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal i_errors_ll_wo_bemf before initialization!");
          return s;
        }
        const dynamicgraph::Vector& u                         = m_uSOUT(iter);
        const dynamicgraph::Vector& currents                  = m_i_realSOUT(iter);
        const dynamicgraph::Vector& bemfFactor                = m_bemf_factorSIN(iter);
        const dynamicgraph::Vector& bemf_comp_perc            = m_percentage_bemf_compensationSIN(iter);
        const dynamicgraph::Vector& dq                        = m_dqSIN(iter);

        s = u-currents - (dynamicgraph::Vector::Ones(m_robot_util->m_nbJoints)-bemf_comp_perc).cwiseProduct(bemfFactor.cwiseProduct(dq));

        for(int i=0; i<s.size(); i++)
          if(s(i)>0.0)
          {
            m_avg_i_err_pos(i) += (s(i)-m_avg_i_err_pos(i))*1e-3;
          }
          else
          {
            m_avg_i_err_neg(i) += (s(i)-m_avg_i_err_neg(i))*1e-3;
          }
        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      void CurrentController::reset_integral()
      {
        m_i_err_integr.setZero();
      }

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */


      void CurrentController::display(std::ostream& os) const
      {
        os << "CurrentController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph
