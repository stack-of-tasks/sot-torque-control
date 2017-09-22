//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 11/05/2017   T Flayols       Make it a dynamic-graph entity
//
//=====================================================================================================



#include <sot/torque_control/madgwickahrs.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
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

      typedef Eigen::Vector6d Vector6;

#define PROFILE_MADGWICKAHRS_COMPUTATION          "MadgwickAHRS computation"

#define INPUT_SIGNALS     m_accelerometerSIN << m_gyroscopeSIN
#define OUTPUT_SIGNALS    m_imu_quatSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef MadgwickAHRS EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MadgwickAHRS,
                                         "MadgwickAHRS");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      MadgwickAHRS::MadgwickAHRS(const std::string& name)
        : Entity(name)
        ,CONSTRUCT_SIGNAL_IN( accelerometer,            dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( gyroscope,                dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(imu_quat,                 dynamicgraph::Vector, m_gyroscopeSIN <<
                                                                              m_accelerometerSIN)
        ,m_initSucceeded(false)
        ,m_beta(betaDef)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init",
                   makeCommandVoid1(*this, &MadgwickAHRS::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Timestep in seconds (double)")));
        addCommand("getBeta",
                   makeDirectGetter(*this,&m_beta,
                                    docDirectGetter("Beta parameter", "double")));
        addCommand("setBeta",
                   makeCommandVoid1(*this, &MadgwickAHRS::set_beta,
                                    docCommandVoid1("Set the filter parameter beta", "double")));

      }

      void MadgwickAHRS::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        m_sampleFreq=1.0f/dt;
        m_initSucceeded = true;
      }

      void MadgwickAHRS::set_beta(const double& beta)
      {
        if(beta<0.0 || beta>1.0)
          return SEND_MSG("Beta must be in [0,1]", MSG_TYPE_ERROR);
        m_beta = beta;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(imu_quat, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal imu_quat before initialization!");
          return s;
        }
        const dynamicgraph::Vector& accelerometer = m_accelerometerSIN(iter);
        const dynamicgraph::Vector& gyroscope = m_gyroscopeSIN(iter);

        getProfiler().start(PROFILE_MADGWICKAHRS_COMPUTATION);
        {
          // Update state with new measurment
          madgwickAHRSupdateIMU(     gyroscope(0),     gyroscope(1),     gyroscope(2),
                                     accelerometer(0), accelerometer(1), accelerometer(2));
          if(s.size()!=4)
            s.resize(4);
          s(0) = m_q0;
          s(1) = m_q1;
          s(2) = m_q2;
          s(3) = m_q3;
        }
        getProfiler().stop(PROFILE_MADGWICKAHRS_COMPUTATION);
        return s;
      }


      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      // ************************ PROTECTED MEMBER METHODS ********************
      /* ------------------------------------------------------------------- */

      // Fast inverse square-root
      // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
      float MadgwickAHRS::invSqrt(float x)
      {
        /*
          float halfx = 0.5f * x;
          float y = x;
          long i = *(long*)&y;
          i = 0x5f3759df - (i>>1);
          y = *(float*)&i;
          y = y * (1.5f - (halfx * y * y));
          return y;*/
        return (1.0f/sqrt(x)); //we'r not in the 70's
      }

      // IMU algorithm update
      void MadgwickAHRS::madgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
      {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-m_q1 * gx - m_q2 * gy - m_q3 * gz);
        qDot2 = 0.5f * ( m_q0 * gx + m_q2 * gz - m_q3 * gy);
        qDot3 = 0.5f * ( m_q0 * gy - m_q1 * gz + m_q3 * gx);
        qDot4 = 0.5f * ( m_q0 * gz + m_q1 * gy - m_q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
          // Normalise accelerometer measurement
          recipNorm = invSqrt(ax * ax + ay * ay + az * az);
          ax *= recipNorm;
          ay *= recipNorm;
          az *= recipNorm;

          // Auxiliary variables to avoid repeated arithmetic
          _2q0 = 2.0f * m_q0;
          _2q1 = 2.0f * m_q1;
          _2q2 = 2.0f * m_q2;
          _2q3 = 2.0f * m_q3;
          _4q0 = 4.0f * m_q0;
          _4q1 = 4.0f * m_q1;
          _4q2 = 4.0f * m_q2;
          _8q1 = 8.0f * m_q1;
          _8q2 = 8.0f * m_q2;
          q0q0 = m_q0 * m_q0;
          q1q1 = m_q1 * m_q1;
          q2q2 = m_q2 * m_q2;
          q3q3 = m_q3 * m_q3;

          // Gradient decent algorithm corrective step
          s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
          s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * m_q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
          s2 = 4.0f * q0q0 * m_q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
          s3 = 4.0f * q1q1 * m_q3 - _2q1 * ax + 4.0f * q2q2 * m_q3 - _2q2 * ay;
          recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
          s0 *= recipNorm;
          s1 *= recipNorm;
          s2 *= recipNorm;
          s3 *= recipNorm;

          // Apply feedback step
          qDot1 -= m_beta * s0;
          qDot2 -= m_beta * s1;
          qDot3 -= m_beta * s2;
          qDot4 -= m_beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        m_q0 += qDot1 * (1.0f / m_sampleFreq);
        m_q1 += qDot2 * (1.0f / m_sampleFreq);
        m_q2 += qDot3 * (1.0f / m_sampleFreq);
        m_q3 += qDot4 * (1.0f / m_sampleFreq);

        // Normalise quaternion
        recipNorm = invSqrt(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
        m_q0 *= recipNorm;
        m_q1 *= recipNorm;
        m_q2 *= recipNorm;
        m_q3 *= recipNorm;
      }


      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void MadgwickAHRS::display(std::ostream& os) const
      {
        os << "MadgwickAHRS "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

      void MadgwickAHRS::commandLine(const std::string& cmdLine,
                                     std::istringstream& cmdArgs,
                                     std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "MadgwickAHRS:\n"
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







