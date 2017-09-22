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



#include <sot/torque_control/imu_offset_compensation.hh>
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

#define PROFILE_IMU_OFFSET_COMPENSATION_COMPUTATION          "ImuOffsetCompensation computation"

#define INPUT_SIGNALS     m_accelerometer_inSIN  << m_gyrometer_inSIN
#define OUTPUT_SIGNALS    m_accelerometer_outSOUT << m_gyrometer_outSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef ImuOffsetCompensation EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ImuOffsetCompensation,
                                         "ImuOffsetCompensation");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      ImuOffsetCompensation::ImuOffsetCompensation(const std::string& name)
        : Entity(name)
        ,CONSTRUCT_SIGNAL_IN( accelerometer_in, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( gyrometer_in,     dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(accelerometer_out, dynamicgraph::Vector, m_accelerometer_inSIN)
        ,CONSTRUCT_SIGNAL_OUT(gyrometer_out,     dynamicgraph::Vector, m_gyrometer_inSIN)
        ,m_initSucceeded(false)
        ,m_update_cycles_left(0)
        ,m_update_cycles(0)
        ,m_dt(0.001)

      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        m_gyro_offset.setZero();
        m_acc_offset.setZero();
        m_gyro_sum.setZero();
        m_acc_sum.setZero();

        /* Commands. */
        addCommand("init",
                   makeCommandVoid1(*this, &ImuOffsetCompensation::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Timestep in seconds (double)")));
        addCommand("update_offset",
                   makeCommandVoid1(*this, &ImuOffsetCompensation::update_offset,
                                    docCommandVoid1("Update the IMU offsets.",
                                                    "Duration of the update phase in seconds (double)")));
      }

      /* ------------------------------------------------------------------- */
      /* --- COMMANDS ------------------------------------------------------ */
      /* ------------------------------------------------------------------- */

      void ImuOffsetCompensation::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        m_dt = dt;
        m_initSucceeded = true;
      }

      void ImuOffsetCompensation::update_offset(const double& duration)
      {
        if(duration<m_dt)
          return SEND_MSG("Duration must be greater than the time step", MSG_TYPE_ERROR);
        m_update_cycles = int(duration/m_dt);
        m_update_cycles_left = m_update_cycles;
      }

      void ImuOffsetCompensation::update_offset_impl(int iter)
      {
        const dynamicgraph::Vector& accelerometer = m_accelerometer_inSIN(iter);
        const dynamicgraph::Vector& gyrometer = m_gyrometer_inSIN(iter);
        m_acc_sum  += accelerometer;
        m_gyro_sum += gyrometer;

        m_update_cycles_left--;
        if(m_update_cycles_left==0)
        {
          Vector3 g;
          g<<0.0, 0.0, 9.81;
          m_acc_offset  = (m_acc_sum /m_update_cycles) - g;
          m_gyro_offset = m_gyro_sum/m_update_cycles;
          m_acc_sum.setZero();
          m_gyro_sum.setZero();
          SEND_MSG("Offset computation finished:\n* acc offset: "+toString(m_acc_offset.transpose())+
                   "\n* gyro offset: "+toString(m_gyro_offset.transpose()), MSG_TYPE_INFO);
        }
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(accelerometer_out, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal accelerometer before initialization!");
          return s;
        }

        if(m_update_cycles_left>0)
          update_offset_impl(iter);

        const dynamicgraph::Vector& accelerometer = m_accelerometer_inSIN(iter);
        if(s.size()!=3)
          s.resize(3);
        s = accelerometer - m_acc_offset;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(gyrometer_out, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal gyrometer before initialization!");
          return s;
        }
        const dynamicgraph::Vector& gyrometer = m_gyrometer_inSIN(iter);
        if(s.size()!=3)
          s.resize(3);
        s = gyrometer - m_gyro_offset;
        return s;
      }


      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void ImuOffsetCompensation::display(std::ostream& os) const
      {
        os << "ImuOffsetCompensation "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

      void ImuOffsetCompensation::commandLine(const std::string& cmdLine,
                                     std::istringstream& cmdArgs,
                                     std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "ImuOffsetCompensation:\n"
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
