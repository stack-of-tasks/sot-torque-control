/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
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
#include <sot/torque_control/imu_offset_compensation.hh>
#include <sot/torque_control/commands-helper.hh>
#include <sot/core/stop-watch.hh>


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

#define CALIBRATION_FILE_NAME "/opt/imu_calib.txt"

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
        ,m_dt(0.001f)
        ,m_update_cycles_left(0)
        ,m_update_cycles(0)
        ,m_a_gyro_DC_blocker(1.0f)

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
        addCommand("setGyroDCBlockerParameter",
                   makeCommandVoid1(*this, &ImuOffsetCompensation::setGyroDCBlockerParameter,
                                    docCommandVoid1("Set DC Blocker filter parameter.",
                                                    "alpha (double)")));


      }

      /* ------------------------------------------------------------------- */
      /* --- COMMANDS ------------------------------------------------------ */
      /* ------------------------------------------------------------------- */

      void ImuOffsetCompensation::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        m_dt = static_cast<float>(dt);
        m_initSucceeded = true;

        // try to read IMU calibration data from file
        std::ifstream infile;
        infile.open(CALIBRATION_FILE_NAME, std::ios::in);
        if(!infile.is_open())
          return SEND_MSG("Error trying to read calibration results from file "+toString(CALIBRATION_FILE_NAME), MSG_TYPE_ERROR);

        double z=0;
        int i=0;
        while(infile>>z)
        {
          m_gyro_offset(i) = z;
          i++;
          if(i==3)
            break;
        }
        if(i!=3)
        {
          m_gyro_offset.setZero();
          return SEND_MSG("Error trying to read gyro offset from file "+toString(CALIBRATION_FILE_NAME)+". Not enough values: "+toString(i), MSG_TYPE_ERROR);
        }

        i=0;
        while(infile>>z)
        {
          m_acc_offset(i) = z;
          i++;
          if(i==3)
            break;
        }
        if(i!=3)
        {
          m_gyro_offset.setZero();
          m_acc_offset.setZero();
          return SEND_MSG("Error trying to read acc offset from file "+toString(CALIBRATION_FILE_NAME)+". Not enough values: "+toString(i), MSG_TYPE_ERROR);
        }

        SEND_MSG("Offset read finished:\n* acc offset: "+toString(m_acc_offset.transpose())+
                 "\n* gyro offset: "+toString(m_gyro_offset.transpose()), MSG_TYPE_INFO);
      }

      void ImuOffsetCompensation::setGyroDCBlockerParameter(const double & alpha)
      {
        if(alpha>1.0 || alpha<=0.0)
          return SEND_MSG("GyroDCBlockerParameter must be > 0 and <= 1", MSG_TYPE_ERROR);
        m_a_gyro_DC_blocker = alpha;
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
          Vector3 g, new_acc_offset, new_gyro_offset;
          g<<0.0, 0.0, 9.81;
          new_acc_offset  = (m_acc_sum /m_update_cycles) - g;
          new_gyro_offset = m_gyro_sum/m_update_cycles;
          m_acc_sum.setZero();
          m_gyro_sum.setZero();
          SEND_MSG("Offset computation finished:"+
                   ("\n* old acc offset: "+toString(m_acc_offset.transpose()))+
                   "\n* new acc offset: "+toString(new_acc_offset.transpose())+
                   "\n* old gyro offset: "+toString(m_gyro_offset.transpose())+
                   "\n* new gyro offset: "+toString(new_gyro_offset.transpose()), MSG_TYPE_INFO);
          m_acc_offset = new_acc_offset;
          m_gyro_offset = new_gyro_offset;

          // save to text file
          ofstream aof(CALIBRATION_FILE_NAME);
          if(!aof.is_open())
            return SEND_MSG("Error trying to save calibration results on file "+toString(CALIBRATION_FILE_NAME), MSG_TYPE_ERROR);

          for(unsigned long int i=0;i<3;i++)
            aof << m_gyro_offset[i] << " " ;
          aof << std::endl;
          for(unsigned long int i=0;i<3;i++)
            aof << m_acc_offset[i] << " " ;
          aof << std::endl;
          aof.close();
          SEND_MSG("IMU calibration data saved to file "+toString(CALIBRATION_FILE_NAME), MSG_TYPE_INFO);
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
        //estimate bias online with the assumption that average angular velocity should be zero.
        if (m_a_gyro_DC_blocker !=1.0)
            m_gyro_offset = m_gyro_offset*m_a_gyro_DC_blocker + (1.-m_a_gyro_DC_blocker)*gyrometer;
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
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph
