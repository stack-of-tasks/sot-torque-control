/*
 * Copyright 2017,Thomas Flayols, LAAS-CNRS
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

#include <sot/torque_control/nd-trajectory-generator.hh>
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
      using namespace Eigen;

#define PROFILE_ND_POSITION_DESIRED_COMPUTATION "NdTrajGen: traj computation"

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef NdTrajectoryGenerator EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NdTrajectoryGenerator,
                                         "NdTrajectoryGenerator");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      NdTrajectoryGenerator::
          NdTrajectoryGenerator(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(initial_value,ml::Vector)
            ,CONSTRUCT_SIGNAL(x,   OUT, ml::Vector)
            ,CONSTRUCT_SIGNAL_OUT(dx,  ml::Vector, m_xSOUT)
            ,CONSTRUCT_SIGNAL_OUT(ddx, ml::Vector, m_xSOUT)
            ,m_firstIter(true)
            ,m_initSucceeded(false)
            ,m_n(1)
            ,m_iterLast(0)
      {
        BIND_SIGNAL_TO_FUNCTION(x,   OUT, ml::Vector);

        Entity::signalRegistration( m_xSOUT << m_dxSOUT << m_ddxSOUT << m_initial_valueSIN);

        /* Commands. */
        addCommand("init",
                   makeCommandVoid2(*this, &NdTrajectoryGenerator::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "Time period in seconds (double)",
                                                    "size of output vector signal (int)")));
        
        addCommand("getValue",
                   makeCommandVoid1(*this, &NdTrajectoryGenerator::getValue,
                                    docCommandVoid1("Get the current value of the specified index.",
                                                    "index (int)")));

        addCommand("playTrajectoryFile",
                   makeCommandVoid1(*this, &NdTrajectoryGenerator::playTrajectoryFile,
                                    docCommandVoid1("Play the trajectory read from the specified text file.",
                                                    "(string) File name, path included")));

        addCommand("startSinusoid",
                   makeCommandVoid3(*this, &NdTrajectoryGenerator::startSinusoid,
                                    docCommandVoid3("Start an infinite sinusoid motion.",
                                                    "(int)    index",
                                                    "(double) final value",
                                                    "(double) time to reach the final value in sec")));

        addCommand("startTriangle",
                   makeCommandVoid4(*this, &NdTrajectoryGenerator::startTriangle,
                                    docCommandVoid4("Start an infinite triangle wave.",
                                                    "(int)    index",
                                                    "(double) final values",
                                                    "(double) time to reach the final value in sec",
                                                    "(double) time to accelerate in sec")));

        addCommand("startConstAcc",
                   makeCommandVoid3(*this, &NdTrajectoryGenerator::startConstAcc,
                                    docCommandVoid3("Start an infinite trajectory with piece-wise constant acceleration.",
                                                    "(int)    index",
                                                    "(double) final values",
                                                    "(double) time to reach the final value in sec")));

        addCommand("startLinChirp",
                   makeCommandVoid5(*this, &NdTrajectoryGenerator::startLinearChirp,
                                    docCommandVoid5("Start a linear-chirp motion.",
                                                    "(int)    index",
                                                    "(double) final values",
                                                    "(double) initial frequency [Hz]",
                                                    "(double) final frequency [Hz]",
                                                    "(double) trajectory time [sec]")));
        addCommand("move",
                   makeCommandVoid3(*this, &NdTrajectoryGenerator::move,
                                    docCommandVoid3("Move component corresponding to index to the specified value with a minimum jerk trajectory.",
                                                    "(int)    index",
                                                    "(double) final values",
                                                    "(double) time to reach the final value in sec")));
        addCommand("stop",
                   makeCommandVoid1(*this, &NdTrajectoryGenerator::stop,
                                    docCommandVoid1("Stop the motion of the specified index, or of all components of the vector if index is equal to -1.",
                                                    "(int) index")));

      }

      void NdTrajectoryGenerator::init(const double& dt, const unsigned int& n)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(n<1)
          return SEND_MSG("n must be at least 1", MSG_TYPE_ERROR);
        m_firstIter = true;
        m_dt = dt;
        m_n = n;
        m_status.resize(m_n,JTG_STOP);
        m_minJerkTrajGen.resize(m_n);
        m_sinTrajGen.resize(m_n);
        m_triangleTrajGen.resize(m_n);
        m_constAccTrajGen.resize(m_n);
        m_linChirpTrajGen.resize(m_n);
        m_currentTrajGen.resize(m_n);
        m_noTrajGen.resize(m_n);
        for(int i=0; i<m_n; i++)
        {
          m_minJerkTrajGen[i]   = new MinimumJerkTrajectoryGenerator(dt,5.0,1);
          m_sinTrajGen[i]       = new SinusoidTrajectoryGenerator(dt,5.0,1);
          m_triangleTrajGen[i]  = new TriangleTrajectoryGenerator(dt,5.0,1);
          m_constAccTrajGen[i]  = new ConstantAccelerationTrajectoryGenerator(dt,5.0,1);
          m_linChirpTrajGen[i]  = new LinearChirpTrajectoryGenerator(dt,5.0,1);
          m_noTrajGen[i]        = new NoTrajectoryGenerator(1);
          m_currentTrajGen[i]   = m_noTrajGen[i];
        }
        m_textFileTrajGen = new TextFileTrajectoryGenerator(dt, n);
        m_initSucceeded = true;
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(x, ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal positionDes before initialization!");
          return s;
        }


        getProfiler().start(PROFILE_ND_POSITION_DESIRED_COMPUTATION);
        {
          if(s.size()!=m_n)
            s.resize(m_n);

          // at first iteration store initial values
          if(m_firstIter)
          {
            const ml::Vector& initial_value = m_initial_valueSIN(iter);
            if(initial_value.size()!=m_n)
            {
              SEND_MSG("Unexpected size of input signal initial_value: "+toString(initial_value.size()),
                       MSG_TYPE_ERROR);
              getProfiler().stop(PROFILE_ND_POSITION_DESIRED_COMPUTATION);
              return s;
            }
            for(unsigned int i=0; i<m_n; i++)
              m_currentTrajGen[i]->set_initial_point(initial_value(i));
            m_firstIter = false;
          }
          else if(iter == m_iterLast)
          {
            if(m_status[0]==JTG_TEXT_FILE)
            {
              for(unsigned int i=0; i<m_n; i++)
                s(i) = m_textFileTrajGen->getPos()[i];
            }
            else
              for(unsigned int i=0; i<m_n; i++)
                s(i) = m_currentTrajGen[i]->getPos()(0);
            getProfiler().stop(PROFILE_ND_POSITION_DESIRED_COMPUTATION);
            return s;
          }
          m_iterLast = iter;

          if(m_status[0]==JTG_TEXT_FILE)
          {
            const VectorXd& xRef = m_textFileTrajGen->compute_next_point();
            for(unsigned int i=0; i<m_n; i++)
            {
              s(i) = xRef[i];
              if(m_textFileTrajGen->isTrajectoryEnded())
              {
                m_noTrajGen[i]->set_initial_point(s(i));
                m_status[i] = JTG_STOP;
              }
            }
            if(m_textFileTrajGen->isTrajectoryEnded())
              SEND_MSG("Text file trajectory ended.", MSG_TYPE_INFO);
          }
          else
          {
            for(unsigned int i=0; i<m_n; i++)
            {
              s(i) = m_currentTrajGen[i]->compute_next_point()(0);
              if(m_currentTrajGen[i]->isTrajectoryEnded())
              {
                m_currentTrajGen[i] = m_noTrajGen[i];
                m_noTrajGen[i]->set_initial_point(s(i));
                m_status[i] = JTG_STOP;
                SEND_MSG("Trajectory of index "+toString(i)+" ended.", MSG_TYPE_INFO);
              }
            }
          }

        }
        getProfiler().stop(PROFILE_ND_POSITION_DESIRED_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dx, ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal positionDes before initialization!");
          return s;
        }

        const ml::Vector& x = m_xSOUT(iter);

        if(s.size()!=m_n)
          s.resize(m_n);
        if(m_status[0]==JTG_TEXT_FILE)
        {
          for(unsigned int i=0; i<m_n; i++)
            s(i) = m_textFileTrajGen->getVel()[i];
        }
        else
          for(unsigned int i=0; i<m_n; i++)
            s(i) = m_currentTrajGen[i]->getVel()(0);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(ddx, ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal positionDes before initialization!");
          return s;
        }

        const ml::Vector& x = m_xSOUT(iter);

        if(s.size()!=m_n)
          s.resize(m_n);

        if(m_status[0]==JTG_TEXT_FILE)
        {
          for(unsigned int i=0; i<m_n; i++)
            s(i) = m_textFileTrajGen->getAcc()[i];
        }
        else
          for(unsigned int i=0; i<m_n; i++)
            s(i) = m_currentTrajGen[i]->getAcc()(0);

        return s;
      }

      /* ------------------------------------------------------------------- */
      /* --- COMMANDS ------------------------------------------------------ */
      /* ------------------------------------------------------------------- */

      void NdTrajectoryGenerator::getValue(const int& id)
      {
        if(id<0 || id>=m_n)
          return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);

        SEND_MSG("Current value of component "+toString(id)+" is "+toString( m_currentTrajGen[id]->getPos()(0) ), MSG_TYPE_INFO);
      }

      void NdTrajectoryGenerator::playTrajectoryFile(const std::string& fileName)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start sinusoid before initialization!",MSG_TYPE_ERROR);

        for(unsigned int i=0; i<m_n; i++)
          if(m_status[i]!=JTG_STOP)
            return SEND_MSG("You cannot control component "+toString(i)+" because it is already controlled.", MSG_TYPE_ERROR);

        if(!m_textFileTrajGen->loadTextFile(fileName))
          return SEND_MSG("Error while loading text file "+fileName, MSG_TYPE_ERROR);

        // check current configuration is not too far from initial trajectory configuration
        bool needToMoveToInitConf = false;
        const VectorXd& xInit = m_textFileTrajGen->get_initial_point();
        for(unsigned int i=0; i<m_n; i++)
          if(fabs(xInit[i] - m_currentTrajGen[i]->getPos()(0)) > 0.001)
          {
            needToMoveToInitConf = true;
            SEND_MSG("Component "+ toString(i) +" is too far from initial configuration so first i will move it there.", MSG_TYPE_WARNING);
          }

        // if necessary move joints to initial configuration
        if(needToMoveToInitConf)
        {
          for(unsigned int i=0; i<m_n; i++)
          {
//            if(!isJointInRange(i, xInit[i]))
//              return;

            m_minJerkTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
            m_minJerkTrajGen[i]->set_final_point(xInit[i]);
            m_minJerkTrajGen[i]->set_trajectory_time(4.0);
            m_status[i] = JTG_MIN_JERK;
            m_currentTrajGen[i] = m_minJerkTrajGen[i];
          }
          return;
        }

        for(unsigned int i=0; i<m_n; i++)
        {
          m_status[i]         = JTG_TEXT_FILE;
        }
      }

      void NdTrajectoryGenerator::startSinusoid(const int& id, const double& xFinal, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start sinusoid before initialization!",MSG_TYPE_ERROR);

        if(id<0 || id>=m_n)
          return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
        unsigned  int i = id;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified component because it is already controlled.", MSG_TYPE_ERROR);
//        if(!isJointInRange(i, xFinal))
//          return;

        m_sinTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
        SEND_MSG("Set initial point of sinusoid to "+toString(m_sinTrajGen[i]->getPos()),MSG_TYPE_DEBUG);
        m_sinTrajGen[i]->set_final_point(xFinal);
        m_sinTrajGen[i]->set_trajectory_time(time);
        m_status[i]         = JTG_SINUSOID;
        m_currentTrajGen[i] = m_sinTrajGen[i];
      }

      void NdTrajectoryGenerator::startTriangle(const int& id, const double& xFinal, const double& time, const double& Tacc)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start triangle before initialization!",MSG_TYPE_ERROR);
        if(id<0 || id>=m_n)
          return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
        unsigned int i = id;
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified component because it is already controlled.", MSG_TYPE_ERROR);
//        if(!isJointInRange(i, xFinal))
//          return;

        m_triangleTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
        SEND_MSG("Set initial point of triangular trajectory to "+toString(m_triangleTrajGen[i]->getPos()),MSG_TYPE_DEBUG);
        m_triangleTrajGen[i]->set_final_point(xFinal);

        if(!m_triangleTrajGen[i]->set_trajectory_time(time))
          return SEND_MSG("Trajectory time cannot be negative.", MSG_TYPE_ERROR);

        if(!m_triangleTrajGen[i]->set_acceleration_time(Tacc))
          return SEND_MSG("Acceleration time cannot be negative or larger than half the trajectory time.", MSG_TYPE_ERROR);

        m_status[i]         = JTG_TRIANGLE;
        m_currentTrajGen[i] = m_triangleTrajGen[i];
      }

      void NdTrajectoryGenerator::startConstAcc(const int& id, const double& xFinal, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start constant-acceleration trajectory before initialization!",MSG_TYPE_ERROR);
        if(id<0 || id>=m_n)
          return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
        unsigned int i = id;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified component because it is already controlled.", MSG_TYPE_ERROR);
//        if(!isJointInRange(i, xFinal))
//          return;

        m_constAccTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
        SEND_MSG("Set initial point of const-acc trajectory to "+toString(m_constAccTrajGen[i]->getPos()),MSG_TYPE_DEBUG);
        m_constAccTrajGen[i]->set_final_point(xFinal);
        m_constAccTrajGen[i]->set_trajectory_time(time);
        m_status[i]         = JTG_CONST_ACC;
        m_currentTrajGen[i] = m_constAccTrajGen[i];
      }

      void NdTrajectoryGenerator::startLinearChirp(const int& id, const double& xFinal, const double& f0, const double& f1, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start linear chirp before initialization!",MSG_TYPE_ERROR);
        if(id<0 || id>=m_n)
          return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
        unsigned int i = id;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified component because it is already controlled.", MSG_TYPE_ERROR);
//        if(!isJointInRange(i, xFinal))
//          return;
        if(f0>f1)
          return SEND_MSG("f0 "+toString(f0)+" cannot to be more than f1 "+toString(f1),MSG_TYPE_ERROR);
        if(f0<=0.0)
          return SEND_MSG("Frequency has to be positive "+toString(f0),MSG_TYPE_ERROR);

        if(!m_linChirpTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos()))
          return SEND_MSG("Error while setting initial point "+toString(m_noTrajGen[i]->getPos()), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen[i]->set_final_point(xFinal))
          return SEND_MSG("Error while setting final point "+toString(xFinal), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen[i]->set_trajectory_time(time))
          return SEND_MSG("Error while setting trajectory time "+toString(time), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen[i]->set_initial_frequency(f0))
          return SEND_MSG("Error while setting initial frequency "+toString(f0), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen[i]->set_final_frequency(f1))
          return SEND_MSG("Error while setting final frequency "+toString(f1), MSG_TYPE_ERROR);
        m_status[i]         = JTG_LIN_CHIRP;
        m_currentTrajGen[i] = m_linChirpTrajGen[i];
      }

      void NdTrajectoryGenerator::move(const int& id, const double& xFinal, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot move value before initialization!",MSG_TYPE_ERROR);
        unsigned int i = id;
        if(id<0 || id>=m_n)
          return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified component because it is already controlled.", MSG_TYPE_ERROR);
//        if(!isJointInRange(i, xFinal))
//          return;

        m_minJerkTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
        m_minJerkTrajGen[i]->set_final_point(xFinal);
        m_minJerkTrajGen[i]->set_trajectory_time(time);
        m_status[i] = JTG_MIN_JERK;
        m_currentTrajGen[i] = m_minJerkTrajGen[i];
      }


      void NdTrajectoryGenerator::stop(const int& id)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot stop value before initialization!",MSG_TYPE_ERROR);

        const ml::Vector& initial_value = m_initial_valueSIN.accessCopy();
        if(id==-1) //Stop entire vector
        {
          for(unsigned int i=0; i<m_n; i++)
          {
            m_status[i] = JTG_STOP;
            // update the initial value
            m_noTrajGen[i]->set_initial_point(m_currentTrajGen[i]->getPos());
            m_currentTrajGen[i] = m_noTrajGen[i];
          }
          return;
        }
        if(id<0 || id>=m_n)
          return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
        unsigned int i = id;
        m_noTrajGen[i]->set_initial_point(m_currentTrajGen[i]->getPos());
        m_status[i] = JTG_STOP;
        m_currentTrajGen[i] = m_noTrajGen[i];
      }

      /* ------------------------------------------------------------------- */
      // ************************ PROTECTED MEMBER METHODS ********************
      /* ------------------------------------------------------------------- */


//      bool NdTrajectoryGenerator::isJointInRange(const int& id, double x)
//      {
//        JointLimits jl = JointUtil::get_limits_from_id(id);
//        if(x<jl.lower)
//        {
//          SEND_MSG("Desired joint angle is smaller than lower limit: "+toString(jl.lower),MSG_TYPE_ERROR);
//          return false;
//        }
//        if(x>jl.upper)
//        {
//          SEND_MSG("Desired joint angle is larger than upper limit: "+toString(jl.upper),MSG_TYPE_ERROR);
//          return false;
//        }
//        return true;
//      }

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void NdTrajectoryGenerator::display(std::ostream& os) const
      {
        os << "NdTrajectoryGenerator "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }


      void NdTrajectoryGenerator::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "sotNdTrajectoryGenerator:\n"
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

