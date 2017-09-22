/*
 * Copyright 2014, Oscar E. Ramos Ponce, LAAS-CNRS
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
#include <sot/torque_control/joint-trajectory-generator.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/utils/stop-watch.hh>

#include "../include/sot/torque_control/stc-commands.hh"

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
      using namespace Eigen;

//Size to be aligned                         "-------------------------------------------------------"
#define PROFILE_POSITION_DESIRED_COMPUTATION "TrajGen: reference joint traj computation              "
#define PROFILE_FORCE_DESIRED_COMPUTATION    "TrajGen: reference force computation                   "

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef JointTrajectoryGenerator EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(JointTrajectoryGenerator,
                                         "JointTrajectoryGenerator");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      JointTrajectoryGenerator::
          JointTrajectoryGenerator(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(base6d_encoders,dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_OUT(q,   dynamicgraph::Vector, m_base6d_encodersSIN)
            ,CONSTRUCT_SIGNAL_OUT(dq,  dynamicgraph::Vector, m_qSOUT)
            ,CONSTRUCT_SIGNAL_OUT(ddq, dynamicgraph::Vector, m_qSOUT)
            ,CONSTRUCT_SIGNAL(fRightFoot, OUT, dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL(fLeftFoot,  OUT, dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL(fRightHand, OUT, dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL(fLeftHand,  OUT, dynamicgraph::Vector)
            ,m_firstIter(true)
            ,m_initSucceeded(false)
	,m_robot_util(RefVoidRobotUtil())
      {
        BIND_SIGNAL_TO_FUNCTION(fRightFoot, OUT, dynamicgraph::Vector);
        BIND_SIGNAL_TO_FUNCTION(fLeftFoot,  OUT, dynamicgraph::Vector);
        BIND_SIGNAL_TO_FUNCTION(fRightHand, OUT, dynamicgraph::Vector);
        BIND_SIGNAL_TO_FUNCTION(fLeftHand,  OUT, dynamicgraph::Vector);


        m_status_force.resize(4,JTG_STOP);
        m_minJerkTrajGen_force.resize(4);
        m_sinTrajGen_force.resize(4);
        m_linChirpTrajGen_force.resize(4);
        m_currentTrajGen_force.resize(4);
        m_noTrajGen_force.resize(4);

        m_iterForceSignals.resize(4, 0);

        Entity::signalRegistration( m_qSOUT << m_dqSOUT << m_ddqSOUT << m_base6d_encodersSIN
                                    << m_fRightFootSOUT << m_fLeftFootSOUT
                                    << m_fRightHandSOUT << m_fLeftHandSOUT);

        /* Commands. */
        addCommand("init",
                   makeCommandVoid2(*this, &JointTrajectoryGenerator::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "Time period in seconds (double)",
						    "robotRef (string)")));
        
        addCommand("getJoint",
                   makeCommandVoid1(*this, &JointTrajectoryGenerator::getJoint,
                                    docCommandVoid1("Get the current angle of the specified joint.",
                                                    "Joint name (string)")));

        //const std::string& docstring = ;

        addCommand("isTrajectoryEnded",
                   new command::IsTrajectoryEnded(*this, "Return whether all joint trajectories have ended"));

        addCommand("playTrajectoryFile",
                   makeCommandVoid1(*this, &JointTrajectoryGenerator::playTrajectoryFile,
                                    docCommandVoid1("Play the trajectory read from the specified text file.",
                                                    "(string) File name, path included")));

        addCommand("startSinusoid",
                   makeCommandVoid3(*this, &JointTrajectoryGenerator::startSinusoid,
                                    docCommandVoid3("Start an infinite sinusoid motion.",
                                                    "(string) joint name",
                                                    "(double) final angle in radians",
                                                    "(double) time to reach the final angle in sec")));

        addCommand("startTriangle",
                   makeCommandVoid4(*this, &JointTrajectoryGenerator::startTriangle,
                                    docCommandVoid4("Start an infinite triangle wave.",
                                                    "(string) joint name",
                                                    "(double) final angle in radians",
                                                    "(double) time to reach the final angle in sec",
                                                    "(double) time to accelerate in sec")));

        addCommand("startConstAcc",
                   makeCommandVoid3(*this, &JointTrajectoryGenerator::startConstAcc,
                                    docCommandVoid3("Start an infinite trajectory with piece-wise constant acceleration.",
                                                    "(string) joint name",
                                                    "(double) final angle in radians",
                                                    "(double) time to reach the final angle in sec")));

        addCommand("startForceSinusoid",
                   makeCommandVoid4(*this, &JointTrajectoryGenerator::startForceSinusoid,
                                    docCommandVoid4("Start an infinite sinusoid force.",
                                                    "(string) force name",
                                                    "(int) force axis in [0, 5]",
                                                    "(double) final 1d force in N or Nm",
                                                    "(double) time to reach the final force in sec")));

//        addCommand("startForceSinusoid",
//                   makeCommandVoid3(*this, &JointTrajectoryGenerator::startForceSinusoid,
//                                    docCommandVoid3("Start an infinite sinusoid force.",
//                                                    "(string) force name",
//                                                    "(Vector) final 6d force in N/Nm",
//                                                    "(double) time to reach the final force in sec")));

        addCommand("startLinChirp",
                   makeCommandVoid5(*this, &JointTrajectoryGenerator::startLinearChirp,
                                    docCommandVoid5("Start a linear-chirp motion.",
                                                    "(string) joint name",
                                                    "(double) final angle in radians",
                                                    "(double) initial frequency [Hz]",
                                                    "(double) final frequency [Hz]",
                                                    "(double) trajectory time [sec]")));

        addCommand("startForceLinChirp",
                   makeCommandVoid6(*this, &JointTrajectoryGenerator::startForceLinearChirp,
                                    docCommandVoid6("Start a linear-chirp force traj.",
                                                    "(string) force name",
                                                    "(int) force axis",
                                                    "(double) final force in N/Nm",
                                                    "(double) initial frequency [Hz]",
                                                    "(double) final frequency [Hz]",
                                                    "(double) trajectory time [sec]")));

        addCommand("moveJoint",
                   makeCommandVoid3(*this, &JointTrajectoryGenerator::moveJoint,
                                    docCommandVoid3("Move the joint to the specified angle with a minimum jerk trajectory.",
                                                    "(string) joint name",
                                                    "(double) final angle in radians",
                                                    "(double) time to reach the final angle in sec")));

        addCommand("moveForce",
                   makeCommandVoid4(*this, &JointTrajectoryGenerator::moveForce,
                                    docCommandVoid4("Move the force to the specified value with a minimum jerk trajectory.",
                                                    "(string) force name",
                                                    "(int) force axis",
                                                    "(double) final force in N/Nm",
                                                    "(double) time to reach the final force in sec")));

        addCommand("stop",
                   makeCommandVoid1(*this, &JointTrajectoryGenerator::stop,
                                    docCommandVoid1("Stop the motion of the specified joint, or of all joints if no joint name is specified.",
                                                    "(string) joint name")));

        addCommand("stopForce",
                   makeCommandVoid1(*this, &JointTrajectoryGenerator::stopForce,
                                    docCommandVoid1("Stop the specified force trajectort",
                                                    "(string) force name (rh,lh,lf,rf)")));
      }

      void JointTrajectoryGenerator::init(const double& dt,
					  const std::string& robotRef)
      {
	/* Retrieve m_robot_util  informations */
	std::string localName(robotRef);
	if (isNameInRobotUtil(localName))
	  m_robot_util = getRobotUtil(localName);
	else 
	  {
	    SEND_MSG("You should have an entity controller manager initialized before",MSG_TYPE_ERROR);
	    return;
	  }

        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        m_firstIter = true;
        m_dt = dt;

        m_status.resize(m_robot_util->m_nbJoints,JTG_STOP);
        m_minJerkTrajGen.resize(m_robot_util->m_nbJoints);
        m_sinTrajGen.resize(m_robot_util->m_nbJoints);
        m_triangleTrajGen.resize(m_robot_util->m_nbJoints);
        m_constAccTrajGen.resize(m_robot_util->m_nbJoints);
        m_linChirpTrajGen.resize(m_robot_util->m_nbJoints);
        m_currentTrajGen.resize(m_robot_util->m_nbJoints);
        m_noTrajGen.resize(m_robot_util->m_nbJoints);

        for(int i=0; i<m_robot_util->m_nbJoints; i++)
        {
          m_minJerkTrajGen[i]   = new MinimumJerkTrajectoryGenerator(dt,5.0,1);
          m_sinTrajGen[i]       = new SinusoidTrajectoryGenerator(dt,5.0,1);
          m_triangleTrajGen[i]  = new TriangleTrajectoryGenerator(dt,5.0,1);
          m_constAccTrajGen[i]  = new ConstantAccelerationTrajectoryGenerator(dt,5.0,1);
          m_linChirpTrajGen[i]  = new LinearChirpTrajectoryGenerator(dt,5.0,1);
          m_noTrajGen[i]        = new NoTrajectoryGenerator(1);
          m_currentTrajGen[i]   = m_noTrajGen[i];
        }
        m_textFileTrajGen = new TextFileTrajectoryGenerator(dt, m_robot_util->m_nbJoints);

        for(int i=0; i<4; i++)
        {
          m_minJerkTrajGen_force[i]   = new MinimumJerkTrajectoryGenerator(dt,5.0,6);
          m_sinTrajGen_force[i]       = new SinusoidTrajectoryGenerator(dt,5.0,6);
          m_linChirpTrajGen_force[i]  = new LinearChirpTrajectoryGenerator(dt,5.0,6);
          m_noTrajGen_force[i]        = new NoTrajectoryGenerator(6);
          m_currentTrajGen_force[i]   = m_noTrajGen_force[i];
        }
        m_initSucceeded = true;
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(q, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal positionDes before initialization!");
          return s;
        }


        getProfiler().start(PROFILE_POSITION_DESIRED_COMPUTATION);
        {
          // at first iteration store current joints positions
          if(m_firstIter)
          {
            const dynamicgraph::Vector& base6d_encoders = m_base6d_encodersSIN(iter);
            if(base6d_encoders.size()!=m_robot_util->m_nbJoints+6)
            {
              SEND_ERROR_STREAM_MSG("Unexpected size of signal base6d_encoder " + 
				    toString(base6d_encoders.size()) + " " +
				    toString(m_robot_util->m_nbJoints+6)
				    );
              return s;
            }
            for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
              m_noTrajGen[i]->set_initial_point(base6d_encoders(6+i));
            m_firstIter = false;
          }

          if(s.size()!=m_robot_util->m_nbJoints)
            s.resize(m_robot_util->m_nbJoints);

          if(m_status[0]==JTG_TEXT_FILE)
          {
            const VectorXd& qRef = m_textFileTrajGen->compute_next_point();
            for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
            {
              s(i) = qRef[i];
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
            for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
            {
              s(i) = m_currentTrajGen[i]->compute_next_point()(0);
              if(m_currentTrajGen[i]->isTrajectoryEnded())
              {
                m_currentTrajGen[i] = m_noTrajGen[i];
                m_noTrajGen[i]->set_initial_point(s(i));
                m_status[i] = JTG_STOP;
                SEND_MSG("Trajectory of joint "+
			 m_robot_util->get_name_from_id(i)+
			 " ended.", MSG_TYPE_INFO);
              }
            }
          }

        }
        getProfiler().stop(PROFILE_POSITION_DESIRED_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dq, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal positionDes before initialization!");
          return s;
        }

        const dynamicgraph::Vector& q = m_qSOUT(iter);

        if(s.size()!=m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
        if(m_status[0]==JTG_TEXT_FILE)
        {
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
            s(i) = m_textFileTrajGen->getVel()[i];
        }
        else
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
            s(i) = m_currentTrajGen[i]->getVel()(0);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(ddq, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal positionDes before initialization!");
          return s;
        }

        const dynamicgraph::Vector& q = m_qSOUT(iter);

        if(s.size()!=m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);

        if(m_status[0]==JTG_TEXT_FILE)
        {
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
            s(i) = m_textFileTrajGen->getAcc()[i];
        }
        else
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
            s(i) = m_currentTrajGen[i]->getAcc()(0);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fRightFoot, dynamicgraph::Vector)
      {
	//        SEND_MSG("Compute force right foot iter "+toString(iter), MSG_TYPE_DEBUG);
        generateReferenceForceSignal("fRightFoot", 
				     m_robot_util->m_force_util.get_force_id_right_foot(),
				     s, iter);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fLeftFoot, dynamicgraph::Vector)
      {
        generateReferenceForceSignal("fLeftFoot", 
				     m_robot_util->m_force_util.get_force_id_left_foot(),
				     s, iter);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fRightHand, dynamicgraph::Vector)
      {
        generateReferenceForceSignal("fRightHand", 
				     m_robot_util->
				     m_force_util.get_force_id_right_hand(), 
				     s, iter);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fLeftHand, dynamicgraph::Vector)
      {
        generateReferenceForceSignal("fLeftHand", 
				     m_robot_util->
				     m_force_util.get_force_id_left_hand(), 
				     s, iter);
        return s;
      }

      bool JointTrajectoryGenerator::generateReferenceForceSignal(const std::string& forceName, int fid, dynamicgraph::Vector& s, int iter)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal "+
				  forceName+" before initialization!");
          return false;
        }

        getProfiler().start(PROFILE_FORCE_DESIRED_COMPUTATION);
        {
          if(s.size()!=6)
            s.resize(6);

          if(iter>m_iterForceSignals[fid])
          {
            m_currentTrajGen_force[fid]->compute_next_point();
//            SEND_MSG("Compute force "+forceName+" with id "+toString(fid)+": "+toString(fr.transpose()), MSG_TYPE_DEBUG);
            m_iterForceSignals[fid]++;
          }

          const Eigen::VectorXd& fr = m_currentTrajGen_force[fid]->getPos();
          for(unsigned int i=0; i<6; i++)
            s(i) = fr(i);

          if(m_currentTrajGen_force[fid]->isTrajectoryEnded())
          {
            m_noTrajGen_force[fid]->set_initial_point(m_currentTrajGen_force[fid]->getPos());
            m_currentTrajGen_force[fid] = m_noTrajGen_force[fid];
            m_status_force[fid] = JTG_STOP;
            SEND_MSG("Trajectory of force "+forceName+" ended.", MSG_TYPE_INFO);
          }
        }
        getProfiler().stop(PROFILE_FORCE_DESIRED_COMPUTATION);

        return true;
      }

      /* ------------------------------------------------------------------- */
      /* --- COMMANDS ------------------------------------------------------ */
      /* ------------------------------------------------------------------- */

      void JointTrajectoryGenerator::getJoint(const std::string& jointName)
      {
        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;
        const dynamicgraph::Vector& base6d_encoders = m_base6d_encodersSIN.accessCopy();
        SEND_MSG("Current angle of joint "+jointName+" is "+toString(base6d_encoders(6+i)), MSG_TYPE_INFO);
      }

      bool JointTrajectoryGenerator::isTrajectoryEnded()
      {
        bool output=true;
        if(m_status[0]==JTG_TEXT_FILE)
        {
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
          {
            if(!m_textFileTrajGen->isTrajectoryEnded())
            {
              output=false;
              SEND_MSG("Text file trajectory not ended.", MSG_TYPE_INFO);
              return output;
            }
          }
        }
        else
        {
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
          {
            if(!m_currentTrajGen[i]->isTrajectoryEnded())
            {
              output=false;
              SEND_MSG("Trajectory of joint "+
                       m_robot_util->get_name_from_id(i)+
                       "not ended.", MSG_TYPE_INFO);
              return output;
            }
          }
        }
        return output;
      }

      void JointTrajectoryGenerator::playTrajectoryFile(const std::string& fileName)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start sinusoid before initialization!",MSG_TYPE_ERROR);

        for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
          if(m_status[i]!=JTG_STOP)
            return SEND_MSG("You cannot joint "+m_robot_util->get_name_from_id(i)+" because it is already controlled.", MSG_TYPE_ERROR);

        if(!m_textFileTrajGen->loadTextFile(fileName))
          return SEND_MSG("Error while loading text file "+fileName, MSG_TYPE_ERROR);

        // check current configuration is not too far from initial trajectory configuration
        bool needToMoveToInitConf = false;
        const VectorXd& qInit = m_textFileTrajGen->get_initial_point();
        for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
          if(fabs(qInit[i] - m_currentTrajGen[i]->getPos()(0)) > 0.001)
          {
            needToMoveToInitConf = true;
            SEND_MSG("Joint "+m_robot_util->get_name_from_id(i)+" is too far from initial configuration so first i will move it there.", MSG_TYPE_WARNING);
          }

        // if necessary move joints to initial configuration
        if(needToMoveToInitConf)
        {
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
          {
            if(!isJointInRange(i, qInit[i]))
              return;

            m_minJerkTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
            m_minJerkTrajGen[i]->set_final_point(qInit[i]);
            m_minJerkTrajGen[i]->set_trajectory_time(4.0);
            m_status[i] = JTG_MIN_JERK;
            m_currentTrajGen[i] = m_minJerkTrajGen[i];
          }
          return;
        }

        for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
        {
          m_status[i]         = JTG_TEXT_FILE;
        }
      }

      void JointTrajectoryGenerator::startSinusoid(const std::string& jointName, const double& qFinal, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start sinusoid before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified joint because it is already controlled.", MSG_TYPE_ERROR);
        if(!isJointInRange(i, qFinal))
          return;

        m_sinTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
        SEND_MSG("Set initial point of sinusoid to "+toString(m_sinTrajGen[i]->getPos()),MSG_TYPE_DEBUG);
        m_sinTrajGen[i]->set_final_point(qFinal);
        m_sinTrajGen[i]->set_trajectory_time(time);
        m_status[i]         = JTG_SINUSOID;
        m_currentTrajGen[i] = m_sinTrajGen[i];
      }

      void JointTrajectoryGenerator::startTriangle(const std::string& jointName, const double& qFinal, const double& time, const double& Tacc)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start triangle before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified joint because it is already controlled.", MSG_TYPE_ERROR);
        if(!isJointInRange(i, qFinal))
          return;

        m_triangleTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
        SEND_MSG("Set initial point of triangular trajectory to "+toString(m_triangleTrajGen[i]->getPos()),MSG_TYPE_DEBUG);
        m_triangleTrajGen[i]->set_final_point(qFinal);

        if(!m_triangleTrajGen[i]->set_trajectory_time(time))
          return SEND_MSG("Trajectory time cannot be negative.", MSG_TYPE_ERROR);

        if(!m_triangleTrajGen[i]->set_acceleration_time(Tacc))
          return SEND_MSG("Acceleration time cannot be negative or larger than half the trajectory time.", MSG_TYPE_ERROR);

        m_status[i]         = JTG_TRIANGLE;
        m_currentTrajGen[i] = m_triangleTrajGen[i];
      }

      void JointTrajectoryGenerator::startConstAcc(const std::string& jointName, const double& qFinal, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start constant-acceleration trajectory before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified joint because it is already controlled.", MSG_TYPE_ERROR);
        if(!isJointInRange(i, qFinal))
          return;

        m_constAccTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
        SEND_MSG("Set initial point of const-acc trajectory to "+toString(m_constAccTrajGen[i]->getPos()),MSG_TYPE_DEBUG);
        m_constAccTrajGen[i]->set_final_point(qFinal);
        m_constAccTrajGen[i]->set_trajectory_time(time);
        m_status[i]         = JTG_CONST_ACC;
        m_currentTrajGen[i] = m_constAccTrajGen[i];
      }

      void JointTrajectoryGenerator::startForceSinusoid(const std::string& forceName, const int& axis, const double& fFinal, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start force sinusoid before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertForceNameToForceId(forceName,i)==false)
          return;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(axis<0 || axis>5)
          return SEND_MSG("Axis must be between 0 and 5", MSG_TYPE_ERROR);
        if(m_status_force[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified force because it is already controlled.", MSG_TYPE_ERROR);
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(fFinal_eig, fFinal);
        if(!isForceInRange(i, axis, fFinal))
          return;

        VectorXd currentF = m_noTrajGen_force[i]->getPos();
        m_sinTrajGen_force[i]->set_initial_point(currentF);
        SEND_MSG("Set initial point of sinusoid to "+toString(m_sinTrajGen_force[i]->getPos()),MSG_TYPE_DEBUG);
        currentF[axis] = fFinal;
        m_sinTrajGen_force[i]->set_final_point(currentF);
        m_sinTrajGen_force[i]->set_trajectory_time(time);
        m_status_force[i]         = JTG_SINUSOID;
        m_currentTrajGen_force[i] = m_sinTrajGen_force[i];
      }

      void JointTrajectoryGenerator::startLinearChirp(const string& jointName, const double& qFinal, const double& f0, const double& f1, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start linear chirp before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified joint because it is already controlled.", MSG_TYPE_ERROR);
        if(!isJointInRange(i, qFinal))
          return;
        if(f0>f1)
          return SEND_MSG("f0 "+toString(f0)+" cannot to be more than f1 "+toString(f1),MSG_TYPE_ERROR);
        if(f0<=0.0)
          return SEND_MSG("Frequency has to be positive "+toString(f0),MSG_TYPE_ERROR);

        if(!m_linChirpTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos()))
          return SEND_MSG("Error while setting initial point "+toString(m_noTrajGen[i]->getPos()), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen[i]->set_final_point(qFinal))
          return SEND_MSG("Error while setting final point "+toString(qFinal), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen[i]->set_trajectory_time(time))
          return SEND_MSG("Error while setting trajectory time "+toString(time), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen[i]->set_initial_frequency(f0))
          return SEND_MSG("Error while setting initial frequency "+toString(f0), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen[i]->set_final_frequency(f1))
          return SEND_MSG("Error while setting final frequency "+toString(f1), MSG_TYPE_ERROR);
        m_status[i]         = JTG_LIN_CHIRP;
        m_currentTrajGen[i] = m_linChirpTrajGen[i];
      }

      void JointTrajectoryGenerator::startForceLinearChirp(const string& forceName, const int& axis, const double& fFinal, const double& f0, const double& f1, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot start force linear chirp before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertForceNameToForceId(forceName,i)==false)
          return;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status_force[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified force because it is already controlled.", MSG_TYPE_ERROR);
        if(!isForceInRange(i, axis, fFinal))
          return;
        if(f0>f1)
          return SEND_MSG("f0 "+toString(f0)+" cannot to be more than f1 "+toString(f1),MSG_TYPE_ERROR);
        if(f0<=0.0)
          return SEND_MSG("Frequency has to be positive "+toString(f0),MSG_TYPE_ERROR);

        VectorXd currentF = m_noTrajGen_force[i]->getPos();
        if(!m_linChirpTrajGen_force[i]->set_initial_point(currentF))
          return SEND_MSG("Error while setting initial point "+toString(m_noTrajGen_force[i]->getPos()), MSG_TYPE_ERROR);
        currentF[axis] = fFinal;
        if(!m_linChirpTrajGen_force[i]->set_final_point(currentF))
          return SEND_MSG("Error while setting final point "+toString(fFinal), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen_force[i]->set_trajectory_time(time))
          return SEND_MSG("Error while setting trajectory time "+toString(time), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen_force[i]->set_initial_frequency(Vector6d::Constant(f0)))
          return SEND_MSG("Error while setting initial frequency "+toString(f0), MSG_TYPE_ERROR);
        if(!m_linChirpTrajGen_force[i]->set_final_frequency(Vector6d::Constant(f1)))
          return SEND_MSG("Error while setting final frequency "+toString(f1), MSG_TYPE_ERROR);
        m_status_force[i]         = JTG_LIN_CHIRP;
        m_currentTrajGen_force[i] = m_linChirpTrajGen_force[i];
      }

      void JointTrajectoryGenerator::moveJoint(const string& jointName, const double& qFinal, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot move joint before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified joint because it is already controlled.", MSG_TYPE_ERROR);
        if(!isJointInRange(i, qFinal))
          return;

        m_minJerkTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
        m_minJerkTrajGen[i]->set_final_point(qFinal);
        m_minJerkTrajGen[i]->set_trajectory_time(time);
        m_status[i] = JTG_MIN_JERK;
        m_currentTrajGen[i] = m_minJerkTrajGen[i];
      }

      void JointTrajectoryGenerator::moveForce(const string& forceName, const int& axis, const double& fFinal, const double& time)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot move force before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertForceNameToForceId(forceName,i)==false)
          return;
        if(time<=0.0)
          return SEND_MSG("Trajectory time must be a positive number", MSG_TYPE_ERROR);
        if(m_status_force[i]!=JTG_STOP)
          return SEND_MSG("You cannot move the specified force because it is already controlled.", MSG_TYPE_ERROR);
        if(!isForceInRange(i, axis, fFinal))
          return;

        VectorXd currentF = m_noTrajGen_force[i]->getPos();
        m_minJerkTrajGen_force[i]->set_initial_point(currentF);
        currentF[axis] = fFinal;
        m_minJerkTrajGen_force[i]->set_final_point(currentF);
        m_minJerkTrajGen_force[i]->set_trajectory_time(time);
        m_status_force[i] = JTG_MIN_JERK;
        m_currentTrajGen_force[i] = m_minJerkTrajGen_force[i];
      }

      void JointTrajectoryGenerator::stop(const std::string& jointName)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot stop joint before initialization!",MSG_TYPE_ERROR);

        const dynamicgraph::Vector& base6d_encoders = m_base6d_encodersSIN.accessCopy();
        if(jointName=="all")
        {
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
          {
            m_status[i] = JTG_STOP;
            // update the initial position
            m_noTrajGen[i]->set_initial_point(base6d_encoders(6+i));
            m_currentTrajGen[i] = m_noTrajGen[i];
          }
          return;
        }

        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;
        m_noTrajGen[i]->set_initial_point(base6d_encoders(6+i));  // update the initial position
        m_status[i] = JTG_STOP;
        m_currentTrajGen[i] = m_noTrajGen[i];
      }

      void JointTrajectoryGenerator::stopForce(const std::string& forceName)
      {
        if(!m_initSucceeded)
          return SEND_MSG("Cannot stop force before initialization!",MSG_TYPE_ERROR);

        unsigned int i;
        if(convertForceNameToForceId(forceName,i)==false)
          return;
        m_noTrajGen_force[i]->set_initial_point(m_currentTrajGen_force[i]->getPos());  // update the initial position
        m_status_force[i] = JTG_STOP;
        m_currentTrajGen_force[i] = m_noTrajGen_force[i];
      }


      /* ------------------------------------------------------------------- */
      // ************************ PROTECTED MEMBER METHODS ********************
      /* ------------------------------------------------------------------- */


      bool JointTrajectoryGenerator::
      convertJointNameToJointId(const std::string& name, unsigned int& id)
      {
        // Check if the joint name exists
        int jid = m_robot_util->get_id_from_name(name);
        if (jid<0)
        {
          SEND_MSG("The specified joint name does not exist", MSG_TYPE_ERROR);
          std::stringstream ss;
          for(map<string, Index>::const_iterator it = 
		m_robot_util->m_name_to_id.begin(); 
	      it != m_robot_util->m_name_to_id.end(); it++)
            ss<<it->first<<", ";
          SEND_MSG("Possible joint names are: "+ss.str(), MSG_TYPE_INFO);
          return false;
        }
        id = jid;
        return true;
      }

      bool JointTrajectoryGenerator::
      convertForceNameToForceId(const std::string& name, unsigned int& id)
      {
        // Check if the joint name exists
        Index fid = m_robot_util->m_force_util.get_id_from_name(name);
        if (fid<0)
        {
          SEND_MSG("The specified force name does not exist", MSG_TYPE_ERROR);
          std::stringstream ss;
          for(map<string, Index>::const_iterator 
		it = m_robot_util->m_force_util.m_name_to_force_id.begin(); 
	      it != m_robot_util->m_force_util.m_name_to_force_id.end(); it++)
            ss<<it->first<<", ";
          SEND_MSG("Possible force names are: "+ss.str(), MSG_TYPE_INFO);
          return false;
        }
        id = (unsigned int)fid;
        return true;
      }

      bool JointTrajectoryGenerator::isJointInRange(unsigned int id, double q)
      {
        JointLimits jl = m_robot_util->get_joint_limits_from_id(id);
        if(q<jl.lower)
        {
          SEND_MSG("Joint "+m_robot_util->get_name_from_id(id)+", desired angle "+toString(q)+" is smaller than lower limit "+toString(jl.lower),MSG_TYPE_ERROR);
          return false;
        }
        if(q>jl.upper)
        {
          SEND_MSG("Joint "+m_robot_util->get_name_from_id(id)+", desired angle "+toString(q)+" is larger than upper limit "+toString(jl.upper),MSG_TYPE_ERROR);
          return false;
        }
        return true;
      }

      bool JointTrajectoryGenerator::isForceInRange(unsigned int id, const Eigen::VectorXd& f)
      {
        ForceLimits fl = m_robot_util->m_force_util.get_limits_from_id(id);
        for(unsigned int i=0; i<6; i++)
          if(f[i]<fl.lower[i] || f[i]>fl.upper[i])
          {
            SEND_MSG("Desired force "+toString(i)+" is out of range: "+toString(fl.lower[i])+" < "+
                     toString(f[i])+" < "+toString(fl.upper[i]),MSG_TYPE_ERROR);
            return false;
          }
        return true;
      }

      bool JointTrajectoryGenerator::isForceInRange(unsigned int id, int axis, double f)
      {
        ForceLimits fl = m_robot_util->m_force_util.get_limits_from_id(id);
        if(f<fl.lower[axis] || f>fl.upper[axis])
        {
          SEND_MSG("Desired force "+toString(axis)+" is out of range: "+toString(fl.lower[axis])+" < "+
                   toString(f)+" < "+toString(fl.upper[axis]),MSG_TYPE_ERROR);
          return false;
        }
        return true;
      }


      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void JointTrajectoryGenerator::display(std::ostream& os) const
      {
        os << "JointTrajectoryGenerator "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }


      void JointTrajectoryGenerator::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "sotJointTrajectoryGenerator:\n"
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

