/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
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

#include <sot/torque_control/control-manager.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <pininvdyn/utils/stop-watch.hpp>
#include <pininvdyn/utils/statistics.hpp>

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
      using namespace dynamicgraph::sot::torque_control;

//Size to be aligned                          "-------------------------------------------------------"
#define PROFILE_PWM_DESIRED_COMPUTATION       "Control manager                                        "
#define PROFILE_DYNAMIC_GRAPH_PERIOD          "Control period                                         "

#define SAFETY_SIGNALS m_max_currentSIN << m_max_tauSIN << m_tauSIN << m_tau_predictedSIN << m_emergencyStopSIN
#define INPUT_SIGNALS  m_base6d_encodersSIN << m_percentageDriverDeadZoneCompensationSIN << SAFETY_SIGNALS << m_signWindowsFilterSizeSIN << m_dqSIN << m_bemfFactorSIN

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef ControlManager EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ControlManager,
                                         "ControlManager");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      //to do rename 'pwm' to 'current'
      ControlManager::
      ControlManager(const std::string& name)
	: Entity(name)
	,CONSTRUCT_SIGNAL_IN(base6d_encoders,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(dq,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(bemfFactor,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(tau,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(tau_predicted,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(max_current,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(max_tau,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(percentageDriverDeadZoneCompensation,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(signWindowsFilterSize, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(emergencyStop, dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(pwmDes,               dynamicgraph::Vector, m_base6d_encodersSIN)
        ,CONSTRUCT_SIGNAL_OUT(signOfControl,        dynamicgraph::Vector, m_pwmDesSOUT)
        ,CONSTRUCT_SIGNAL_OUT(signOfControlFiltered,dynamicgraph::Vector, m_pwmDesSafeSOUT)
        ,CONSTRUCT_SIGNAL_OUT(pwmDesSafe,dynamicgraph::Vector, INPUT_SIGNALS << m_pwmDesSOUT)
	,m_robot_util(RefVoidRobotUtil())
        ,m_initSucceeded(false)
        ,m_emergency_stop_triggered(false)
        ,m_is_first_iter(true)
      {

        Entity::signalRegistration( INPUT_SIGNALS << m_pwmDesSOUT << m_pwmDesSafeSOUT << m_signOfControlFilteredSOUT << m_signOfControlSOUT);

        /* Commands. */
        addCommand("init",
                   makeCommandVoid3(*this, &ControlManager::init,
                                    docCommandVoid3("Initialize the entity.",
                                                    "Time period in seconds (double)",
						    "URDF file path (string)",
						    "Max current (double)")));
        
        addCommand("addCtrlMode",
                   makeCommandVoid1(*this, &ControlManager::addCtrlMode,
                                    docCommandVoid1("Create an input signal with name 'ctrl_x' where x is the specified name.",
                                                    "Name (string)")));

        addCommand("ctrlModes",
                   makeCommandVoid0(*this, &ControlManager::ctrlModes,
                                    docCommandVoid0("Get a list of all the available control modes.")));

        addCommand("setCtrlMode",
                   makeCommandVoid2(*this, &ControlManager::setCtrlMode,
                                    docCommandVoid2("Set the control mode for a joint.",
                                                    "(string) joint name",
                                                    "(string) control mode")));

        addCommand("getCtrlMode",
                   makeCommandVoid1(*this, &ControlManager::getCtrlMode,
                                    docCommandVoid1("Get the control mode of a joint.",
                                                    "(string) joint name")));

        addCommand("resetProfiler",
                   makeCommandVoid0(*this, &ControlManager::resetProfiler,
                                    docCommandVoid0("Reset the statistics computed by the profiler (print this entity to see them).")));

	addCommand("setDefaultMaxCurrent",
		   makeCommandVoid1(*this,&ControlManager::setDefaultMaxCurrent,
				    docCommandVoid1("Set the default max current",
						    "(double) max current >0")));

	addCommand("getDefaultMaxCurrent",
		   makeDirectGetter(*this,&m_maxCurrent,
				    docCommandVoid0("Get the default max current")));

	addCommand("setNameToId",
		   makeCommandVoid2(*this,&ControlManager::setNameToId,
				    docCommandVoid2("Set map for a name to an Id",
						    "(string) joint name",
						    "(double) joint id")));

	addCommand("setForceNameToForceId",
		   makeCommandVoid2(*this,&ControlManager::setForceNameToForceId,
				    docCommandVoid2("Set map from a force sensor name to a force sensor Id",
						    "(string) force sensor name",
						    "(double) force sensor id")));


	addCommand("setJointLimitsFromId",
		   makeCommandVoid3(*this,&ControlManager::setJointLimitsFromId,
				    docCommandVoid3("Set the joint limits for a given joint ID",
						    "(double) joint id",
						    "(double) lower limit",
						    "(double) uppper limit")));

	addCommand("setForceLimitsFromId",
		   makeCommandVoid3(*this,&ControlManager::setForceLimitsFromId,
				    docCommandVoid3("Set the force limits for a given force sensor ID",
						    "(double) force sensor id",
						    "(double) lower limit",
						    "(double) uppper limit")));

	addCommand("setJointsUrdfToSot",
		   makeCommandVoid1(*this, &ControlManager::setJoints,
                                    docCommandVoid1("Map Joints From URDF to SoT.",
                                                    "Vector of integer for mapping")));
	
	addCommand("setRightFootSoleXYZ",
		   makeCommandVoid1(*this, &ControlManager::setRightFootSoleXYZ,
                                    docCommandVoid1("Set the right foot sole XYZ.",
                                                    "Vector of double")));

	addCommand("setFootFrameName",
                   makeCommandVoid2(*this, &ControlManager::setFootFrameName,
                                    docCommandVoid2("Set the Frame Name for the Foot Name.",
                                                    "(string) Foot name",
                                                    "(string) Frame name")));
		   
        addCommand("displayRobotUtil",
                   makeCommandVoid0(*this, &ControlManager::displayRobotUtil,
                                    docCommandVoid0("Display the current robot util data set.")));

      }

      void ControlManager::init(const double& dt, 
				const std::string &urdfFile,
				const double & lmax_current)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);

	m_maxCurrent = lmax_current;

        m_dt = dt;
        m_emergency_stop_triggered = false; 
        m_initSucceeded = true;
	vector<string> package_dirs;

	m_robot = new pininvdyn::RobotWrapper(urdfFile, package_dirs, se3::JointModelFreeFlyer());
	
	std::string localName("control-manager-robot");
	if (!isNameInRobotUtil(localName))
	  {
	    m_robot_util = createRobotUtil(localName);
	  }
	else
	  {
	    m_robot_util = getRobotUtil(localName);
	  }

	m_robot_util->m_urdf_filename = urdfFile;

	addCommand("getJointsUrdfToSot",
		   makeDirectGetter(*this, &m_robot_util->m_dgv_urdf_to_sot,
                                    docDirectSetter("Display map Joints From URDF to SoT.",
                                                    "Vector of integer for mapping")));
	
	m_robot_util->m_nbJoints = m_robot->nv()-6;

	m_jointCtrlModes_current.resize(m_robot_util->m_nbJoints);
	m_jointCtrlModes_previous.resize(m_robot_util->m_nbJoints);
        m_jointCtrlModesCountDown.resize(m_robot_util->m_nbJoints,0);
        m_signIsPos.resize(m_robot_util->m_nbJoints, false);
        m_changeSignCpt.resize(m_robot_util->m_nbJoints, 0);
        m_winSizeAdapt.resize(m_robot_util->m_nbJoints, 0);

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

        if(m_is_first_iter)
          m_is_first_iter = false;
        else
          getProfiler().stop(PROFILE_DYNAMIC_GRAPH_PERIOD);
        getProfiler().start(PROFILE_DYNAMIC_GRAPH_PERIOD);

        //const Eigen::VectorXd& base6d_encoders = m_base6d_encodersSIN(iter);

        if(s.size()!=(Eigen::VectorXd::Index) m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);

        getProfiler().start(PROFILE_PWM_DESIRED_COMPUTATION);
        {
          // trigger computation of all ctrl inputs
          for(unsigned int i=0; i<m_ctrlInputsSIN.size(); i++)
            (*m_ctrlInputsSIN[i])(iter);

          int cm_id, cm_id_prev;
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
          {
            cm_id = m_jointCtrlModes_current[i].id;
            const dynamicgraph::Vector& ctrl = (*m_ctrlInputsSIN[cm_id])(iter);
            if(m_jointCtrlModesCountDown[i]==0)
              s(i) = ctrl(i);
            else
            {
              cm_id_prev = m_jointCtrlModes_previous[i].id;
              const dynamicgraph::Vector& ctrl_prev = (*m_ctrlInputsSIN[cm_id_prev])(iter);
              double alpha = m_jointCtrlModesCountDown[i]/CTRL_MODE_TRANSITION_TIME_STEP;
//              SEND_MSG("Joint "+toString(i)+" changing ctrl mode from "+toString(cm_id_prev)+
//                       " to "+toString(cm_id)+" alpha="+toString(alpha),MSG_TYPE_DEBUG);
              s(i) = alpha*ctrl_prev(i) + (1-alpha)*ctrl(i);
              m_jointCtrlModesCountDown[i]--;

              if(m_jointCtrlModesCountDown[i]==0)
              {
                SEND_MSG("Joint "+toString(i)+" changed ctrl mode from "+toString(cm_id_prev)+
                         " to "+toString(cm_id),MSG_TYPE_INFO);
                updateJointCtrlModesOutputSignal();
              }
            }
          }
        }
        getProfiler().stop(PROFILE_PWM_DESIRED_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(pwmDesSafe,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal pwmDesSafe before initialization!");
          return s;
        }

        // const dynamicgraph::Vector& base6d_encoders = m_base6d_encodersSIN(iter);
        const dynamicgraph::Vector& pwmDes          = m_pwmDesSOUT(iter);
        const dynamicgraph::Vector& tau_max         = m_max_tauSIN(iter);
        const dynamicgraph::Vector& tau             = m_tauSIN(iter);
        const dynamicgraph::Vector& tau_predicted   = m_tau_predictedSIN(iter);
        const dynamicgraph::Vector& dq              = m_dqSIN(iter);
        const dynamicgraph::Vector& bemfFactor      = m_bemfFactorSIN(iter);        
        const dynamicgraph::Vector& percentageDriverDeadZoneCompensation = m_percentageDriverDeadZoneCompensationSIN(iter);
        const dynamicgraph::Vector& signWindowsFilterSize                = m_signWindowsFilterSizeSIN(iter);
        if(s.size()!=m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);

        
        if(!m_emergency_stop_triggered)
        {
          stringstream ss;
          if(m_emergencyStopSIN.isPlugged())
          {
            if (m_emergencyStopSIN(iter))
              m_emergency_stop_triggered = true;
              SEND_MSG("Emergency Stop has been triggered by an external entity",MSG_TYPE_ERROR);
          }
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
          {
            //Trigger sign filter**********************
            /*              _    _   _________________________    _
               input ______| |__| |_|                         |__| |________
                            __________________________________
               output______|                                  |_____________
            */
            if (( (pwmDes(i) > 0) && (!m_signIsPos[i]) )   
              ||( (pwmDes(i) < 0) && ( m_signIsPos[i]) ))  //If not the same sign
            {
              m_changeSignCpt[i]++; //cpt the straight-times we disagree on sign
              //if (i == 2) printf("inc cpt=%d \t pos=%d\r\n", m_changeSignCpt[i],m_signIsPos[i]);
            }
            else
            { 
              m_changeSignCpt[i] = 0; //we agree
              //if (i == 2) printf("rst cpt=%d \t pos=%d \t win=%d\r\n", m_changeSignCpt[i],m_signIsPos[i],m_winSizeAdapt[i]);
              if (m_winSizeAdapt[i]>0) m_winSizeAdapt[i]--;  //decrese reactivity (set a smaller windows)
            }
            if (m_changeSignCpt[i] > m_winSizeAdapt[i]) 
            {
              
              m_signIsPos[i] = !m_signIsPos[i];//let's change our mind
              m_changeSignCpt[i] = 0; //we just agreed
              m_winSizeAdapt[i]  = signWindowsFilterSize(i); //be not so reactive for next event (set a large windows size)
              //if (i == 2) printf("toogle signIsPos=%d\r\n", m_signIsPos[i]);
            }
            //*****************************************

            if (pwmDes(i) == 0)
              s(i) = 0;
            else if (m_signIsPos[i])
              s(i) = (pwmDes(i) + bemfFactor(i)*dq(i) ) * FROM_CURRENT_TO_12_BIT_CTRL + percentageDriverDeadZoneCompensation(i) * DEAD_ZONE_OFFSET;
            else
              s(i) = (pwmDes(i) + bemfFactor(i)*dq(i) ) * FROM_CURRENT_TO_12_BIT_CTRL - percentageDriverDeadZoneCompensation(i) * DEAD_ZONE_OFFSET;


            if(fabs(tau(i)) > tau_max(i))
            {
              m_emergency_stop_triggered = true;
              SEND_MSG("Estimated torque "+toString(tau(i))+" > max torque "+toString(tau_max(i))+
                       " for joint "+ m_robot_util->get_name_from_id(i), MSG_TYPE_ERROR);
              SEND_MSG(", but predicted torque "+toString(tau_predicted(i))+" < "+toString(tau_max(i)), MSG_TYPE_ERROR);
              SEND_MSG(", and current "+toString(pwmDes(i))+"A < "+toString(m_maxCurrent)+"A", MSG_TYPE_ERROR);
              break;
            }

            if(fabs(tau_predicted(i)) > tau_max(i))
            {
              m_emergency_stop_triggered = true;
              SEND_MSG("Predicted torque "+toString(tau_predicted(i))+" > max torque "+toString(tau_max(i))+
                       " for joint "+m_robot_util->get_name_from_id(i), MSG_TYPE_ERROR);
              SEND_MSG(", but estimated torque "+toString(tau(i))+" < "+toString(tau_max(i)), MSG_TYPE_ERROR);
              SEND_MSG(", and current "+toString(pwmDes(i))+"A < "+toString(m_maxCurrent)+"A", MSG_TYPE_ERROR);
              break;
            }

            /// if the signal is plugged, read maxPwm from the associated signal
            /// if not use the default value
            if(m_max_currentSIN.isPlugged())
              m_maxCurrent = m_max_currentSIN(iter)(i);

            if( (fabs(pwmDes(i)) > m_maxCurrent) || 
                (fabs(s(i))      > m_maxCurrent * FROM_CURRENT_TO_12_BIT_CTRL) )
            {
              m_emergency_stop_triggered = true;
              SEND_MSG("Joint "+m_robot_util->get_name_from_id(i)+" desired current is too large: "+
                       toString(pwmDes(i))+"A > "+toString(m_maxCurrent)+"A", MSG_TYPE_ERROR);
              SEND_MSG(", but estimated torque "+toString(tau(i))+" < "+toString(tau_max(i)), MSG_TYPE_ERROR);
              SEND_MSG(", and predicted torque "+toString(tau_predicted(i))+" < "+toString(tau_max(i)), MSG_TYPE_ERROR);
              break;
            }
          }
        }

        if(m_emergency_stop_triggered)
	  s.setZero();

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(signOfControl,dynamicgraph::Vector)
      {
        const dynamicgraph::Vector& pwmDes = m_pwmDesSOUT(iter);
        if(s.size()!=(Eigen::VectorXd::Index)m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
        for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
        {
          if (pwmDes(i)>0)
            s(i)= 1;
          else if (pwmDes(i)<0)
            s(i)=-1;
          else 
            s(i)=0;
        }
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(signOfControlFiltered,dynamicgraph::Vector)
      {
        const dynamicgraph::Vector& pwmDesSafe = m_pwmDesSafeSOUT(iter);
        if(s.size()!=m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);
        for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
        {
          if (pwmDesSafe(i)==0) 
            s(i)=0;
          else if (m_signIsPos[i])
            s(i)=1;
          else 
            s(i)=-1;

        }
        return s;
      }


      /* --- COMMANDS ---------------------------------------------------------- */

      void ControlManager::addCtrlMode(const string& name)
      {
        // check there is no other control mode with the same name
        for(unsigned int i=0;i<m_ctrlModes.size(); i++)
          if(name==m_ctrlModes[i])
            return SEND_MSG("It already exists a control mode with name "+name, MSG_TYPE_ERROR);

        // create a new input signal to read the new control
        m_ctrlInputsSIN.push_back(new SignalPtr<dynamicgraph::Vector, int>(NULL,
           getClassName()+"("+getName()+")::input(dynamicgraph::Vector)::ctrl_"+name));

        // create a new output signal to specify which joints are controlled with the new
        // control mode
        m_jointsCtrlModesSOUT.push_back(new Signal<dynamicgraph::Vector, int>(
           getClassName()+"("+getName()+")::output(dynamicgraph::Vector)::joints_ctrl_mode_"+name));

        // add the new control mode to the list of available control modes
        m_ctrlModes.push_back(name);

        // register the new signals and add the new signal dependecy
	Eigen::VectorXd::Index i = m_ctrlModes.size()-1;
        m_pwmDesSOUT.addDependency(*m_ctrlInputsSIN[i]);
        Entity::signalRegistration(*m_ctrlInputsSIN[i]);
        Entity::signalRegistration(*m_jointsCtrlModesSOUT[i]);
        updateJointCtrlModesOutputSignal();
      }

      void ControlManager::ctrlModes()
      {
        SEND_MSG(toString(m_ctrlModes), MSG_TYPE_INFO);
      }


      void ControlManager::setCtrlMode(const string& jointName, const string& ctrlMode)
      {
        CtrlMode cm;
        if(convertStringToCtrlMode(ctrlMode,cm)==false)
          return;
          
        if(jointName=="all")
        {
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
            setCtrlMode(i,cm);
        }
        else
        {
          // decompose strings like "rk-rhp-lhp-..."
          std::stringstream ss(jointName);
          std::string item;
          std::list<int> jIdList;
          unsigned int i;
          while (std::getline(ss, item, '-'))
          {
            SEND_MSG("parsed joint : "+item, MSG_TYPE_INFO);
            if(convertJointNameToJointId(item,i))
              jIdList.push_back(i);
          }
          for (std::list<int>::iterator it=jIdList.begin(); it != jIdList.end(); ++it)
            setCtrlMode(*it,cm);
        }
        updateJointCtrlModesOutputSignal();
      }

      void ControlManager::setCtrlMode(const int jid, const CtrlMode& cm)
      {
        if(m_jointCtrlModesCountDown[jid]==0 && cm.id!=m_jointCtrlModes_current[jid].id)
        {
          if(m_jointCtrlModes_current[jid].id<0)
          {
            // first setting of the control mode
            m_jointCtrlModes_previous[jid] = cm;
            m_jointCtrlModes_current[jid]  = cm;
          }
          else
          {
            m_jointCtrlModesCountDown[jid] = CTRL_MODE_TRANSITION_TIME_STEP;
            m_jointCtrlModes_previous[jid] = m_jointCtrlModes_current[jid];
            m_jointCtrlModes_current[jid]  = cm;
          }
        }
        else
          SEND_MSG("Cannot change control mode of joint "+m_robot_util->get_name_from_id(jid)+
                   " because either it has already the specified ctrl mode or its previous"+
                   " ctrl mode transition has not terminated yet", MSG_TYPE_ERROR);
      }

      void ControlManager::getCtrlMode(const std::string& jointName)
      {
        if(jointName=="all")
        {
          stringstream ss;
          for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
            ss<<m_robot_util->get_name_from_id(i) <<" "
	      <<m_jointCtrlModes_current[i]<<"; ";
          SEND_MSG(ss.str(),MSG_TYPE_INFO);
          return;
        }

        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;
        SEND_MSG("The control mode of joint "+jointName+" is "+m_jointCtrlModes_current[i].name,MSG_TYPE_INFO);
      }

      void ControlManager::resetProfiler()
      {
        getProfiler().reset_all();
        getStatistics().reset_all();
      }

      void ControlManager::setDefaultMaxCurrent( const double & lDefaultMaxCurrent)
      {
	if (lDefaultMaxCurrent>=0.0)
	  m_maxCurrent = lDefaultMaxCurrent;
	else
	  SEND_MSG("Max current should be positive",MSG_TYPE_ERROR);
      }

      void ControlManager::setNameToId(const std::string &jointName,
				      const double & jointId)
      {
	if(!m_initSucceeded)
	  {
	    SEND_WARNING_STREAM_MSG("Cannot set joint name from joint id  before initialization!");
	    return;
	  }

	m_robot_util->set_name_to_id(jointName,jointId);
      }

      void ControlManager::setJointLimitsFromId( const double &jointId,
					       const double &lq,
					       const double &uq)
      {
	if(!m_initSucceeded)
	  {
	    SEND_WARNING_STREAM_MSG("Cannot set joints limits from joint id  before initialization!");
	    return;
	  }
	
	m_robot_util->set_joint_limits_for_id((Index)jointId,lq,uq);
      }

      void ControlManager::setForceLimitsFromId( const double &jointId,
						 const dynamicgraph::Vector &lq,
						 const dynamicgraph::Vector &uq)
      {
	if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot set force limits from force id  before initialization!");
          return;
        }

	m_robot_util->m_force_util.set_force_id_to_limits((Index)jointId,lq,uq);
      }

      void ControlManager::setForceNameToForceId(const std::string &forceName,
						 const double & forceId)
      {
	if(!m_initSucceeded)
	  {
	    SEND_WARNING_STREAM_MSG("Cannot set force sensor name from force sensor id  before initialization!");
	    return;
	  }

	m_robot_util->m_force_util.set_name_to_force_id(forceName,forceId);
      }

      void ControlManager::setJoints(const dg::Vector & urdf_to_sot)
      {
	if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot set mapping to sot before initialization!");
          return;
        }
	m_robot_util->set_urdf_to_sot(urdf_to_sot);
      }

      void ControlManager::setRightFootSoleXYZ( const dynamicgraph::Vector &xyz)
      {
	if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot set right foot sole XYZ before initialization!");
          return;
        }

	m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ = xyz;
      }

      void ControlManager::setFootFrameName( const std::string &FootName,
					     const std::string &FrameName)
      {
	if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot set foot frame name!");
          return;
        }
	if (FootName=="Left")
	  m_robot_util->m_foot_util.m_Left_Foot_Frame_Name = FrameName;
	else if (FootName=="Right")
	  m_robot_util->m_foot_util.m_Right_Foot_Frame_Name = FrameName;
	else 
	  SEND_WARNING_STREAM_MSG("Did not understand the foot name !" + FootName);
      }
      
      void ControlManager::displayRobotUtil()
      {
	
	m_robot_util->display(std::cout);
      }

      /* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */

      void ControlManager::updateJointCtrlModesOutputSignal()
      {
	if (m_robot_util->m_nbJoints==0)
	  {
	    SEND_MSG("You should call init first. The size of the vector is unknown.", MSG_TYPE_ERROR);
	    return;
	  }

        dynamicgraph::Vector cm(m_robot_util->m_nbJoints);
        for(unsigned int i=0; i<m_jointsCtrlModesSOUT.size(); i++)
        {
          for(unsigned int j=0; j<m_robot_util->m_nbJoints; j++)
          {
            cm(j) = 0;
            if((unsigned int)m_jointCtrlModes_current[j].id == i)
              cm(j) = 1;

            // during the transition between two ctrl modes they both result active
            if(m_jointCtrlModesCountDown[j]>0 && (unsigned int)m_jointCtrlModes_previous[j].id == i)
                cm(j) = 1;
          }
          m_jointsCtrlModesSOUT[i]->setConstant(cm);
        }

      }

      bool ControlManager::convertStringToCtrlMode(const std::string& name, CtrlMode& cm)
      {
        // Check if the ctrl mode name exists
        for(unsigned int i=0;i<m_ctrlModes.size(); i++)
          if(name==m_ctrlModes[i])
          {
            cm = CtrlMode(i,name);
            return true;
          }
        SEND_MSG("The specified control mode does not exist", MSG_TYPE_ERROR);
        SEND_MSG("Possible control modes are: "+toString(m_ctrlModes), MSG_TYPE_INFO);
        return false;
      }

      bool ControlManager::convertJointNameToJointId(const std::string& name, unsigned int& id)
      {
        // Check if the joint name exists
	se3::Model::JointIndex jid = m_robot_util->get_id_from_name(name);
        if (jid<0)
        {
          SEND_MSG("The specified joint name does not exist: "+name, MSG_TYPE_ERROR);
          std::stringstream ss;
          for(se3::Model::JointIndex it=0; it< m_robot_util->m_nbJoints;it++)
            ss<< m_robot_util->get_name_from_id(it) <<", ";
          SEND_MSG("Possible joint names are: "+ss.str(), MSG_TYPE_INFO);
          return false;
        }
        id = (unsigned int )jid;
        return true;
      }

      bool ControlManager::isJointInRange(unsigned int id, double q)
      {
	const JointLimits & JL = m_robot_util->get_limits_from_id((Index)id);

	double jl= JL.lower;
        if(q<jl)
        {
          SEND_MSG("Desired joint angle "+toString(q)+" is smaller than lower limit: "+toString(jl),MSG_TYPE_ERROR);
          return false;
        }
	double ju = JL.upper;
        if(q>ju)
        {
          SEND_MSG("Desired joint angle "+toString(q)+" is larger than upper limit: "+toString(ju),MSG_TYPE_ERROR);
          return false;
        }
        return true;
      }


      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */


      void ControlManager::display(std::ostream& os) const
      {
        os << "ControlManager "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }


      void ControlManager::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "sotControlManager:\n"
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

