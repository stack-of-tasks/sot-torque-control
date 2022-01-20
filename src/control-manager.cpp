/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 */

#include <sot/torque_control/control-manager.hh>
#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>

#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/commands-helper.hh>

using namespace tsid;

namespace dynamicgraph {
namespace sot {
namespace torque_control {
namespace dynamicgraph = ::dynamicgraph;
using namespace dynamicgraph;
using namespace dynamicgraph::command;
using namespace std;
using namespace dg::sot::torque_control;

// Size to be aligned                          "-------------------------------------------------------"
#define PROFILE_PWM_DESIRED_COMPUTATION "Control manager                                        "
#define PROFILE_DYNAMIC_GRAPH_PERIOD "Control period                                         "

#define INPUT_SIGNALS m_i_maxSIN << m_u_maxSIN << m_tau_maxSIN << m_tauSIN << m_tau_predictedSIN << m_i_measuredSIN
#define OUTPUT_SIGNALS m_uSOUT << m_u_safeSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef ControlManager EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ControlManager, "ControlManager");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
ControlManager::ControlManager(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(i_measured, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tau, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tau_predicted, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(i_max, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(u_max, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tau_max, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(u, dynamicgraph::Vector, m_i_measuredSIN),
      CONSTRUCT_SIGNAL_OUT(u_safe, dynamicgraph::Vector, INPUT_SIGNALS << m_uSOUT),
      m_robot_util(RefVoidRobotUtil()),
      m_initSucceeded(false),
      m_emergency_stop_triggered(false),
      m_is_first_iter(true),
      m_iter(0),
      m_sleep_time(0.0) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid3(*this, &ControlManager::init,
                                      docCommandVoid3("Initialize the entity.", "Time period in seconds (double)",
                                                      "URDF file path (string)", "Robot reference (string)")));
  addCommand(
      "addCtrlMode",
      makeCommandVoid1(*this, &ControlManager::addCtrlMode,
                       docCommandVoid1("Create an input signal with name 'ctrl_x' where x is the specified name.",
                                       "Name (string)")));

  addCommand("ctrlModes", makeCommandVoid0(*this, &ControlManager::ctrlModes,
                                           docCommandVoid0("Get a list of all the available control modes.")));

  addCommand("setCtrlMode", makeCommandVoid2(*this, &ControlManager::setCtrlMode,
                                             docCommandVoid2("Set the control mode for a joint.",
                                                             "(string) joint name", "(string) control mode")));

  addCommand("getCtrlMode",
             makeCommandVoid1(*this, &ControlManager::getCtrlMode,
                              docCommandVoid1("Get the control mode of a joint.", "(string) joint name")));

  addCommand("resetProfiler",
             makeCommandVoid0(
                 *this, &ControlManager::resetProfiler,
                 docCommandVoid0("Reset the statistics computed by the profiler (print this entity to see them).")));

  addCommand("setNameToId", makeCommandVoid2(*this, &ControlManager::setNameToId,
                                             docCommandVoid2("Set map for a name to an Id", "(string) joint name",
                                                             "(double) joint id")));

  addCommand("setForceNameToForceId",
             makeCommandVoid2(*this, &ControlManager::setForceNameToForceId,
                              docCommandVoid2("Set map from a force sensor name to a force sensor Id",
                                              "(string) force sensor name", "(double) force sensor id")));

  addCommand("setJointLimitsFromId",
             makeCommandVoid3(*this, &ControlManager::setJointLimitsFromId,
                              docCommandVoid3("Set the joint limits for a given joint ID", "(double) joint id",
                                              "(double) lower limit", "(double) uppper limit")));

  addCommand(
      "setForceLimitsFromId",
      makeCommandVoid3(*this, &ControlManager::setForceLimitsFromId,
                       docCommandVoid3("Set the force limits for a given force sensor ID", "(double) force sensor id",
                                       "(double) lower limit", "(double) uppper limit")));

  addCommand("setJointsUrdfToSot",
             makeCommandVoid1(*this, &ControlManager::setJoints,
                              docCommandVoid1("Map Joints From URDF to SoT.", "Vector of integer for mapping")));

  addCommand("setRightFootSoleXYZ",
             makeCommandVoid1(*this, &ControlManager::setRightFootSoleXYZ,
                              docCommandVoid1("Set the right foot sole 3d position.", "Vector of 3 doubles")));
  addCommand("setRightFootForceSensorXYZ",
             makeCommandVoid1(*this, &ControlManager::setRightFootForceSensorXYZ,
                              docCommandVoid1("Set the right foot sensor 3d position.", "Vector of 3 doubles")));

  addCommand("setFootFrameName", makeCommandVoid2(*this, &ControlManager::setFootFrameName,
                                                  docCommandVoid2("Set the Frame Name for the Foot Name.",
                                                                  "(string) Foot name", "(string) Frame name")));
  addCommand("setHandFrameName", makeCommandVoid2(*this, &ControlManager::setHandFrameName,
                                                  docCommandVoid2("Set the Frame Name for the Hand Name.",
                                                                  "(string) Hand name", "(string) Frame name")));
  addCommand("setImuJointName",
             makeCommandVoid1(*this, &ControlManager::setImuJointName,
                              docCommandVoid1("Set the Joint Name wich IMU is attached to.", "(string) Joint name")));
  addCommand("displayRobotUtil", makeCommandVoid0(*this, &ControlManager::displayRobotUtil,
                                                  docCommandVoid0("Display the current robot util data set.")));

  addCommand("setStreamPrintPeriod", makeCommandVoid1(*this, &ControlManager::setStreamPrintPeriod,
                                                      docCommandVoid1("Set the period used for printing in streaming.",
                                                                      "Print period in seconds (double)")));

  addCommand("setSleepTime",
             makeCommandVoid1(*this, &ControlManager::setSleepTime,
                              docCommandVoid1("Set the time to sleep at every iteration (to slow down simulation).",
                                              "Sleep time in seconds (double)")));

  addCommand(
      "addEmergencyStopSIN",
      makeCommandVoid1(
          *this, &ControlManager::addEmergencyStopSIN,
          docCommandVoid1("Add emergency signal input from another entity that can stop the control if necessary.",
                          "(string) signal name : 'emergencyStop_' + name")));
}

void ControlManager::init(const double& dt, const std::string& urdfFile, const std::string& robotRef) {
  if (dt <= 0.0) return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
  m_dt = dt;
  m_emergency_stop_triggered = false;
  m_initSucceeded = true;
  vector<string> package_dirs;
  m_robot = new robots::RobotWrapper(urdfFile, package_dirs, pinocchio::JointModelFreeFlyer());
  std::string localName(robotRef);
  if (!isNameInRobotUtil(localName)) {
    m_robot_util = createRobotUtil(localName);
    SEND_MSG("createRobotUtil success\n", MSG_TYPE_INFO);
  } else {
    m_robot_util = getRobotUtil(localName);
    SEND_MSG("getRobotUtil success\n", MSG_TYPE_INFO);
  }
  SEND_MSG(m_robot_util->m_urdf_filename, MSG_TYPE_INFO);
  m_robot_util->m_urdf_filename = urdfFile;
  addCommand("getJointsUrdfToSot", makeDirectGetter(*this, &m_robot_util->m_dgv_urdf_to_sot,
                                                    docDirectSetter("Display map Joints From URDF to SoT.",
                                                                    "Vector of integer for mapping")));

  m_robot_util->m_nbJoints = m_robot->nv() - 6;
  m_jointCtrlModes_current.resize(m_robot_util->m_nbJoints);
  m_jointCtrlModes_previous.resize(m_robot_util->m_nbJoints);
  m_jointCtrlModesCountDown.resize(m_robot_util->m_nbJoints, 0);
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(u, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal u before initialization!");
    return s;
  }

  if (m_is_first_iter)
    m_is_first_iter = false;
  else
    getProfiler().stop(PROFILE_DYNAMIC_GRAPH_PERIOD);
  getProfiler().start(PROFILE_DYNAMIC_GRAPH_PERIOD);

  if (s.size() != (Eigen::VectorXd::Index)m_robot_util->m_nbJoints) s.resize(m_robot_util->m_nbJoints);

  getProfiler().start(PROFILE_PWM_DESIRED_COMPUTATION);
  {
    // trigger computation of all ctrl inputs
    for (unsigned int i = 0; i < m_ctrlInputsSIN.size(); i++) (*m_ctrlInputsSIN[i])(iter);

    int cm_id, cm_id_prev;
    for (unsigned int i = 0; i < m_robot_util->m_nbJoints; i++) {
      cm_id = m_jointCtrlModes_current[i].id;
      if (cm_id < 0) {
        SEND_MSG("You forgot to set the control mode of joint " + toString(i), MSG_TYPE_ERROR_STREAM);
        continue;
      }

      const dynamicgraph::Vector& ctrl = (*m_ctrlInputsSIN[cm_id])(iter);
      assert(ctrl.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

      if (m_jointCtrlModesCountDown[i] == 0)
        s(i) = ctrl(i);
      else {
        cm_id_prev = m_jointCtrlModes_previous[i].id;
        const dynamicgraph::Vector& ctrl_prev = (*m_ctrlInputsSIN[cm_id_prev])(iter);
        assert(ctrl_prev.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

        double alpha = m_jointCtrlModesCountDown[i] / CTRL_MODE_TRANSITION_TIME_STEP;
        //              SEND_MSG("Joint "+toString(i)+" changing ctrl mode from "+toString(cm_id_prev)+
        //                       " to "+toString(cm_id)+" alpha="+toString(alpha),MSG_TYPE_DEBUG);
        s(i) = alpha * ctrl_prev(i) + (1 - alpha) * ctrl(i);
        m_jointCtrlModesCountDown[i]--;

        if (m_jointCtrlModesCountDown[i] == 0) {
          SEND_MSG(
              "Joint " + toString(i) + " changed ctrl mode from " + toString(cm_id_prev) + " to " + toString(cm_id),
              MSG_TYPE_INFO);
          updateJointCtrlModesOutputSignal();
        }
      }
    }
  }
  getProfiler().stop(PROFILE_PWM_DESIRED_COMPUTATION);

  usleep(static_cast<unsigned int>(1e6 * m_sleep_time));
  if (m_sleep_time >= 0.1) {
    for (unsigned int i = 0; i < m_ctrlInputsSIN.size(); i++) {
      const dynamicgraph::Vector& ctrl = (*m_ctrlInputsSIN[i])(iter);
      SEND_MSG(toString(iter) + ") tau =" + toString(ctrl, 1, 4, " ") + m_ctrlModes[i], MSG_TYPE_ERROR);
    }
  }

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(u_safe, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal u_safe before initialization!");
    return s;
  }

  const dynamicgraph::Vector& u = m_uSOUT(iter);
  const dynamicgraph::Vector& tau_max = m_tau_maxSIN(iter);
  const dynamicgraph::Vector& ctrl_max = m_u_maxSIN(iter);
  const dynamicgraph::Vector& i_max = m_i_maxSIN(iter);
  const dynamicgraph::Vector& tau = m_tauSIN(iter);
  const dynamicgraph::Vector& i_real = m_i_measuredSIN(iter);
  const dynamicgraph::Vector& tau_predicted = m_tau_predictedSIN(iter);

  for (std::size_t i = 0; i < m_emergencyStopSIN.size(); i++) {
    if ((*m_emergencyStopSIN[i]).isPlugged() && (*m_emergencyStopSIN[i])(iter)) {
      m_emergency_stop_triggered = true;
      SEND_MSG("Emergency Stop has been triggered by an external entity", MSG_TYPE_ERROR);
    }
  }

  s = u;

  if (!m_emergency_stop_triggered) {
    for (unsigned int i = 0; i < m_robot_util->m_nbJoints; i++) {
      if (fabs(tau(i)) > tau_max(i)) {
        m_emergency_stop_triggered = true;
        SEND_MSG("Estimated torque " + toString(tau(i)) + " > max torque " + toString(tau_max(i)) + " for joint " +
                     m_robot_util->get_name_from_id(i),
                 MSG_TYPE_ERROR);
      }

      if (fabs(tau_predicted(i)) > tau_max(i)) {
        m_emergency_stop_triggered = true;
        SEND_MSG("Predicted torque " + toString(tau_predicted(i)) + " > max torque " + toString(tau_max(i)) +
                     " for joint " + m_robot_util->get_name_from_id(i),
                 MSG_TYPE_ERROR);
      }

      if (fabs(i_real(i)) > i_max(i)) {
        m_emergency_stop_triggered = true;
        SEND_MSG("Joint " + m_robot_util->get_name_from_id(i) +
                     " measured current is too large: " + toString(i_real(i)) + "A > " + toString(i_max(i)) + "A",
                 MSG_TYPE_ERROR);
        break;
      }

      if (fabs(u(i)) > ctrl_max(i)) {
        m_emergency_stop_triggered = true;
        SEND_MSG("Joint " + m_robot_util->get_name_from_id(i) + " desired current is too large: " + toString(u(i)) +
                     "A > " + toString(ctrl_max(i)) + "A",
                 MSG_TYPE_ERROR);
        break;
      }
    }
  }

  if (m_emergency_stop_triggered) s.setZero();

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

void ControlManager::addCtrlMode(const string& name) {
  // check there is no other control mode with the same name
  for (unsigned int i = 0; i < m_ctrlModes.size(); i++)
    if (name == m_ctrlModes[i]) return SEND_MSG("It already exists a control mode with name " + name, MSG_TYPE_ERROR);

  // create a new input signal to read the new control
  m_ctrlInputsSIN.push_back(new SignalPtr<dynamicgraph::Vector, int>(
      NULL, getClassName() + "(" + getName() + ")::input(dynamicgraph::Vector)::ctrl_" + name));

  // create a new output signal to specify which joints are controlled with the new
  // control mode
  m_jointsCtrlModesSOUT.push_back(new Signal<dynamicgraph::Vector, int>(
      getClassName() + "(" + getName() + ")::output(dynamicgraph::Vector)::joints_ctrl_mode_" + name));

  // add the new control mode to the list of available control modes
  m_ctrlModes.push_back(name);

  // register the new signals and add the new signal dependecy
  Eigen::VectorXd::Index i = m_ctrlModes.size() - 1;
  m_uSOUT.addDependency(*m_ctrlInputsSIN[i]);
  Entity::signalRegistration(*m_ctrlInputsSIN[i]);
  Entity::signalRegistration(*m_jointsCtrlModesSOUT[i]);
  updateJointCtrlModesOutputSignal();
}

void ControlManager::ctrlModes() { SEND_MSG(toString(m_ctrlModes), MSG_TYPE_INFO); }

void ControlManager::setCtrlMode(const string& jointName, const string& ctrlMode) {
  CtrlMode cm;
  if (convertStringToCtrlMode(ctrlMode, cm) == false) return;

  if (jointName == "all") {
    for (unsigned int i = 0; i < m_robot_util->m_nbJoints; i++) setCtrlMode(i, cm);
  } else {
    // decompose strings like "rk-rhp-lhp-..."
    std::stringstream ss(jointName);
    std::string item;
    std::list<int> jIdList;
    unsigned int i;
    while (std::getline(ss, item, '-')) {
      SEND_MSG("parsed joint : " + item, MSG_TYPE_INFO);
      if (convertJointNameToJointId(item, i)) jIdList.push_back(i);
    }
    for (std::list<int>::iterator it = jIdList.begin(); it != jIdList.end(); ++it) setCtrlMode(*it, cm);
  }
  updateJointCtrlModesOutputSignal();
}

void ControlManager::setCtrlMode(const int jid, const CtrlMode& cm) {
  if (m_jointCtrlModesCountDown[jid] != 0)
    return SEND_MSG("Cannot change control mode of joint " + m_robot_util->get_name_from_id(jid) +
                        " because its previous ctrl mode transition has not terminated yet: " +
                        toString(m_jointCtrlModesCountDown[jid]),
                    MSG_TYPE_ERROR);

  if (cm.id == m_jointCtrlModes_current[jid].id)
    return SEND_MSG("Cannot change control mode of joint " + m_robot_util->get_name_from_id(jid) +
                        " because it has already the specified ctrl mode",
                    MSG_TYPE_ERROR);

  if (m_jointCtrlModes_current[jid].id < 0) {
    // first setting of the control mode
    m_jointCtrlModes_previous[jid] = cm;
    m_jointCtrlModes_current[jid] = cm;
  } else {
    m_jointCtrlModesCountDown[jid] = CTRL_MODE_TRANSITION_TIME_STEP;
    m_jointCtrlModes_previous[jid] = m_jointCtrlModes_current[jid];
    m_jointCtrlModes_current[jid] = cm;
  }
}

void ControlManager::getCtrlMode(const std::string& jointName) {
  if (jointName == "all") {
    stringstream ss;
    for (unsigned int i = 0; i < m_robot_util->m_nbJoints; i++)
      ss << m_robot_util->get_name_from_id(i) << " " << m_jointCtrlModes_current[i] << "; ";
    SEND_MSG(ss.str(), MSG_TYPE_INFO);
    return;
  }

  unsigned int i;
  if (convertJointNameToJointId(jointName, i) == false) return;
  SEND_MSG("The control mode of joint " + jointName + " is " + m_jointCtrlModes_current[i].name, MSG_TYPE_INFO);
}

void ControlManager::resetProfiler() {
  getProfiler().reset_all();
  getStatistics().reset_all();
}

void ControlManager::setStreamPrintPeriod(const double& s) { setStreamPrintPeriod(s); }

void ControlManager::setSleepTime(const double& seconds) {
  if (seconds < 0.0) return SEND_MSG("Sleep time cannot be negative!", MSG_TYPE_ERROR);
  m_sleep_time = seconds;
}

void ControlManager::addEmergencyStopSIN(const string& name) {
  SEND_MSG("New emergency signal input emergencyStop_" + name + " created", MSG_TYPE_INFO);
  // create a new input signal
  m_emergencyStopSIN.push_back(
      new SignalPtr<bool, int>(NULL, getClassName() + "(" + getName() + ")::input(bool)::emergencyStop_" + name));

  // register the new signals and add the new signal dependecy
  Eigen::VectorXd::Index i = m_emergencyStopSIN.size() - 1;
  m_u_safeSOUT.addDependency(*m_emergencyStopSIN[i]);
  Entity::signalRegistration(*m_emergencyStopSIN[i]);
}

void ControlManager::setNameToId(const std::string& jointName, const double& jointId) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set joint name from joint id  before initialization!");
    return;
  }
  m_robot_util->set_name_to_id(jointName, static_cast<Eigen::VectorXd::Index>(jointId));
}

void ControlManager::setJointLimitsFromId(const double& jointId, const double& lq, const double& uq) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set joints limits from joint id  before initialization!");
    return;
  }

  m_robot_util->set_joint_limits_for_id((Index)jointId, lq, uq);
}

void ControlManager::setForceLimitsFromId(const double& jointId, const dynamicgraph::Vector& lq,
                                          const dynamicgraph::Vector& uq) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set force limits from force id  before initialization!");
    return;
  }

  m_robot_util->m_force_util.set_force_id_to_limits((Index)jointId, lq, uq);
}

void ControlManager::setForceNameToForceId(const std::string& forceName, const double& forceId) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set force sensor name from force sensor id  before initialization!");
    return;
  }

  m_robot_util->m_force_util.set_name_to_force_id(forceName, static_cast<Eigen::VectorXd::Index>(forceId));
}

void ControlManager::setJoints(const dg::Vector& urdf_to_sot) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set mapping to sot before initialization!");
    return;
  }
  m_robot_util->set_urdf_to_sot(urdf_to_sot);
}

void ControlManager::setRightFootSoleXYZ(const dynamicgraph::Vector& xyz) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set right foot sole XYZ before initialization!");
    return;
  }

  m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ = xyz;
}

void ControlManager::setRightFootForceSensorXYZ(const dynamicgraph::Vector& xyz) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set right foot force sensor XYZ before initialization!");
    return;
  }

  m_robot_util->m_foot_util.m_Right_Foot_Force_Sensor_XYZ = xyz;
}

void ControlManager::setFootFrameName(const std::string& FootName, const std::string& FrameName) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set foot frame name!");
    return;
  }
  if (FootName == "Left")
    m_robot_util->m_foot_util.m_Left_Foot_Frame_Name = FrameName;
  else if (FootName == "Right")
    m_robot_util->m_foot_util.m_Right_Foot_Frame_Name = FrameName;
  else
    SEND_WARNING_STREAM_MSG("Did not understand the foot name !" + FootName);
}

void ControlManager::setHandFrameName(const std::string& HandName, const std::string& FrameName) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set hand frame name!");
    return;
  }
  if (HandName == "Left")
    m_robot_util->m_hand_util.m_Left_Hand_Frame_Name = FrameName;
  else if (HandName == "Right")
    m_robot_util->m_hand_util.m_Right_Hand_Frame_Name = FrameName;
  else
    SEND_WARNING_STREAM_MSG("Did not understand the hand name !" + HandName);
}

void ControlManager::setImuJointName(const std::string& JointName) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set IMU joint name!");
    return;
  }
  m_robot_util->m_imu_joint_name = JointName;
}

void ControlManager::displayRobotUtil() { m_robot_util->display(std::cout); }

/* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */

void ControlManager::updateJointCtrlModesOutputSignal() {
  if (m_robot_util->m_nbJoints == 0) {
    SEND_MSG("You should call init first. The size of the vector is unknown.", MSG_TYPE_ERROR);
    return;
  }

  dynamicgraph::Vector cm(m_robot_util->m_nbJoints);
  for (unsigned int i = 0; i < m_jointsCtrlModesSOUT.size(); i++) {
    for (unsigned int j = 0; j < m_robot_util->m_nbJoints; j++) {
      cm(j) = 0;
      if ((unsigned int)m_jointCtrlModes_current[j].id == i) cm(j) = 1;

      // during the transition between two ctrl modes they both result active
      if (m_jointCtrlModesCountDown[j] > 0 && (unsigned int)m_jointCtrlModes_previous[j].id == i) cm(j) = 1;
    }
    m_jointsCtrlModesSOUT[i]->setConstant(cm);
  }
}

bool ControlManager::convertStringToCtrlMode(const std::string& name, CtrlMode& cm) {
  // Check if the ctrl mode name exists
  for (unsigned int i = 0; i < m_ctrlModes.size(); i++)
    if (name == m_ctrlModes[i]) {
      cm = CtrlMode(i, name);
      return true;
    }
  SEND_MSG("The specified control mode does not exist", MSG_TYPE_ERROR);
  SEND_MSG("Possible control modes are: " + toString(m_ctrlModes), MSG_TYPE_INFO);
  return false;
}

bool ControlManager::convertJointNameToJointId(const std::string& name, unsigned int& id) {
  // Check if the joint name exists
  sot::Index jid = m_robot_util->get_id_from_name(name);
  if (jid < 0) {
    SEND_MSG("The specified joint name does not exist: " + name, MSG_TYPE_ERROR);
    std::stringstream ss;
    for (pinocchio::Model::JointIndex it = 0; it < m_robot_util->m_nbJoints; it++)
      ss << m_robot_util->get_name_from_id(it) << ", ";
    SEND_MSG("Possible joint names are: " + ss.str(), MSG_TYPE_INFO);
    return false;
  }
  id = (unsigned int)jid;
  return true;
}

bool ControlManager::isJointInRange(unsigned int id, double q) {
  const JointLimits& JL = m_robot_util->get_joint_limits_from_id((Index)id);

  double jl = JL.lower;
  if (q < jl) {
    SEND_MSG("Desired joint angle " + toString(q) + " is smaller than lower limit: " + toString(jl), MSG_TYPE_ERROR);
    return false;
  }
  double ju = JL.upper;
  if (q > ju) {
    SEND_MSG("Desired joint angle " + toString(q) + " is larger than upper limit: " + toString(ju), MSG_TYPE_ERROR);
    return false;
  }
  return true;
}

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void ControlManager::display(std::ostream& os) const {
  os << "ControlManager " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph
