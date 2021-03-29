/*
 * Copyright 2021, Noëlie Ramuzat, LAAS-CNRS
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

#include <sot/torque_control/posture-task.hh>

#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/math/utils.hpp>

#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>
#include "pinocchio/algorithm/center-of-mass.hpp"
#include <pinocchio/algorithm/frames.hpp>

#include <sot/torque_control/commands-helper.hh>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/debug-sot-torque-control.dat"

#define RESETDEBUG5() { std::ofstream DebugFile;  \
    DebugFile.open(DBGFILE,std::ofstream::out);   \
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app);   \
    DebugFile << __FILE__ << ":"      \
              << __FUNCTION__ << "(#"     \
              << __LINE__ << "):" << x << std::endl;  \
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;    \
    DebugFile.close();}

#define RESETDEBUG4()
#define ODEBUG4FULL(x)
#define ODEBUG4(x)

namespace dynamicgraph {
namespace sot {
namespace torque_control {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;
using namespace std;
using namespace tsid;
using namespace tsid::trajectories;
using namespace tsid::math;
using namespace tsid::contacts;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace dg::sot;

#define REQUIRE_FINITE(A) assert(is_finite(A))


#define INPUT_SIGNALS  m_posture_ref_posSIN \
  << m_posture_ref_velSIN \
  << m_posture_ref_accSIN \
  << m_kp_postureSIN \
  << m_kd_postureSIN \
  << m_w_postureSIN \
  << m_w_forcesSIN \
  << m_kp_constraintsSIN \
  << m_kd_constraintsSIN \
  << m_muSIN \
  << m_contact_pointsSIN \
  << m_contact_normalSIN \
  << m_f_minSIN \
  << m_f_max_right_footSIN \
  << m_f_max_left_footSIN \
  << m_qSIN \
  << m_vSIN \
  << m_com_measuredSIN \
  << m_active_jointsSIN

#define OUTPUT_SIGNALS m_tau_desSOUT << m_dv_desSOUT << m_v_desSOUT << m_q_desSOUT << m_comSOUT \
                       << m_right_foot_posSOUT << m_left_foot_posSOUT


/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef PostureTask EntityClassName;

typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorN;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorN6;
/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PostureTask,
                                   "PostureTask");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
PostureTask::
PostureTask(const std::string& name)
  : Entity(name)

  , CONSTRUCT_SIGNAL_IN(posture_ref_pos,             dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(posture_ref_vel,             dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(posture_ref_acc,             dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kp_posture,                  dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kd_posture,                  dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(w_posture,                   double)
  , CONSTRUCT_SIGNAL_IN(kp_constraints,              dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kd_constraints,              dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(w_forces,                    double)
  , CONSTRUCT_SIGNAL_IN(mu,                          double)
  , CONSTRUCT_SIGNAL_IN(contact_points,              dynamicgraph::Matrix)
  , CONSTRUCT_SIGNAL_IN(contact_normal,              dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(f_min,                       double)
  , CONSTRUCT_SIGNAL_IN(f_max_right_foot,            double)
  , CONSTRUCT_SIGNAL_IN(f_max_left_foot,             double)
  , CONSTRUCT_SIGNAL_IN(q,                           dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(v,                           dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(com_measured,                dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(active_joints,               dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_INNER(active_joints_checked,    dg::Vector, m_active_jointsSIN)
  , CONSTRUCT_SIGNAL_OUT(tau_des,                    dynamicgraph::Vector, INPUT_SIGNALS)
  , CONSTRUCT_SIGNAL_OUT(dv_des,                     dg::Vector, m_tau_desSOUT)
  , CONSTRUCT_SIGNAL_OUT(v_des,                      dg::Vector, m_dv_desSOUT)
  , CONSTRUCT_SIGNAL_OUT(q_des,                      dg::Vector, m_v_desSOUT)
  , CONSTRUCT_SIGNAL_OUT(com,                        dg::Vector, m_tau_desSOUT)
  , CONSTRUCT_SIGNAL_OUT(right_foot_pos,             dg::Vector, m_tau_desSOUT)
  , CONSTRUCT_SIGNAL_OUT(left_foot_pos,              dg::Vector, m_tau_desSOUT)
  , m_t(0.0)
  , m_initSucceeded(false)
  , m_enabled(false)
  , m_firstTime(true)
  , m_timeLast(0)
  , m_robot_util(RefVoidRobotUtil()) 
  , m_ctrlMode(CONTROL_OUTPUT_VELOCITY){
  RESETDEBUG5();
  Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

  m_com_offset.setZero();

  /* Commands. */
  addCommand("init",
             makeCommandVoid2(*this, &PostureTask::init,
                              docCommandVoid2("Initialize the entity.",
                                              "Time period in seconds (double)",
                                              "Robot reference (string)")));
  /* SET of control output type. */
  addCommand("setControlOutputType",
             makeCommandVoid1(*this, &PostureTask::setControlOutputType,
                              docCommandVoid1("Set the type of control output.",
                                              "Control type: velocity or torque (string)")));

  addCommand("updateComOffset",
             makeCommandVoid1(*this, &PostureTask::updateComOffset,
                              docCommandVoid1("Update the offset on the CoM based on its measurement.",
                                              "Measured CoM")));

}

void PostureTask::updateComOffset(const dg::Vector& com_measured) {
  const Vector3 & com = m_robot->com(m_invDyn->data());
  m_com_offset = com_measured - com;
  SEND_MSG("CoM offset updated: " + toString(m_com_offset), MSG_TYPE_INFO);
}

void PostureTask::setControlOutputType(const std::string& type) {
  for (int i = 0; i < CONTROL_OUTPUT_SIZE; i++)
    if (type == ControlOutput_s[i]) {
      m_ctrlMode = (ControlOutput)i;
      sotDEBUG(25) << "Control output type: " << ControlOutput_s[i] << endl;
      return;
    }
  sotDEBUG(25) << "Unrecognized control output type: " << type << endl;
}

void PostureTask::init(const double& dt, const std::string& robotRef) {
  if (dt <= 0.0)
    return SEND_MSG("Init failed: Timestep must be positive", MSG_TYPE_ERROR);

  /* Retrieve m_robot_util  informations */
  std::string localName(robotRef);
  if (isNameInRobotUtil(localName))
    m_robot_util = getRobotUtil(localName);
  else {
    SEND_MSG("You should have an entity controller manager initialized before", MSG_TYPE_ERROR);
    return;
  }
  const Eigen::Matrix<double, 3, 4>& contactPoints = m_contact_pointsSIN(0);
  const Eigen::Vector3d& contactNormal = m_contact_normalSIN(0);
  const dg::sot::Vector6d& kp_contact = m_kp_constraintsSIN(0);
  const dg::sot::Vector6d& kd_contact = m_kd_constraintsSIN(0);
  const double& w_forces = m_w_forcesSIN(0);
  const double& mu = m_muSIN(0);
  const double& fMin = m_f_minSIN(0);
  const double& fMaxRF = m_f_max_right_footSIN(0);
  const double& fMaxLF = m_f_max_left_footSIN(0);
  const VectorN& kp_posture = m_kp_postureSIN(0);
  const VectorN& kd_posture = m_kd_postureSIN(0);

  assert(kp_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  assert(kd_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  m_w_posture = m_w_postureSIN(0);

  try {
    vector<string> package_dirs;
    m_robot = new robots::RobotWrapper(m_robot_util->m_urdf_filename,
                                       package_dirs,
                                       pinocchio::JointModelFreeFlyer());

    assert(m_robot->nv() >= 6);
    m_robot_util->m_nbJoints = m_robot->nv() - 6;

    m_q_sot.setZero(m_robot->nv());
    m_v_sot.setZero(m_robot->nv());
    m_dv_sot.setZero(m_robot->nv());
    m_tau_sot.setZero(m_robot->nv() - 6);
    // m_f.setZero(24);
    m_q_urdf.setZero(m_robot->nq());
    m_v_urdf.setZero(m_robot->nv());
    m_dv_urdf.setZero(m_robot->nv());

    m_invDyn = new InverseDynamicsFormulationAccForce("invdyn", *m_robot);

    // CONTACT 6D TASKS
    m_contactRF = new Contact6d("contact_rfoot", *m_robot, m_robot_util->m_foot_util.m_Right_Foot_Frame_Name,
                                contactPoints, contactNormal, mu, fMin, fMaxRF);
    m_contactRF->Kp(kp_contact);
    m_contactRF->Kd(kd_contact);
    m_invDyn->addRigidContact(*m_contactRF, w_forces);

    m_contactLF = new Contact6d("contact_lfoot", *m_robot, m_robot_util->m_foot_util.m_Left_Foot_Frame_Name,
                                contactPoints, contactNormal, mu, fMin, fMaxLF);
    m_contactLF->Kp(kp_contact);
    m_contactLF->Kd(kd_contact);
    m_invDyn->addRigidContact(*m_contactLF, w_forces);

    // POSTURE TASK
    m_taskPosture = new TaskJointPosture("task-posture", *m_robot);
    m_taskPosture->Kp(kp_posture);
    m_taskPosture->Kd(kd_posture);
    m_invDyn->addMotionTask(*m_taskPosture, m_w_posture, 1);

    // ACTUATION BOUNDS TASK
    Vector tau_max = 0.8 * m_robot->model().effortLimit.tail(m_robot->nv() - 6);
    m_taskActBounds = new TaskActuationBounds("task-actuation-bounds", *m_robot);
    m_taskActBounds->setBounds(-tau_max, tau_max);
    m_invDyn->addActuationTask(*m_taskActBounds, 1.0, 0);

    // TRAJECTORY INIT
    m_samplePosture = TrajectorySample(m_robot->nv() - 6);

    m_frame_id_rf = (int)m_robot->model().getFrameId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
    m_frame_id_lf = (int)m_robot->model().getFrameId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);

    // COM OFFSET
    if (m_com_measuredSIN.isPlugged()){
      const VectorN6& q_robot = m_qSIN(0);
      assert(q_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
      const VectorN6& v_robot = m_vSIN(0);
      assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
      const dg::Vector& com_measured = m_com_measuredSIN(0);
      assert(com_measured.size() == 3);
      SEND_MSG("COM_measured: " + toString(com_measured), MSG_TYPE_ERROR);

      m_robot_util->config_sot_to_urdf(q_robot, m_q_urdf);
      m_robot_util->velocity_sot_to_urdf(m_q_urdf, v_robot, m_v_urdf);

      pinocchio::Data data = pinocchio::Data(m_robot->model());
      pinocchio::centerOfMass(m_robot->model(), data, m_q_urdf, m_v_urdf);
      m_com_offset = com_measured - data.com[0];
      SEND_MSG("Update COM: " + toString(m_com_offset), MSG_TYPE_ERROR);
    }

    m_hqpSolver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                    "eiquadprog-fast");
    m_hqpSolver->resize(m_invDyn->nVar(), m_invDyn->nEq(), m_invDyn->nIn());

  } catch (const std::exception& e) {
    std::cout << e.what();
    return SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
  }
  m_dt = dt;
  m_initSucceeded = true;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */
/** Copy active_joints only if a valid transition occurs. (From all OFF) or (To all OFF)**/
DEFINE_SIGNAL_INNER_FUNCTION(active_joints_checked, dynamicgraph::Vector) {
  if (s.size() != static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints))
    s.resize(m_robot_util->m_nbJoints);

  const Eigen::VectorXd& active_joints_sot = m_active_jointsSIN(iter);
  if (m_enabled == false) {
    if (active_joints_sot.any()) {
      /* from all OFF to some ON */
      m_enabled = true ;
      s = active_joints_sot;
      Eigen::VectorXd active_joints_urdf(m_robot_util->m_nbJoints);
      m_robot_util->joints_sot_to_urdf(active_joints_sot, active_joints_urdf);

      m_taskBlockedJoints = new TaskJointPosture("task-posture-blocked", *m_robot);
      Eigen::VectorXd blocked_joints(m_robot_util->m_nbJoints);
      for (unsigned int i = 0; i < m_robot_util->m_nbJoints; i++)
        if (active_joints_urdf(i) == 0.0)
          blocked_joints(i) = 1.0;
        else
          blocked_joints(i) = 0.0;
      SEND_MSG("Blocked joints: " + toString(blocked_joints.transpose()), MSG_TYPE_INFO);
      m_taskBlockedJoints->setMask(blocked_joints);
      TrajectorySample ref_zero(static_cast<unsigned int>(m_robot_util->m_nbJoints));
      m_taskBlockedJoints->setReference(ref_zero);
      m_invDyn->addMotionTask(*m_taskBlockedJoints, 1.0, 0);
    }
  } else if (!active_joints_sot.any()) {
    /* from some ON to all OFF */
    m_enabled = false ;
  }
  if (m_enabled == false)
    for (unsigned int i = 0; i < m_robot_util->m_nbJoints; i++)
      s(i) = false;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(tau_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal tau_des before initialization!");
    return s;
  }
  if (s.size() != static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints))
    s.resize(m_robot_util->m_nbJoints);

  m_active_joints_checkedSINNER(iter);

  const VectorN6& q_robot = m_qSIN(iter);
  assert(q_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN6& v_robot = m_vSIN(iter);
  assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));

  const VectorN& q_ref =   m_posture_ref_posSIN(iter);
  assert(q_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& dq_ref =  m_posture_ref_velSIN(iter);
  assert(dq_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& ddq_ref = m_posture_ref_accSIN(iter);
  assert(ddq_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  const VectorN& kp_posture = m_kp_postureSIN(iter);
  assert(kp_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& kd_posture = m_kd_postureSIN(iter);
  assert(kd_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  const double & w_posture = m_w_postureSIN(iter);


  // Update tasks
  m_robot_util->joints_sot_to_urdf(q_ref, m_samplePosture.pos);
  m_robot_util->joints_sot_to_urdf(dq_ref, m_samplePosture.vel);
  m_robot_util->joints_sot_to_urdf(ddq_ref, m_samplePosture.acc);
  m_taskPosture->setReference(m_samplePosture);
  m_taskPosture->Kp(kp_posture);
  m_taskPosture->Kd(kd_posture);
  if (m_w_posture != w_posture) {
    m_w_posture = w_posture;
    m_invDyn->updateTaskWeight(m_taskPosture->name(), w_posture);
  }


  if (m_firstTime) {
    m_firstTime = false;    
    m_robot_util->config_sot_to_urdf(q_robot, m_q_urdf);
    m_robot_util->velocity_sot_to_urdf(m_q_urdf, v_robot, m_v_urdf);
    
    m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
    pinocchio::SE3 H_lf = m_robot->position(
        m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
    m_contactLF->setReference(H_lf);
    SEND_MSG("Setting left foot reference to " + toString(H_lf), MSG_TYPE_DEBUG);

    pinocchio::SE3 H_rf = m_robot->position(
        m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
    m_contactRF->setReference(H_rf);
    SEND_MSG("Setting right foot reference to " + toString(H_rf), MSG_TYPE_DEBUG);

  } else if (m_timeLast != static_cast<unsigned int>(iter - 1)) {
    SEND_MSG("Last time " + toString(m_timeLast) + " is not current time-1: " + toString(iter), MSG_TYPE_ERROR);
    if (m_timeLast == static_cast<unsigned int>(iter)) {
      s = m_tau_sot;
      return s;
    }
  }
  else if (m_ctrlMode == CONTROL_OUTPUT_TORQUE){
    // In velocity close the TSID loop on itself (v_des, q_des), in torque on the (q,v) of the robot.
    m_robot_util->config_sot_to_urdf(q_robot, m_q_urdf);
    m_robot_util->velocity_sot_to_urdf(m_q_urdf, v_robot, m_v_urdf);
  }

  m_timeLast = static_cast<unsigned int>(iter);

  const HQPData & hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
  SolverHQPBase * solver = m_hqpSolver;
  const HQPOutput & sol = solver->solve(hqpData);

  if (sol.status != HQP_STATUS_OPTIMAL) {
    SEND_ERROR_STREAM_MSG("HQP solver failed to find a solution at iter " + toString(iter) + " : "+ toString(sol.status));
    SEND_ERROR_STREAM_MSG(tsid::solvers::HQPDataToString(hqpData, false));
    SEND_ERROR_STREAM_MSG("q=" + toString(q_robot));
    SEND_ERROR_STREAM_MSG("v=" + toString(v_robot));
    s.setZero();
    return s;
  }

  m_dv_urdf = m_invDyn->getAccelerations(sol);
  m_robot_util->velocity_urdf_to_sot(m_q_urdf, m_dv_urdf, m_dv_sot);
  m_robot_util->joints_urdf_to_sot(m_invDyn->getActuatorForces(sol), m_tau_sot);
  m_t += m_dt;

  s = m_tau_sot;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dv_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dv_des before initialization!");
    return s;
  }
  if (s.size() != m_robot->nv())
    s.resize(m_robot->nv());
  m_tau_desSOUT(iter);
  s = m_dv_sot;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(v_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal v_des before initialization!");
    return s;
  }
  if (s.size() != m_robot->nv())
    s.resize(m_robot->nv());
  m_dv_desSOUT(iter);
  tsid::math::Vector v_mean;
  v_mean = m_v_urdf + 0.5 * m_dt * m_dv_urdf;
  m_v_urdf = m_v_urdf + m_dt * m_dv_urdf;
  m_q_urdf = pinocchio::integrate(m_robot->model(), m_q_urdf, m_dt * v_mean);
  m_robot_util->velocity_urdf_to_sot(m_q_urdf, m_v_urdf, m_v_sot);
  s = m_v_sot;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(q_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal q_des before initialization!");
    return s;
  }
  if (s.size() != m_robot->nv())
    s.resize(m_robot->nv());
  m_v_desSOUT(iter);
  m_robot_util->config_urdf_to_sot(m_q_urdf, m_q_sot);
  s = m_q_sot;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(com, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal com before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  if (s.size() != 3) s.resize(3);
  const Vector3& com = m_robot->com(m_invDyn->data());
  s = com + m_com_offset;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_foot_pos, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal left_foot_pos before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  m_tau_desSOUT(iter);
  pinocchio::SE3 oMi;
  s.resize(7);
  m_robot->framePosition(m_invDyn->data(), m_frame_id_lf, oMi);
  tsid::math::SE3ToXYZQUAT(oMi, s);
  // tsid::math::SE3ToVector(oMi, s);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_foot_pos, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal rigt_foot_pos before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  m_tau_desSOUT(iter);
  pinocchio::SE3 oMi;
  s.resize(7);
  m_robot->framePosition(m_invDyn->data(), m_frame_id_rf, oMi);
  tsid::math::SE3ToXYZQUAT(oMi, s);
  // tsid::math::SE3ToVector(oMi, s);
  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void PostureTask::display(std::ostream& os) const {
  os << "PostureTask " << getName();
  try {
    os << "QP size: nVar " << m_invDyn->nVar() << " nEq " << m_invDyn->nEq() << " nIn " << m_invDyn->nIn() << "\n";
  } catch (ExceptionSignal e) {}
}
} // namespace torquecontrol
} // namespace sot
} // namespace dynamicgraph
