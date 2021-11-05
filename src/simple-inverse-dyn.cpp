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

#include <boost/test/unit_test.hpp>

#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/math/utils.hpp>

#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>
#include "pinocchio/algorithm/center-of-mass.hpp"
#include <pinocchio/algorithm/frames.hpp>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/simple-inverse-dyn.hh>

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

//Size to be aligned                "-------------------------------------------------------"
#define PROFILE_TAU_DES_COMPUTATION "SimpleInverseDyn: desired tau"
#define PROFILE_HQP_SOLUTION        "SimpleInverseDyn: HQP"
#define PROFILE_PREPARE_INV_DYN     "SimpleInverseDyn: prepare inv-dyn"
#define PROFILE_READ_INPUT_SIGNALS  "SimpleInverseDyn: read input signals"

#define ZERO_FORCE_THRESHOLD 1e-3

#define INPUT_SIGNALS  m_posture_ref_posSIN \
  << m_posture_ref_velSIN \
  << m_posture_ref_accSIN \
  << m_kp_postureSIN \
  << m_kd_postureSIN \
  << m_w_postureSIN \
  << m_kp_posSIN \
  << m_kd_posSIN \
  << m_tau_measuredSIN \
  << m_kp_tauSIN \
  << m_com_ref_posSIN \
  << m_com_ref_velSIN \
  << m_com_ref_accSIN \
  << m_kp_comSIN \
  << m_kd_comSIN \
  << m_w_comSIN \
  << m_kp_contactSIN \
  << m_kd_contactSIN \
  << m_w_forcesSIN \
  << m_muSIN \
  << m_contact_pointsSIN \
  << m_contact_normalSIN \
  << m_f_minSIN \
  << m_f_maxSIN \
  << m_waist_ref_posSIN \
  << m_waist_ref_velSIN \
  << m_waist_ref_accSIN \
  << m_kp_waistSIN \
  << m_kd_waistSIN \
  << m_w_waistSIN \
  << m_qSIN \
  << m_vSIN \
  << m_com_measuredSIN \
  << m_active_jointsSIN

#define OUTPUT_SIGNALS m_tau_desSOUT << m_dv_desSOUT << m_v_desSOUT << m_q_desSOUT << m_uSOUT << m_comSOUT \
                       << m_right_foot_posSOUT << m_left_foot_posSOUT


/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SimpleInverseDyn EntityClassName;

typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorN;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorN6;
/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleInverseDyn,
                                   "SimpleInverseDyn");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SimpleInverseDyn::
SimpleInverseDyn(const std::string& name)
  : Entity(name)

  , CONSTRUCT_SIGNAL_IN(posture_ref_pos,             dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(posture_ref_vel,             dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(posture_ref_acc,             dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kp_posture,                  dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kd_posture,                  dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(w_posture,                   double)
  , CONSTRUCT_SIGNAL_IN(kp_pos,                      dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kd_pos,                      dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(tau_measured,                dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kp_tau,                      dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(com_ref_pos,                 dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(com_ref_vel,                 dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(com_ref_acc,                 dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kp_com,                      dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kd_com,                      dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(w_com,                       double)
  , CONSTRUCT_SIGNAL_IN(kp_contact,                  dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kd_contact,                  dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(w_forces,                    double)
  , CONSTRUCT_SIGNAL_IN(mu,                          double)
  , CONSTRUCT_SIGNAL_IN(contact_points,              dynamicgraph::Matrix)
  , CONSTRUCT_SIGNAL_IN(contact_normal,              dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(f_min,                       double)
  , CONSTRUCT_SIGNAL_IN(f_max,                       double)
  , CONSTRUCT_SIGNAL_IN(waist_ref_pos,               dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(waist_ref_vel,               dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(waist_ref_acc,               dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kp_waist,                    dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(kd_waist,                    dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(w_waist,                     double)
  , CONSTRUCT_SIGNAL_IN(q,                           dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(v,                           dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(com_measured,                dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(active_joints,               dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_INNER(active_joints_checked,    dg::Vector, m_active_jointsSIN)
  , CONSTRUCT_SIGNAL_OUT(tau_des,                    dynamicgraph::Vector, INPUT_SIGNALS)
  , CONSTRUCT_SIGNAL_OUT(dv_des,                     dg::Vector, m_tau_desSOUT)
  , CONSTRUCT_SIGNAL_OUT(v_des,                      dg::Vector, m_dv_desSOUT)
  , CONSTRUCT_SIGNAL_OUT(q_des,                      dg::Vector, m_v_desSOUT)
  , CONSTRUCT_SIGNAL_OUT(u,                          dg::Vector, INPUT_SIGNALS << m_tau_desSOUT << m_v_desSOUT << m_q_desSOUT)
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
             makeCommandVoid2(*this, &SimpleInverseDyn::init,
                              docCommandVoid2("Initialize the entity.",
                                              "Time period in seconds (double)",
                                              "Robot reference (string)")));
  /* SET of control output type. */
  addCommand("setControlOutputType",
             makeCommandVoid1(*this, &SimpleInverseDyn::setControlOutputType,
                              docCommandVoid1("Set the type of control output.",
                                              "Control type: velocity or torque (string)")));

  addCommand("updateComOffset",
             makeCommandVoid1(*this, &SimpleInverseDyn::updateComOffset,
                              docCommandVoid1("Update the offset on the CoM based on its measurement.",
                                              "Measured CoM")));

}

void SimpleInverseDyn::updateComOffset(const dg::Vector& com_measured) {
  const Vector3 & com = m_robot->com(m_invDyn->data());
  m_com_offset = com_measured - com;
  SEND_MSG("CoM offset updated: " + toString(m_com_offset), MSG_TYPE_INFO);
}

void SimpleInverseDyn::setControlOutputType(const std::string& type) {
  for (int i = 0; i < CONTROL_OUTPUT_SIZE; i++)
    if (type == ControlOutput_s[i]) {
      m_ctrlMode = (ControlOutput)i;
      sotDEBUG(25) << "Control output type: " << ControlOutput_s[i] << endl;
      return;
    }
  sotDEBUG(25) << "Unrecognized control output type: " << type << endl;
}

void SimpleInverseDyn::init(const double& dt, const std::string& robotRef) {
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
  const dg::sot::Vector6d& kp_contact = m_kp_contactSIN(0);
  const dg::sot::Vector6d& kd_contact = m_kd_contactSIN(0);
  const Eigen::Vector3d& kp_com = m_kp_comSIN(0);
  const Eigen::Vector3d& kd_com = m_kd_comSIN(0);
  const VectorN& kp_posture = m_kp_postureSIN(0);
  const VectorN& kd_posture = m_kd_postureSIN(0);
  const dg::sot::Vector6d& kp_waist = m_kp_waistSIN(0);
  const dg::sot::Vector6d& kd_waist = m_kd_waistSIN(0);

  assert(kp_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  assert(kd_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  m_w_com = m_w_comSIN(0);
  m_w_posture = m_w_postureSIN(0);
  m_w_waist = m_w_waistSIN(0);
  const double & w_forces = m_w_forcesSIN(0);
  const double & mu = m_muSIN(0);
  const double & fMin = m_f_minSIN(0);
  const double & fMax = m_f_maxSIN(0);

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
    m_contactRF = new Contact6d("contact_rfoot", *m_robot,
                                m_robot_util->m_foot_util.m_Right_Foot_Frame_Name,
                                contactPoints, contactNormal,
                                mu, fMin, fMax);
    m_contactRF->Kp(kp_contact);
    m_contactRF->Kd(kd_contact);
    m_invDyn->addRigidContact(*m_contactRF, w_forces);

    m_contactLF = new Contact6d("contact_lfoot", *m_robot,
                                m_robot_util->m_foot_util.m_Left_Foot_Frame_Name,
                                contactPoints, contactNormal,
                                mu, fMin, fMax);
    m_contactLF->Kp(kp_contact);
    m_contactLF->Kd(kd_contact);
    m_invDyn->addRigidContact(*m_contactLF, w_forces);

    // TASK COM
    m_taskCom = new TaskComEquality("task-com", *m_robot);
    m_taskCom->Kp(kp_com);
    m_taskCom->Kd(kd_com);
    m_invDyn->addMotionTask(*m_taskCom, m_w_com, 0);

    // WAIST Task
    m_taskWaist = new TaskSE3Equality("task-waist", *m_robot, "root_joint");
    m_taskWaist->Kp(kp_waist);
    m_taskWaist->Kd(kd_waist);
    // Add a Mask to the task which will select the vector dimensions on which the task will act.
    // In this case the waist configuration is a vector 6d (position and orientation -> SE3)
    // Here we set a mask = [0 0 0 1 1 1] so the task on the waist will act on the orientation of the robot
    Eigen::VectorXd mask_orientation(6);
    mask_orientation << 0, 0, 0, 1, 1, 1;
    m_taskWaist->setMask(mask_orientation);
    // Add the task to the HQP with weight = 1.0, priority level = 1 (in the cost function) and a transition duration = 0.0
    m_invDyn->addMotionTask(*m_taskWaist, m_w_waist, 1);

    // POSTURE TASK
    m_taskPosture = new TaskJointPosture("task-posture", *m_robot);
    m_taskPosture->Kp(kp_posture);
    m_taskPosture->Kd(kd_posture);
    m_invDyn->addMotionTask(*m_taskPosture, m_w_posture, 1);

    // TRAJECTORY INIT
    m_sampleCom = TrajectorySample(3);
    m_sampleWaist = TrajectorySample(6);
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
      m_taskBlockedJoints->mask(blocked_joints);
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

  getProfiler().start(PROFILE_TAU_DES_COMPUTATION);

  getProfiler().start(PROFILE_READ_INPUT_SIGNALS);

  m_active_joints_checkedSINNER(iter);

  const VectorN6& q_robot = m_qSIN(iter);
  assert(q_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN6& v_robot = m_vSIN(iter);
  assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));

  const Vector3& x_com_ref =   m_com_ref_posSIN(iter);
  const Vector3& dx_com_ref =  m_com_ref_velSIN(iter);
  const Vector3& ddx_com_ref = m_com_ref_accSIN(iter);
  const VectorN& x_waist_ref =   m_waist_ref_posSIN(iter);
  const Vector6& dx_waist_ref =  m_waist_ref_velSIN(iter);
  const Vector6& ddx_waist_ref = m_waist_ref_accSIN(iter);
  const VectorN& q_ref =   m_posture_ref_posSIN(iter);
  assert(q_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& dq_ref =  m_posture_ref_velSIN(iter);
  assert(dq_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& ddq_ref = m_posture_ref_accSIN(iter);
  assert(ddq_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  const Vector6& kp_contact = m_kp_contactSIN(iter);
  const Vector6& kd_contact = m_kd_contactSIN(iter);
  const Vector3& kp_com = m_kp_comSIN(iter);
  const Vector3& kd_com = m_kd_comSIN(iter);
  const Vector6& kp_waist = m_kp_waistSIN(iter);
  const Vector6& kd_waist = m_kd_waistSIN(iter);

  const VectorN& kp_posture = m_kp_postureSIN(iter);
  assert(kp_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& kd_posture = m_kd_postureSIN(iter);
  assert(kd_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  /*const double & w_hands = m_w_handsSIN(iter);*/
  const double & w_com = m_w_comSIN(iter);
  const double & w_posture = m_w_postureSIN(iter);
  const double & w_forces = m_w_forcesSIN(iter);
  const double & w_waist = m_w_waistSIN(iter);

  getProfiler().stop(PROFILE_READ_INPUT_SIGNALS);

  getProfiler().start(PROFILE_PREPARE_INV_DYN);

  // Update tasks
  m_sampleCom.pos = x_com_ref - m_com_offset;
  m_sampleCom.vel = dx_com_ref;
  m_sampleCom.acc = ddx_com_ref;
  m_taskCom->setReference(m_sampleCom);
  m_taskCom->Kp(kp_com);
  m_taskCom->Kd(kd_com);
  if (m_w_com != w_com) {
    m_w_com = w_com;
    m_invDyn->updateTaskWeight(m_taskCom->name(), w_com);
  }

  m_sampleWaist.pos = x_waist_ref;
  m_sampleWaist.vel = dx_waist_ref;
  m_sampleWaist.acc = ddx_waist_ref;
  m_taskWaist->setReference(m_sampleWaist);
  m_taskWaist->Kp(kp_waist);
  m_taskWaist->Kd(kd_waist);
  if (m_w_waist != w_waist) {
    m_w_waist = w_waist;
    m_invDyn->updateTaskWeight(m_taskWaist->name(), w_waist);
  }

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

  const double & fMin = m_f_minSIN(0);
  const double & fMax = m_f_maxSIN(iter);
  m_contactLF->setMinNormalForce(fMin);
  m_contactRF->setMinNormalForce(fMin);
  m_contactLF->setMaxNormalForce(fMax);
  m_contactRF->setMaxNormalForce(fMax);
  m_contactLF->Kp(kp_contact);
  m_contactLF->Kd(kd_contact);
  m_contactRF->Kp(kp_contact);
  m_contactRF->Kd(kd_contact);
  m_invDyn->updateRigidContactWeights(m_contactLF->name(), w_forces);
  m_invDyn->updateRigidContactWeights(m_contactRF->name(), w_forces);

  if (m_firstTime) {
    m_firstTime = false;    
    m_robot_util->config_sot_to_urdf(q_robot, m_q_urdf);
    m_robot_util->velocity_sot_to_urdf(m_q_urdf, v_robot, m_v_urdf);
    
    m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);

    pinocchio::SE3 H_lf = m_robot->position(m_invDyn->data(),
                                            m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
    m_contactLF->setReference(H_lf);
    SEND_MSG("Setting left foot reference to " + toString(H_lf), MSG_TYPE_INFO);

    pinocchio::SE3 H_rf = m_robot->position(m_invDyn->data(),
                                            m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
    m_contactRF->setReference(H_rf);
    SEND_MSG("Setting right foot reference to " + toString(H_rf), MSG_TYPE_INFO);
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

  getProfiler().stop(PROFILE_PREPARE_INV_DYN);

  getProfiler().start(PROFILE_HQP_SOLUTION);
  SolverHQPBase * solver = m_hqpSolver;
  getStatistics().store("solver dynamic size", 1.0);

  const HQPOutput & sol = solver->solve(hqpData);
  getProfiler().stop(PROFILE_HQP_SOLUTION);

  if (sol.status != HQP_STATUS_OPTIMAL) {
    SEND_ERROR_STREAM_MSG("HQP solver failed to find a solution at iter " + toString(iter) + " : "+ toString(sol.status));
    SEND_ERROR_STREAM_MSG(tsid::solvers::HQPDataToString(hqpData, false));
    SEND_ERROR_STREAM_MSG("q=" + toString(q_robot));
    SEND_ERROR_STREAM_MSG("v=" + toString(v_robot));
    s.setZero();
    return s;
  }

  getStatistics().store("active inequalities", static_cast<double>(sol.activeSet.size()));
  getStatistics().store("solver iterations", sol.iterations);

  m_dv_urdf = m_invDyn->getAccelerations(sol);
  m_robot_util->velocity_urdf_to_sot(m_q_urdf, m_dv_urdf, m_dv_sot);
  m_robot_util->joints_urdf_to_sot(m_invDyn->getActuatorForces(sol), m_tau_sot);

  getProfiler().stop(PROFILE_TAU_DES_COMPUTATION);
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

DEFINE_SIGNAL_OUT_FUNCTION(u, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal u before initialization!");
    return s;
  }
  if (s.size() != static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints))
    s.resize(m_robot_util->m_nbJoints);

  const VectorN& kp_pos = m_kp_posSIN(iter);
  assert(kp_pos.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& kd_pos = m_kd_posSIN(iter);
  assert(kd_pos.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& kp_tau = m_kp_tauSIN(iter);
  assert(kp_tau.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  const VectorN6& q_robot = m_qSIN(iter);
  assert(q_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN6& v_robot = m_vSIN(iter);
  assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN6& tau_measured = m_tau_measuredSIN(iter);
  assert(tau_measured.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  m_q_desSOUT(iter);

  s = m_tau_sot +
      kp_pos.cwiseProduct(m_q_sot.tail(m_robot_util->m_nbJoints) - q_robot.tail(m_robot_util->m_nbJoints)) +
      kd_pos.cwiseProduct(m_v_sot.tail(m_robot_util->m_nbJoints) - v_robot.tail(m_robot_util->m_nbJoints));

  s += kp_tau.cwiseProduct(tau_measured - s);

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

void SimpleInverseDyn::display(std::ostream& os) const {
  os << "SimpleInverseDyn " << getName();
  try {
    getProfiler().report_all(3, os);
    getStatistics().report_all(1, os);
    os << "QP size: nVar " << m_invDyn->nVar() << " nEq " << m_invDyn->nEq() << " nIn " << m_invDyn->nIn() << "\n";
  } catch (ExceptionSignal e) {}
}
} // namespace torquecontrol
} // namespace sot
} // namespace dynamicgraph
