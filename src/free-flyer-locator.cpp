/*
 * Copyright 2017, Thomas Flayols, LAAS-CNRS
 *
 */

#include <sot/torque_control/free-flyer-locator.hh>
#include "pinocchio/algorithm/frames.hpp"

#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/commands-helper.hh>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
namespace torque_control {
namespace dynamicgraph = ::dynamicgraph;
using namespace dynamicgraph;
using namespace dynamicgraph::command;
using namespace std;
using namespace pinocchio;

typedef dynamicgraph::sot::Vector6d Vector6;

#define PROFILE_FREE_FLYER_COMPUTATION "Free-flyer position computation"
#define PROFILE_FREE_FLYER_VELOCITY_COMPUTATION "Free-flyer velocity computation"

#define INPUT_SIGNALS m_base6d_encodersSIN << m_joint_velocitiesSIN
#define OUTPUT_SIGNALS m_freeflyer_aaSOUT << m_base6dFromFoot_encodersSOUT << m_vSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef FreeFlyerLocator EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FreeFlyerLocator, "FreeFlyerLocator");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
FreeFlyerLocator::FreeFlyerLocator(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(base6d_encoders, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(joint_velocities, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_INNER(kinematics_computations, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(freeflyer_aa, dynamicgraph::Vector, m_base6dFromFoot_encodersSOUT),
      CONSTRUCT_SIGNAL_OUT(base6dFromFoot_encoders, dynamicgraph::Vector, m_kinematics_computationsSINNER),
      CONSTRUCT_SIGNAL_OUT(v, dynamicgraph::Vector, m_kinematics_computationsSINNER),
      m_initSucceeded(false),
      m_model(0),
      m_data(0),
      m_robot_util(RefVoidRobotUtil()) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &FreeFlyerLocator::init,
                                      docCommandVoid1("Initialize the entity.", "Robot reference (string)")));

  addCommand(
      "displayRobotUtil",
      makeCommandVoid0(*this, &FreeFlyerLocator::displayRobotUtil,
                       docCommandVoid0("Display the robot util data set linked with this free flyer locator.")));
}
FreeFlyerLocator::~FreeFlyerLocator() {
  if (m_model) delete m_model;
  if (m_data) delete m_data;
}

void FreeFlyerLocator::init(const std::string& robotRef) {
  try {
    /* Retrieve m_robot_util  informations */
    std::string localName(robotRef);
    if (isNameInRobotUtil(localName)) {
      m_robot_util = getRobotUtil(localName);
      std::cerr << "m_robot_util:" << m_robot_util << std::endl;
    } else {
      SEND_MSG("You should have an entity controller manager initialized before", MSG_TYPE_ERROR);
      return;
    }

    m_model = new pinocchio::Model();
    m_model->name.assign("EmptyRobot");

    pinocchio::urdf::buildModel(m_robot_util->m_urdf_filename, pinocchio::JointModelFreeFlyer(), *m_model);
    assert(m_model->nv == static_cast<int>(m_robot_util->m_nbJoints + 6));
    assert(m_model->existFrame(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
    assert(m_model->existFrame(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
    m_left_foot_id = m_model->getFrameId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);
    m_right_foot_id = m_model->getFrameId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
    m_q_pin.setZero(m_model->nq);
    m_q_pin[6] = 1.;  // for quaternion
    m_q_sot.setZero(m_robot_util->m_nbJoints + 6);
    m_v_pin.setZero(m_robot_util->m_nbJoints + 6);
    m_v_sot.setZero(m_robot_util->m_nbJoints + 6);
  } catch (const std::exception& e) {
    std::cout << e.what();
    return SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
  }
  m_data = new pinocchio::Data(*m_model);
  m_initSucceeded = true;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_INNER_FUNCTION(kinematics_computations, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal kinematics_computations before initialization!");
    return s;
  }

  const Eigen::VectorXd& q = m_base6d_encodersSIN(iter);  // n+6
  const Eigen::VectorXd& dq = m_joint_velocitiesSIN(iter);
  assert(q.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6) &&
         "Unexpected size of signal base6d_encoder");
  assert(dq.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints) &&
         "Unexpected size of signal joint_velocities");

  /* convert sot to pinocchio joint order */
  m_robot_util->joints_sot_to_urdf(q.tail(m_robot_util->m_nbJoints), m_q_pin.tail(m_robot_util->m_nbJoints));
  m_robot_util->joints_sot_to_urdf(dq, m_v_pin.tail(m_robot_util->m_nbJoints));

  /* Compute kinematic and return q with freeflyer */
  pinocchio::forwardKinematics(*m_model, *m_data, m_q_pin, m_v_pin);
  pinocchio::framesForwardKinematics(*m_model, *m_data);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(base6dFromFoot_encoders, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal base6dFromFoot_encoders before initialization!");
    return s;
  }
  if (s.size() != static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6))
    s.resize(m_robot_util->m_nbJoints + 6);

  m_kinematics_computationsSINNER(iter);

  getProfiler().start(PROFILE_FREE_FLYER_COMPUTATION);
  {
    const Eigen::VectorXd& q = m_base6d_encodersSIN(iter);  // n+6
    assert(q.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6) &&
           "Unexpected size of signal base6d_encoder");

    /* Compute kinematic and return q with freeflyer */
    const pinocchio::SE3 iMo1(m_data->oMf[m_left_foot_id].inverse());
    const pinocchio::SE3 iMo2(m_data->oMf[m_right_foot_id].inverse());
    // Average in SE3
    const pinocchio::SE3::Vector3 w(0.5 * (pinocchio::log3(iMo1.rotation()) + pinocchio::log3(iMo2.rotation())));
    m_Mff = pinocchio::SE3(pinocchio::exp3(w), 0.5 * (iMo1.translation() + iMo2.translation()));

    // due to distance from ankle to ground
    Eigen::Map<const Eigen::Vector3d> righ_foot_sole_xyz(&m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ[0]);

    m_q_sot.tail(m_robot_util->m_nbJoints) = q.tail(m_robot_util->m_nbJoints);
    base_se3_to_sot(m_Mff.translation() - righ_foot_sole_xyz, m_Mff.rotation(), m_q_sot.head<6>());

    s = m_q_sot;
  }
  getProfiler().stop(PROFILE_FREE_FLYER_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(freeflyer_aa, dynamicgraph::Vector) {
  m_base6dFromFoot_encodersSOUT(iter);
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal freeflyer_aa before initialization!");
    return s;
  }
  // oMi is has been calulated before since we depend on base6dFromFoot_encoders signal.
  // just read the data, convert to axis angle
  if (s.size() != 6) s.resize(6);
  //~ const pinocchio::SE3 & iMo = m_data->oMi[31].inverse();
  const Eigen::AngleAxisd aa(m_Mff.rotation());
  dynamicgraph::sot::Vector6d freeflyer;
  freeflyer << m_Mff.translation(), aa.axis() * aa.angle();

  // due to distance from ankle to ground
  Eigen::Map<const Eigen::Vector3d> righ_foot_sole_xyz(&m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ[0]);
  freeflyer.head<3>() -= righ_foot_sole_xyz;

  s = freeflyer;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(v, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal v before initialization!");
    return s;
  }
  if (s.size() != static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6))
    s.resize(m_robot_util->m_nbJoints + 6);

  m_kinematics_computationsSINNER(iter);

  getProfiler().start(PROFILE_FREE_FLYER_VELOCITY_COMPUTATION);
  {
    const Eigen::VectorXd& dq = m_joint_velocitiesSIN(iter);
    assert(dq.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints) &&
           "Unexpected size of signal joint_velocities");

    /* Compute foot velocities */
    const Frame& f_lf = m_model->frames[m_left_foot_id];
    const Motion v_lf_local = f_lf.placement.actInv(m_data->v[f_lf.parent]);
    const Vector6 v_lf = m_data->oMf[m_left_foot_id].act(v_lf_local).toVector();

    const Frame& f_rf = m_model->frames[m_right_foot_id];
    const Motion v_rf_local = f_rf.placement.actInv(m_data->v[f_rf.parent]);
    const Vector6 v_rf = m_data->oMf[m_right_foot_id].act(v_rf_local).toVector();

    m_v_sot.head<6>() = -0.5 * (v_lf + v_rf);
    m_v_sot.tail(m_robot_util->m_nbJoints) = dq;

    s = m_v_sot;
  }
  getProfiler().stop(PROFILE_FREE_FLYER_VELOCITY_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */
void FreeFlyerLocator::displayRobotUtil() { m_robot_util->display(std::cout); }

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void FreeFlyerLocator::display(std::ostream& os) const {
  os << "FreeFlyerLocator " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph
