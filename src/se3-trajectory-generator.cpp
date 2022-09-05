/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 *
 */

#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>
#include <sot/core/stop-watch.hh>
#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/se3-trajectory-generator.hh>

namespace dynamicgraph {
namespace sot {
namespace torque_control {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;
using namespace std;
using namespace Eigen;

#define PROFILE_SE3_POSITION_DESIRED_COMPUTATION "SE3TrajGen: traj computation"

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef SE3TrajectoryGenerator EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SE3TrajectoryGenerator,
                                   "SE3TrajectoryGenerator");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
SE3TrajectoryGenerator::SE3TrajectoryGenerator(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(initial_value, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL(x, OUT, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(trigger, bool),
      CONSTRUCT_SIGNAL_OUT(dx, dynamicgraph::Vector, m_xSOUT),
      CONSTRUCT_SIGNAL_OUT(ddx, dynamicgraph::Vector, m_xSOUT),
      m_initSucceeded(false),
      m_firstIter(true),
      m_np(12),
      m_nv(6),
      m_iterLast(0),
      m_t(0),
      m_splineReady(false) {
  BIND_SIGNAL_TO_FUNCTION(x, OUT, dynamicgraph::Vector);

  Entity::signalRegistration(m_xSOUT << m_dxSOUT << m_ddxSOUT
                                     << m_initial_valueSIN << m_triggerSIN);

  /* Commands. */
  addCommand("init", makeCommandVoid1(
                         *this, &SE3TrajectoryGenerator::init,
                         docCommandVoid1("Initialize the entity.",
                                         "Time period in seconds (double)")));

  addCommand(
      "getValue",
      makeCommandVoid1(
          *this, &SE3TrajectoryGenerator::getValue,
          docCommandVoid1("Get the current value of the specified index.",
                          "index (int)")));

  addCommand("playTrajectoryFile",
             makeCommandVoid1(
                 *this, &SE3TrajectoryGenerator::playTrajectoryFile,
                 docCommandVoid1(
                     "Play the trajectory read from the specified text file.",
                     "(string) File name, path included")));
  addCommand(
      "setSpline",
      makeCommandVoid3(*this, &SE3TrajectoryGenerator::setSpline,
                       docCommandVoid3("Load serialized spline from file",
                                       "(string)   filename",
                                       "(double) time to initial conf",
                                       "(Matrix) orientation of the point")));

  addCommand(
      "startSinusoid",
      makeCommandVoid3(
          *this, &SE3TrajectoryGenerator::startSinusoid,
          docCommandVoid3("Start an infinite sinusoid motion.",
                          "(int)    index", "(double) final value",
                          "(double) time to reach the final value in sec")));

  addCommand(
      "startTriangle",
      makeCommandVoid4(
          *this, &SE3TrajectoryGenerator::startTriangle,
          docCommandVoid4("Start an infinite triangle wave.", "(int)    index",
                          "(double) final values",
                          "(double) time to reach the final value in sec",
                          "(double) time to accelerate in sec")));

  addCommand(
      "startConstAcc",
      makeCommandVoid3(
          *this, &SE3TrajectoryGenerator::startConstAcc,
          docCommandVoid3("Start an infinite trajectory with piece-wise "
                          "constant acceleration.",
                          "(int)    index", "(double) final values",
                          "(double) time to reach the final value in sec")));

  addCommand("startLinChirp",
             makeCommandVoid5(
                 *this, &SE3TrajectoryGenerator::startLinearChirp,
                 docCommandVoid5("Start a linear-chirp motion.",
                                 "(int)    index", "(double) final values",
                                 "(double) initial frequency [Hz]",
                                 "(double) final frequency [Hz]",
                                 "(double) trajectory time [sec]")));
  addCommand("move", makeCommandVoid3(
                         *this, &SE3TrajectoryGenerator::move,
                         docCommandVoid3(
                             "Move component corresponding to index to the "
                             "specified value with a minimum jerk trajectory.",
                             "(int)    index", "(double) final values",
                             "(double) time to reach the final value in sec")));
  addCommand(
      "stop",
      makeCommandVoid1(
          *this, &SE3TrajectoryGenerator::stop,
          docCommandVoid1("Stop the motion of the specified index, or of all "
                          "components of the vector if index is equal to -1.",
                          "(int) index")));
}

void SE3TrajectoryGenerator::init(const double& dt) {
  if (dt <= 0.0) return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
  m_firstIter = true;
  m_dt = dt;
  m_status.resize(m_np, TG_STOP);
  m_minJerkTrajGen.resize(m_np);
  m_sinTrajGen.resize(m_np);
  m_triangleTrajGen.resize(m_np);
  m_constAccTrajGen.resize(m_np);
  m_linChirpTrajGen.resize(m_np);
  m_currentTrajGen.resize(m_np);
  m_noTrajGen.resize(m_np);
  for (unsigned int i = 0; i < m_np; i++) {
    m_minJerkTrajGen[i] = new MinimumJerkTrajectoryGenerator(dt, 5.0, 1);
    m_sinTrajGen[i] = new SinusoidTrajectoryGenerator(dt, 5.0, 1);
    m_triangleTrajGen[i] = new TriangleTrajectoryGenerator(dt, 5.0, 1);
    m_constAccTrajGen[i] =
        new ConstantAccelerationTrajectoryGenerator(dt, 5.0, 1);
    m_linChirpTrajGen[i] = new LinearChirpTrajectoryGenerator(dt, 5.0, 1);
    m_noTrajGen[i] = new NoTrajectoryGenerator(1);
    m_currentTrajGen[i] = m_noTrajGen[i];
  }
  m_splineTrajGen = new parametriccurves::Spline<double, Eigen::Dynamic>();
  m_textFileTrajGen = new TextFileTrajectoryGenerator(dt, m_np);
  m_initSucceeded = true;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(x, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot compute signal positionDes before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_SE3_POSITION_DESIRED_COMPUTATION);
  {
    if (s.size() != m_np) s.resize(m_np);

    // at first iteration store initial values
    if (m_firstIter) {
      const dynamicgraph::Vector& initial_value = m_initial_valueSIN(iter);
      if (initial_value.size() != m_np) {
        SEND_ERROR_STREAM_MSG(
            "Unexpected size of input signal initial_value: " +
            toString(initial_value.size()));
        getProfiler().stop(PROFILE_SE3_POSITION_DESIRED_COMPUTATION);
        return s;
      }
      for (unsigned int i = 0; i < m_np; i++)
        m_currentTrajGen[i]->set_initial_point(initial_value(i));
      m_firstIter = false;
    } else if (iter == static_cast<int>(m_iterLast)) {
      if (m_triggerSIN(iter) == true && m_splineReady) startSpline();
      if (m_status[0] == TG_TEXT_FILE) {
        for (unsigned int i = 0; i < m_np; i++)
          s(i) = m_textFileTrajGen->getPos()[i];
      } else if (m_status[0] == TG_SPLINE) {
        const Eigen::Vector3d& t = (*m_splineTrajGen)(m_t);
        s.head<3>() = t;
        s.segment<3>(3) = m_splineRotation.row(0);
        s.segment<3>(6) = m_splineRotation.row(1);
        s.segment<3>(9) = m_splineRotation.row(2);
      } else
        for (unsigned int i = 0; i < m_np; i++)
          s(i) = m_currentTrajGen[i]->getPos()(0);
      getProfiler().stop(PROFILE_SE3_POSITION_DESIRED_COMPUTATION);
      return s;
    }
    m_iterLast = iter;
    if (m_triggerSIN(iter) == true && m_splineReady) startSpline();
    if (m_status[0] == TG_TEXT_FILE) {
      const VectorXd& xRef = m_textFileTrajGen->compute_next_point();
      for (unsigned int i = 0; i < m_np; i++) {
        s(i) = xRef[i];
        if (m_textFileTrajGen->isTrajectoryEnded()) {
          m_noTrajGen[i]->set_initial_point(s(i));
          m_status[i] = TG_STOP;
        }
      }
      if (m_textFileTrajGen->isTrajectoryEnded())
        SEND_MSG("Text file trajectory ended.", MSG_TYPE_INFO);
    } else if (m_status[0] == TG_SPLINE) {
      m_t += m_dt;
      if (!m_splineTrajGen->checkRange(m_t)) {
        const Eigen::Vector3d& t = (*m_splineTrajGen)(m_splineTrajGen->tmax());
        s.head<3>() = t;
        s.segment<3>(3) = m_splineRotation.row(0);
        s.segment<3>(6) = m_splineRotation.row(1);
        s.segment<3>(9) = m_splineRotation.row(2);
        for (unsigned int i = 0; i < m_np; i++) {
          m_noTrajGen[i]->set_initial_point(s(i));
          m_status[i] = TG_STOP;
        }
        m_splineReady = false;
        SEND_MSG("Spline trajectory ended. Remember to turn off the trigger.",
                 MSG_TYPE_INFO);
        m_t = 0;
      } else {
        const Eigen::Vector3d& t = (*m_splineTrajGen)(m_t);
        s.head<3>() = t;
        s.segment<3>(3) = m_splineRotation.row(0);
        s.segment<3>(6) = m_splineRotation.row(1);
        s.segment<3>(9) = m_splineRotation.row(2);
      }
    } else {
      for (unsigned int i = 0; i < m_np; i++) {
        s(i) = m_currentTrajGen[i]->compute_next_point()(0);
        if (m_currentTrajGen[i]->isTrajectoryEnded()) {
          m_currentTrajGen[i] = m_noTrajGen[i];
          m_noTrajGen[i]->set_initial_point(s(i));
          m_status[i] = TG_STOP;
          SEND_MSG("Trajectory of index " + toString(i) + " ended.",
                   MSG_TYPE_INFO);
        }
      }
    }
  }
  getProfiler().stop(PROFILE_SE3_POSITION_DESIRED_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dx, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss(
        "Cannot compute signal positionDes before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }

  if (s.size() != m_nv) s.resize(m_nv);
  if (m_status[0] == TG_TEXT_FILE) {
    for (unsigned int i = 0; i < m_nv; i++)
      s(i) = m_textFileTrajGen->getVel()[i];
  } else if (m_status[0] == TG_SPLINE) {
    const Eigen::Vector3d& t = m_splineTrajGen->derivate(m_t, 1);
    s.segment<3>(0) = t;
    s.segment<3>(3).setZero();
  } else
    for (unsigned int i = 0; i < m_nv; i++)
      s(i) = m_currentTrajGen[i]->getVel()(0);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(ddx, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss(
        "Cannot compute signal positionDes before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }

  if (s.size() != m_nv) s.resize(m_nv);

  if (m_status[0] == TG_TEXT_FILE) {
    for (unsigned int i = 0; i < m_nv; i++)
      s(i) = m_textFileTrajGen->getAcc()[i];
  } else if (m_status[0] == TG_SPLINE) {
    const Eigen::Vector3d& t = m_splineTrajGen->derivate(m_t, 2);
    s.segment<3>(0) = t;
    s.segment<3>(3).setZero();
  } else
    for (unsigned int i = 0; i < m_nv; i++)
      s(i) = m_currentTrajGen[i]->getAcc()(0);

  return s;
}

/* ------------------------------------------------------------------- */
/* --- COMMANDS ------------------------------------------------------ */
/* ------------------------------------------------------------------- */

void SE3TrajectoryGenerator::getValue(const int& id) {
  if (id < 0 || id >= static_cast<int>(m_np))
    return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);

  SEND_MSG("Current value of component " + toString(id) + " is " +
               toString(m_currentTrajGen[id]->getPos()(0)),
           MSG_TYPE_INFO);
}

void SE3TrajectoryGenerator::playTrajectoryFile(const std::string& fileName) {
  if (!m_initSucceeded)
    return SEND_MSG("Cannot start sinusoid before initialization!",
                    MSG_TYPE_ERROR);

  for (unsigned int i = 0; i < m_np; i++)
    if (m_status[i] != TG_STOP)
      return SEND_MSG("You cannot control component " + toString(i) +
                          " because it is already controlled.",
                      MSG_TYPE_ERROR);

  if (!m_textFileTrajGen->loadTextFile(fileName))
    return SEND_MSG("Error while loading text file " + fileName,
                    MSG_TYPE_ERROR);

  // check current configuration is not too far from initial trajectory
  // configuration
  bool needToMoveToInitConf = false;
  const VectorXd& xInit = m_textFileTrajGen->get_initial_point();
  for (unsigned int i = 0; i < m_np; i++)
    if (fabs(xInit[i] - m_currentTrajGen[i]->getPos()(0)) > 0.001) {
      needToMoveToInitConf = true;
      SEND_MSG("Component " + toString(i) +
                   " is too far from initial configuration so first i will "
                   "move it there.",
               MSG_TYPE_WARNING);
    }

  // if necessary move joints to initial configuration
  if (needToMoveToInitConf) {
    for (unsigned int i = 0; i < m_np; i++) {
      m_minJerkTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
      m_minJerkTrajGen[i]->set_final_point(xInit[i]);
      m_minJerkTrajGen[i]->set_trajectory_time(4.0);
      m_status[i] = TG_MIN_JERK;
      m_currentTrajGen[i] = m_minJerkTrajGen[i];
    }
    return;
  }

  for (unsigned int i = 0; i < m_np; i++) {
    m_status[i] = TG_TEXT_FILE;
  }
}

void SE3TrajectoryGenerator::setSpline(const std::string& fileName,
                                       const double& timeToInitConf,
                                       const Eigen::MatrixXd& init_rotation) {
  if (!m_initSucceeded)
    return SEND_MSG("Cannot start sinusoid before initialization!",
                    MSG_TYPE_ERROR);

  for (unsigned int i = 0; i < m_np; i++)
    if (m_status[i] != TG_STOP)
      return SEND_MSG("You cannot control component " + toString(i) +
                          " because it is already controlled.",
                      MSG_TYPE_ERROR);

  if (!m_splineTrajGen->loadFromFile(fileName))
    return SEND_MSG("Error while loading text file " + fileName,
                    MSG_TYPE_ERROR);

  // check current configuration is not too far from initial trajectory
  // configuration
  bool needToMoveToInitConf = false;
  m_splineReady = true;
  m_splineRotation = init_rotation;
  if (timeToInitConf < 0) {
    SEND_MSG("Spline Ready. Set trigger to true to start playing",
             MSG_TYPE_INFO);
    return;
  }

  const VectorXd& t = (*m_splineTrajGen)(0.0);
  VectorXd xInit(12);
  xInit.segment<3>(3) = m_splineRotation.row(0);
  xInit.segment<3>(6) = m_splineRotation.row(1);
  xInit.segment<3>(9) = m_splineRotation.row(2);
  xInit.head<3>() = t;

  for (unsigned int i = 0; i < m_np; i++)
    if (fabs(xInit[i] - m_currentTrajGen[i]->getPos()(0)) > 0.001) {
      needToMoveToInitConf = true;
      SEND_MSG("Component " + toString(i) +
                   " is too far from initial configuration so first i will "
                   "move it there.",
               MSG_TYPE_WARNING);
    }

  // if necessary move joints to initial configuration
  if (needToMoveToInitConf) {
    for (unsigned int i = 0; i < m_np; i++) {
      m_minJerkTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
      m_minJerkTrajGen[i]->set_final_point(xInit[i]);
      m_minJerkTrajGen[i]->set_trajectory_time(timeToInitConf);
      m_status[i] = TG_MIN_JERK;
      m_currentTrajGen[i] = m_minJerkTrajGen[i];
    }
    return;
  }

  SEND_MSG("Spline Ready. Set trigger to true to start playing", MSG_TYPE_INFO);
}

void SE3TrajectoryGenerator::startSpline() {
  if (m_status[0] == TG_SPLINE) return;
  m_t = 0.0;
  for (unsigned int i = 0; i < m_np; i++) {
    m_status[i] = TG_SPLINE;
  }
}

void SE3TrajectoryGenerator::startSinusoid(const int& id, const double& xFinal,
                                           const double& time) {
  if (!m_initSucceeded)
    return SEND_MSG("Cannot start sinusoid before initialization!",
                    MSG_TYPE_ERROR);

  if (id < 0 || id >= static_cast<int>(m_np))
    return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
  unsigned int i = id;
  if (time <= 0.0)
    return SEND_MSG("Trajectory time must be a positive number",
                    MSG_TYPE_ERROR);
  if (m_status[i] != TG_STOP)
    return SEND_MSG(
        "You cannot move the specified component because it is already "
        "controlled.",
        MSG_TYPE_ERROR);

  m_sinTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
  SEND_MSG(
      "Set initial point of sinusoid to " + toString(m_sinTrajGen[i]->getPos()),
      MSG_TYPE_DEBUG);
  m_sinTrajGen[i]->set_final_point(xFinal);
  m_sinTrajGen[i]->set_trajectory_time(time);
  m_status[i] = TG_SINUSOID;
  m_currentTrajGen[i] = m_sinTrajGen[i];
}

void SE3TrajectoryGenerator::startTriangle(const int& id, const double& xFinal,
                                           const double& time,
                                           const double& Tacc) {
  if (!m_initSucceeded)
    return SEND_MSG("Cannot start triangle before initialization!",
                    MSG_TYPE_ERROR);
  if (id < 0 || id >= static_cast<int>(m_np))
    return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
  unsigned int i = id;
  if (m_status[i] != TG_STOP)
    return SEND_MSG(
        "You cannot move the specified component because it is already "
        "controlled.",
        MSG_TYPE_ERROR);

  m_triangleTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
  SEND_MSG("Set initial point of triangular trajectory to " +
               toString(m_triangleTrajGen[i]->getPos()),
           MSG_TYPE_DEBUG);
  m_triangleTrajGen[i]->set_final_point(xFinal);

  if (!m_triangleTrajGen[i]->set_trajectory_time(time))
    return SEND_MSG("Trajectory time cannot be negative.", MSG_TYPE_ERROR);

  if (!m_triangleTrajGen[i]->set_acceleration_time(Tacc))
    return SEND_MSG(
        "Acceleration time cannot be negative or larger than half the "
        "trajectory time.",
        MSG_TYPE_ERROR);

  m_status[i] = TG_TRIANGLE;
  m_currentTrajGen[i] = m_triangleTrajGen[i];
}

void SE3TrajectoryGenerator::startConstAcc(const int& id, const double& xFinal,
                                           const double& time) {
  if (!m_initSucceeded)
    return SEND_MSG(
        "Cannot start constant-acceleration trajectory before initialization!",
        MSG_TYPE_ERROR);
  if (id < 0 || id >= static_cast<int>(m_np))
    return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
  unsigned int i = id;
  if (time <= 0.0)
    return SEND_MSG("Trajectory time must be a positive number",
                    MSG_TYPE_ERROR);
  if (m_status[i] != TG_STOP)
    return SEND_MSG(
        "You cannot move the specified component because it is already "
        "controlled.",
        MSG_TYPE_ERROR);

  m_constAccTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
  SEND_MSG("Set initial point of const-acc trajectory to " +
               toString(m_constAccTrajGen[i]->getPos()),
           MSG_TYPE_DEBUG);
  m_constAccTrajGen[i]->set_final_point(xFinal);
  m_constAccTrajGen[i]->set_trajectory_time(time);
  m_status[i] = TG_CONST_ACC;
  m_currentTrajGen[i] = m_constAccTrajGen[i];
}

void SE3TrajectoryGenerator::startLinearChirp(const int& id,
                                              const double& xFinal,
                                              const double& f0,
                                              const double& f1,
                                              const double& time) {
  if (!m_initSucceeded)
    return SEND_MSG("Cannot start linear chirp before initialization!",
                    MSG_TYPE_ERROR);
  if (id < 0 || id >= static_cast<int>(m_np))
    return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
  unsigned int i = id;
  if (time <= 0.0)
    return SEND_MSG("Trajectory time must be a positive number",
                    MSG_TYPE_ERROR);
  if (m_status[i] != TG_STOP)
    return SEND_MSG(
        "You cannot move the specified component because it is already "
        "controlled.",
        MSG_TYPE_ERROR);
  if (f0 > f1)
    return SEND_MSG(
        "f0 " + toString(f0) + " cannot to be more than f1 " + toString(f1),
        MSG_TYPE_ERROR);
  if (f0 <= 0.0)
    return SEND_MSG("Frequency has to be positive " + toString(f0),
                    MSG_TYPE_ERROR);

  if (!m_linChirpTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos()))
    return SEND_MSG("Error while setting initial point " +
                        toString(m_noTrajGen[i]->getPos()),
                    MSG_TYPE_ERROR);
  if (!m_linChirpTrajGen[i]->set_final_point(xFinal))
    return SEND_MSG("Error while setting final point " + toString(xFinal),
                    MSG_TYPE_ERROR);
  if (!m_linChirpTrajGen[i]->set_trajectory_time(time))
    return SEND_MSG("Error while setting trajectory time " + toString(time),
                    MSG_TYPE_ERROR);
  if (!m_linChirpTrajGen[i]->set_initial_frequency(f0))
    return SEND_MSG("Error while setting initial frequency " + toString(f0),
                    MSG_TYPE_ERROR);
  if (!m_linChirpTrajGen[i]->set_final_frequency(f1))
    return SEND_MSG("Error while setting final frequency " + toString(f1),
                    MSG_TYPE_ERROR);
  m_status[i] = TG_LIN_CHIRP;
  m_currentTrajGen[i] = m_linChirpTrajGen[i];
}

void SE3TrajectoryGenerator::move(const int& id, const double& xFinal,
                                  const double& time) {
  if (!m_initSucceeded)
    return SEND_MSG("Cannot move value before initialization!", MSG_TYPE_ERROR);
  unsigned int i = id;
  if (id < 0 || id >= static_cast<int>(m_np))
    return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
  if (time <= 0.0)
    return SEND_MSG("Trajectory time must be a positive number",
                    MSG_TYPE_ERROR);
  if (m_status[i] != TG_STOP)
    return SEND_MSG(
        "You cannot move the specified component because it is already "
        "controlled.",
        MSG_TYPE_ERROR);

  m_minJerkTrajGen[i]->set_initial_point(m_noTrajGen[i]->getPos());
  m_minJerkTrajGen[i]->set_final_point(xFinal);
  m_minJerkTrajGen[i]->set_trajectory_time(time);
  m_status[i] = TG_MIN_JERK;
  m_currentTrajGen[i] = m_minJerkTrajGen[i];
}

void SE3TrajectoryGenerator::stop(const int& id) {
  if (!m_initSucceeded)
    return SEND_MSG("Cannot stop value before initialization!", MSG_TYPE_ERROR);

  if (id == -1)  // Stop entire vector
  {
    for (unsigned int i = 0; i < m_np; i++) {
      m_status[i] = TG_STOP;
      // update the initial value
      m_noTrajGen[i]->set_initial_point(m_currentTrajGen[i]->getPos());
      m_currentTrajGen[i] = m_noTrajGen[i];
    }
    return;
  }
  if (id < 0 || id >= static_cast<int>(m_np))
    return SEND_MSG("Index is out of bounds", MSG_TYPE_ERROR);
  unsigned int i = id;
  m_noTrajGen[i]->set_initial_point(m_currentTrajGen[i]->getPos());
  m_status[i] = TG_STOP;
  m_currentTrajGen[i] = m_noTrajGen[i];

  m_splineReady = false;
  m_t = 0.0;
}

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void SE3TrajectoryGenerator::display(std::ostream& os) const {
  os << "SE3TrajectoryGenerator " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph
