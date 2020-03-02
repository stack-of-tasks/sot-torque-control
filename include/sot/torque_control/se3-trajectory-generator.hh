/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 *
 */

#ifndef __sot_torque_control_se3_trajectory_generator_H__
#define __sot_torque_control_se3_trajectory_generator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(se3_position_controller_EXPORTS)
#define SOTSE3TRAJECTORYGENERATOR_EXPORT __declspec(dllexport)
#else
#define SOTSE3TRAJECTORYGENERATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SOTSE3TRAJECTORYGENERATOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <map>
#include "boost/assign.hpp"

#include <parametric-curves/spline.hpp>

/* HELPER */
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/trajectory-generators.hh>

namespace dynamicgraph {
namespace sot {
namespace torque_control {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSE3TRAJECTORYGENERATOR_EXPORT SE3TrajectoryGenerator : public ::dynamicgraph::Entity {
  typedef SE3TrajectoryGenerator EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  /* --- CONSTRUCTOR ---- */
  SE3TrajectoryGenerator(const std::string& name);

  void init(const double& dt);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(initial_value, dynamicgraph::Vector);
  DECLARE_SIGNAL(x, OUT, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(trigger, bool);
  DECLARE_SIGNAL_OUT(dx, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(ddx, dynamicgraph::Vector);

 protected:
  DECLARE_SIGNAL_OUT_FUNCTION(x, dynamicgraph::Vector);

 public:
  /* --- COMMANDS --- */

  void playTrajectoryFile(const std::string& fileName);
  void startSpline();
  void setSpline(const std::string& filename, const double& timeToInitConf, const Eigen::MatrixXd& init_rotation);

  /** Print the current value of the specified component. */
  void getValue(const int& id);

  /** Move a component to a position with a minimum-jerk trajectory.
   * @param id integer index.
   * @param xFinal The desired final position of the component.
   * @param time The time to go from the current position to xFinal [sec].
   */
  void move(const int& id, const double& xFinal, const double& time);

  /** Start an infinite sinusoidal trajectory.
   * @param id integer index.
   * @param xFinal The position of the component corresponding to the max amplitude of the sinusoid.
   * @param time The time to go from the current position to xFinal [sec].
   */
  void startSinusoid(const int& id, const double& xFinal, const double& time);

  /** Start an infinite triangle trajectory.
   * @param id integer index.
   * @param xFinal The position of the component corresponding to the max amplitude of the trajectory.
   * @param time The time to go from the current position to xFinal [sec].
   */
  void startTriangle(const int& id, const double& xFinal, const double& time, const double& Tacc);

  /** Start an infinite trajectory with piece-wise constant acceleration.
   * @param id integer index.
   * @param xFinal The position of the component corresponding to the max amplitude of the trajectory.
   * @param time The time to go from the current position to xFinal [sec].
   * @param Tacc The time during witch acceleration is keept constant [sec].
   */
  void startConstAcc(const int& id, const double& xFinal, const double& time);

  /** Start a linear-chirp trajectory, that is a sinusoidal trajectory with frequency
   * being a linear function of time.
   * @param id integer index.
   * @param xFinal The position of the component corresponding to the max amplitude of the sinusoid [rad].
   * @param f0 The initial (min) frequency of the sinusoid [Hz]
   * @param f1 The final (max) frequency of the sinusoid [Hz]
   * @param time The time to get from f0 to f1 [sec]
   */
  void startLinearChirp(const int& id, const double& xFinal, const double& f0, const double& f1, const double& time);

  /** Stop the motion of the specified component. If id is -1
   * it stops the trajectory of all the vector.
   * @param id integer index.
   * */
  void stop(const int& id);

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

  void sendMsg(const std::string& msg, MsgType t = MSG_TYPE_INFO, const char* = "", int = 0) {
    logger_.stream(t) << ("[SE3TrajectoryGenerator-" + name + "] " + msg) << '\n';
  }

 protected:
  enum TG_Status {
    TG_STOP,
    TG_SINUSOID,
    TG_MIN_JERK,
    TG_LIN_CHIRP,
    TG_TRIANGLE,
    TG_CONST_ACC,
    TG_TEXT_FILE,
    TG_SPLINE
  };

  bool m_initSucceeded;     /// true if the entity has been successfully initialized
  bool m_firstIter;         /// true if it is the first iteration, false otherwise
  double m_dt;              /// control loop time period
  unsigned int m_np;        /// size of position vector
  unsigned int m_nv;        /// size of velocity vector
  unsigned int m_iterLast;  /// last iter index

  double m_t;
  Eigen::Matrix3d m_splineRotation;
  parametriccurves::Spline<double, Eigen::Dynamic>* m_splineTrajGen;
  bool m_splineReady;

  std::vector<TG_Status> m_status;  /// status of the component
  std::vector<AbstractTrajectoryGenerator*> m_currentTrajGen;
  std::vector<NoTrajectoryGenerator*> m_noTrajGen;
  std::vector<MinimumJerkTrajectoryGenerator*> m_minJerkTrajGen;
  std::vector<SinusoidTrajectoryGenerator*> m_sinTrajGen;
  std::vector<LinearChirpTrajectoryGenerator*> m_linChirpTrajGen;
  std::vector<TriangleTrajectoryGenerator*> m_triangleTrajGen;
  std::vector<ConstantAccelerationTrajectoryGenerator*> m_constAccTrajGen;
  TextFileTrajectoryGenerator* m_textFileTrajGen;

};  // class SE3TrajectoryGenerator

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_nd_trajectory_generator_H__
