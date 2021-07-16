/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 *
 */

#define EIGEN_RUNTIME_NO_MALLOC

#include <sot/torque_control/inverse-dynamics-balance-controller.hh>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-rt.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/math/utils.hpp>

#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>

#include <sot/torque_control/commands-helper.hh>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/debug-sot-torqe-control.dat"

#define RESETDEBUG5()                            \
  {                                              \
    std::ofstream DebugFile;                     \
    DebugFile.open(DBGFILE, std::ofstream::out); \
    DebugFile.close();                           \
  }
#define ODEBUG5FULL(x)                                                                          \
  {                                                                                             \
    std::ofstream DebugFile;                                                                    \
    DebugFile.open(DBGFILE, std::ofstream::app);                                                \
    DebugFile << __FILE__ << ":" << __FUNCTION__ << "(#" << __LINE__ << "):" << x << std::endl; \
    DebugFile.close();                                                                          \
  }
#define ODEBUG5(x)                               \
  {                                              \
    std::ofstream DebugFile;                     \
    DebugFile.open(DBGFILE, std::ofstream::app); \
    DebugFile << x << std::endl;                 \
    DebugFile.close();                           \
  }

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

typedef SolverHQuadProgRT<60, 36, 34> SolverHQuadProgRT60x36x34;
typedef SolverHQuadProgRT<48, 30, 17> SolverHQuadProgRT48x30x17;

#define REQUIRE_FINITE(A) assert(is_finite(A))

// Size to be aligned                "-------------------------------------------------------"
#define PROFILE_TAU_DES_COMPUTATION "InvDynBalCtrl: desired tau"
#define PROFILE_HQP_SOLUTION "InvDynBalCtrl: HQP"
#define PROFILE_PREPARE_INV_DYN "InvDynBalCtrl: prepare inv-dyn"
#define PROFILE_READ_INPUT_SIGNALS "InvDynBalCtrl: read input signals"

#define ZERO_FORCE_THRESHOLD 10

#define INPUT_SIGNALS m_com_ref_posSIN \
  << m_com_ref_velSIN \
  << m_com_ref_accSIN \
  << m_com_adm_ref_posSIN \
  << m_com_adm_ref_velSIN \
  << m_com_adm_ref_accSIN \
  << m_am_ref_LSIN \
  << m_am_ref_dLSIN \
  << m_rf_ref_posSIN \
  << m_rf_ref_velSIN \
  << m_rf_ref_accSIN \
  << m_lf_ref_posSIN \
  << m_lf_ref_velSIN \
  << m_lf_ref_accSIN \
  << m_rh_ref_posSIN \
  << m_rh_ref_velSIN \
  << m_rh_ref_accSIN \
  << m_lh_ref_posSIN \
  << m_lh_ref_velSIN \
  << m_lh_ref_accSIN \
  << m_posture_ref_posSIN \
  << m_posture_ref_velSIN \
  << m_posture_ref_accSIN \
  << m_base_orientation_ref_posSIN \
  << m_base_orientation_ref_velSIN \
  << m_base_orientation_ref_accSIN \
  << m_f_ref_right_footSIN \
  << m_f_ref_left_footSIN \
  << m_kp_base_orientationSIN \
  << m_kd_base_orientationSIN \
  << m_kp_constraintsSIN \
  << m_kd_constraintsSIN \
  << m_kp_comSIN \
  << m_kd_comSIN \
  << m_kp_amSIN \
  << m_kd_amSIN \
  << m_kp_feetSIN \
  << m_kd_feetSIN \
  << m_kp_handsSIN \
  << m_kd_handsSIN \
  << m_kp_postureSIN \
  << m_kd_postureSIN \
  << m_kp_posSIN \
  << m_kd_posSIN \
  << m_kp_tauSIN \
  << m_kff_tauSIN \
  << m_kff_dqSIN \
  << m_w_comSIN \
  << m_w_amSIN \
  << m_w_feetSIN \
  << m_w_handsSIN \
  << m_w_postureSIN \
  << m_w_base_orientationSIN \
  << m_w_torquesSIN \
  << m_w_forcesSIN \
  << m_weight_contact_forcesSIN \
  << m_muSIN \
  << m_contact_pointsSIN \
  << m_contact_normalSIN \
  << m_f_minSIN \
  << m_f_max_right_footSIN \
  << m_f_max_left_footSIN \
  << m_rotor_inertiasSIN \
  << m_gear_ratiosSIN \
  << m_tau_maxSIN \
  << m_q_minSIN \
  << m_q_maxSIN \
  << m_dq_maxSIN \
  << m_ddq_maxSIN \
  << m_dt_joint_pos_limitsSIN \
  << m_tau_measuredSIN \
  << m_com_measuredSIN \
  << m_qSIN \
  << m_vSIN \
  << m_wrench_baseSIN \
  << m_wrench_left_footSIN  \
  << m_wrench_right_footSIN \
  << m_ref_phaseSIN \
  << m_active_jointsSIN \
  << m_ref_pos_finalSIN

#define OUTPUT_SIGNALS m_tau_desSOUT \
  << m_MSOUT \
  << m_dv_desSOUT \
  << m_v_desSOUT \
  << m_q_desSOUT \
  << m_tau_pd_desSOUT \
  << m_f_des_right_footSOUT \
  << m_f_des_left_footSOUT \
  << m_zmp_des_right_footSOUT \
  << m_zmp_des_left_footSOUT \
  << m_zmp_des_right_foot_localSOUT \
  << m_zmp_des_left_foot_localSOUT \
  << m_zmp_desSOUT \
  << m_zmp_refSOUT \
  << m_zmp_right_footSOUT \
  << m_zmp_left_footSOUT \
  << m_zmpSOUT \
  << m_comSOUT \
  << m_com_estSOUT \
  << m_com_velSOUT \
  << m_com_accSOUT \
  << m_com_acc_desSOUT \
  << m_dcmSOUT \
  << m_am_LSOUT \
  << m_am_dLSOUT \
  << m_am_dL_desSOUT \
  << m_base_orientationSOUT \
  << m_left_foot_posSOUT \
  << m_left_foot_pos_quatSOUT \
  << m_left_foot_pos_ref_quatSOUT \
  << m_right_foot_posSOUT \
  << m_right_foot_pos_quatSOUT \
  << m_right_foot_pos_ref_quatSOUT \
  << m_lf_estSOUT \
  << m_rf_estSOUT \
  << m_left_hand_posSOUT \
  << m_right_hand_posSOUT \
  << m_left_foot_velSOUT \
  << m_right_foot_velSOUT \
  << m_left_hand_velSOUT \
  << m_right_hand_velSOUT \
  << m_left_foot_accSOUT \
  << m_right_foot_accSOUT \
  << m_left_hand_accSOUT \
  << m_right_hand_accSOUT \
  << m_right_foot_acc_desSOUT \
  << m_left_foot_acc_desSOUT \
  << m_energySOUT \
  << m_energy_derivativeSOUT \
  << m_energy_tankSOUT \
  << m_energy_boundSOUT \
  << m_task_energy_constSOUT \
  << m_task_energy_boundSOUT \
  << m_task_energy_alphaSOUT \
  << m_task_energy_betaSOUT \
  << m_task_energy_gammaSOUT \
  << m_task_energy_SSOUT \
  << m_task_energy_ASOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef InverseDynamicsBalanceController EntityClassName;

typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorN;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorN6;
/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(InverseDynamicsBalanceController, "InverseDynamicsBalanceController");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
InverseDynamicsBalanceController::InverseDynamicsBalanceController(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(com_ref_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(com_ref_vel, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(com_ref_acc, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(com_adm_ref_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(com_adm_ref_vel, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(com_adm_ref_acc, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(am_ref_L, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(am_ref_dL, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rf_ref_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rf_ref_vel, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rf_ref_acc, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(lf_ref_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(lf_ref_vel, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(lf_ref_acc, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rh_ref_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rh_ref_vel, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(rh_ref_acc, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(lh_ref_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(lh_ref_vel, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(lh_ref_acc, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(posture_ref_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(posture_ref_vel, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(posture_ref_acc, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(base_orientation_ref_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(base_orientation_ref_vel, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(base_orientation_ref_acc, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(f_ref_right_foot, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(f_ref_left_foot, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_base_orientation, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_base_orientation, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_constraints, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_constraints, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_com, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_com, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_am, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_am, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_feet, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_feet, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_hands, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_hands, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_posture, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_posture, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kd_pos, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kp_tau, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kff_tau, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(kff_dq, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(w_com, double),
      CONSTRUCT_SIGNAL_IN(w_am, double),
      CONSTRUCT_SIGNAL_IN(w_feet, double),
      CONSTRUCT_SIGNAL_IN(w_hands, double),
      CONSTRUCT_SIGNAL_IN(w_posture, double),
      CONSTRUCT_SIGNAL_IN(w_base_orientation, double),
      CONSTRUCT_SIGNAL_IN(w_torques, double),
      CONSTRUCT_SIGNAL_IN(w_forces, double),
      CONSTRUCT_SIGNAL_IN(weight_contact_forces, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(mu, double),
      CONSTRUCT_SIGNAL_IN(contact_points, dynamicgraph::Matrix),
      CONSTRUCT_SIGNAL_IN(contact_normal, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(f_min, double),
      CONSTRUCT_SIGNAL_IN(f_max_right_foot, double),
      CONSTRUCT_SIGNAL_IN(f_max_left_foot, double),
      CONSTRUCT_SIGNAL_IN(rotor_inertias, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(gear_ratios, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tau_max, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(q_min, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(q_max, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dq_max, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(ddq_max, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dt_joint_pos_limits, double),
      CONSTRUCT_SIGNAL_IN(tau_measured, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(com_measured, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(q, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(v, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(wrench_base, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(wrench_left_foot, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(wrench_right_foot, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(ref_phase, int),
      CONSTRUCT_SIGNAL_IN(active_joints, dynamicgraph::Vector),      
      CONSTRUCT_SIGNAL_IN(ref_pos_final, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(tau_des, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(M, dg::Matrix, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(dv_des, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(v_des, dg::Vector, m_dv_desSOUT),
      CONSTRUCT_SIGNAL_OUT(q_des, dg::Vector, m_v_desSOUT),
      CONSTRUCT_SIGNAL_OUT(tau_pd_des, dg::Vector, INPUT_SIGNALS << m_q_desSOUT),
      CONSTRUCT_SIGNAL_OUT(f_des_right_foot, dynamicgraph::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(f_des_left_foot, dynamicgraph::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(zmp_des_right_foot, dynamicgraph::Vector, m_f_des_right_footSOUT),
      CONSTRUCT_SIGNAL_OUT(zmp_des_left_foot, dynamicgraph::Vector, m_f_des_left_footSOUT),
      CONSTRUCT_SIGNAL_OUT(zmp_des_right_foot_local, dynamicgraph::Vector, m_f_des_right_footSOUT),
      CONSTRUCT_SIGNAL_OUT(zmp_des_left_foot_local, dynamicgraph::Vector, m_f_des_left_footSOUT),
      CONSTRUCT_SIGNAL_OUT(zmp_des, dynamicgraph::Vector, m_zmp_des_left_footSOUT << m_zmp_des_right_footSOUT),
      CONSTRUCT_SIGNAL_OUT(zmp_ref, dynamicgraph::Vector, m_f_ref_left_footSIN << m_f_ref_right_footSIN),
      CONSTRUCT_SIGNAL_OUT(zmp_right_foot, dg::Vector, m_wrench_right_footSIN),
      CONSTRUCT_SIGNAL_OUT(zmp_left_foot, dg::Vector, m_wrench_left_footSIN),
      CONSTRUCT_SIGNAL_OUT(zmp, dg::Vector, m_wrench_left_footSIN << m_wrench_right_footSIN << m_zmp_left_footSOUT << m_zmp_right_footSOUT),
      CONSTRUCT_SIGNAL_OUT(com, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(com_est, dg::Vector, INPUT_SIGNALS << m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(com_vel, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(com_acc, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(com_acc_des, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(dcm, dg::Vector, m_tau_desSOUT << m_comSOUT << m_com_velSOUT),
      CONSTRUCT_SIGNAL_OUT(am_L, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(am_dL, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(am_dL_des, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(base_orientation, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_foot_pos, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_foot_pos_quat, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_foot_pos_ref_quat, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(left_foot_pos, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(left_foot_pos_quat, dg::Vector, m_tau_desSOUT),   
      CONSTRUCT_SIGNAL_OUT(left_foot_pos_ref_quat, dg::Vector, m_tau_desSOUT),      
      CONSTRUCT_SIGNAL_OUT(lf_est, dg::Vector, INPUT_SIGNALS << m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(rf_est, dg::Vector, INPUT_SIGNALS << m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_hand_pos, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(left_hand_pos, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_foot_vel, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(left_foot_vel, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_hand_vel, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(left_hand_vel, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_foot_acc, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(left_foot_acc, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_hand_acc, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(left_hand_acc, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(right_foot_acc_des, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(left_foot_acc_des, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(energy, double, INPUT_SIGNALS << m_q_desSOUT),
      CONSTRUCT_SIGNAL_OUT(energy_derivative, double, m_energySOUT),
      CONSTRUCT_SIGNAL_OUT(energy_tank, double, INPUT_SIGNALS << m_q_desSOUT),
      CONSTRUCT_SIGNAL_OUT(energy_bound, double, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(task_energy_const, double, m_tau_desSOUT), 
      CONSTRUCT_SIGNAL_OUT(task_energy_bound, double, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(task_energy_alpha, double, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(task_energy_beta, double, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(task_energy_gamma, double, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(task_energy_S, dg::Vector, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_OUT(task_energy_A, double, m_tau_desSOUT),
      CONSTRUCT_SIGNAL_INNER(active_joints_checked, dg::Vector, m_active_jointsSIN),
      m_t(0.0),
      m_initSucceeded(false),
      m_enabled(false),
      m_firstTime(true),
      m_contactState(DOUBLE_SUPPORT),
      m_rightHandState(TASK_RIGHT_HAND_OFF),
      m_leftHandState(TASK_LEFT_HAND_OFF),
      m_timeLast(0),
      m_robot_util(RefVoidRobotUtil()),
      m_ctrlMode(CONTROL_OUTPUT_TORQUE) {
  RESETDEBUG5();
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  m_zmp_des_RF.setZero();
  m_zmp_des_LF.setZero();
  m_zmp_des_RF_local.setZero();
  m_zmp_des_LF_local.setZero();
  m_zmp_des.setZero();
  m_zmp_RF.setZero();
  m_zmp_LF.setZero();
  m_zmp.setZero();
  m_com_offset.setZero();
  m_dcom_offset.setZero();
  m_v_RF_int.setZero();
  m_v_LF_int.setZero();

  /* Commands. */
  addCommand("init", makeCommandVoid2(*this, &InverseDynamicsBalanceController::init,
                                      docCommandVoid2("Initialize the entity.", "Time period in seconds (double)",
                                                      "Robot reference (string)")));

  addCommand("updateComOffset",
             makeCommandVoid0(*this, &InverseDynamicsBalanceController::updateComOffset,
                              docCommandVoid0("Update the offset on the CoM based on the CoP measurement.")));

  /* SET of control output type. */
  addCommand("setControlOutputType",
             makeCommandVoid1(*this, &InverseDynamicsBalanceController::setControlOutputType,
                              docCommandVoid1("Set the type of control output.",
                                              "Control type: velocity or torque (string)")));
  addCommand("removeRightFootContact",
             makeCommandVoid1(
                 *this, &InverseDynamicsBalanceController::removeRightFootContact,
                 docCommandVoid1("Remove the contact at the right foot.", "Transition time in seconds (double)")));

  addCommand("removeLeftFootContact", makeCommandVoid1(*this, &InverseDynamicsBalanceController::removeLeftFootContact,
                                                       docCommandVoid1("Remove the contact at the left foot.",
                                                                       "Transition time in seconds (double)")));
  addCommand("addRightFootContact",
             makeCommandVoid1(
                 *this, &InverseDynamicsBalanceController::addRightFootContact,
                 docCommandVoid1("Add the contact at the right foot.", "Transition time in seconds (double)")));

  addCommand("addLeftFootContact", makeCommandVoid1(*this, &InverseDynamicsBalanceController::addLeftFootContact,
                                                       docCommandVoid1("Add the contact at the left foot.",
                                                                       "Transition time in seconds (double)")));
  addCommand("addTaskRightHand", makeCommandVoid0(*this, &InverseDynamicsBalanceController::addTaskRightHand,
                                                  docCommandVoid0("Adds an SE3 task for the right hand.")));
  addCommand("removeTaskRightHand", makeCommandVoid1(*this, &InverseDynamicsBalanceController::removeTaskRightHand,
                                                     docCommandVoid1("Removes the SE3 task for the right hand.",
                                                                     "Transition time in seconds (double)")));
  addCommand("addTaskLeftHand", makeCommandVoid0(*this, &InverseDynamicsBalanceController::addTaskLeftHand,
                                                 docCommandVoid0("Raises the left hand.")));
  addCommand("removeTaskLeftHand",
             makeCommandVoid1(*this, &InverseDynamicsBalanceController::removeTaskLeftHand,
                              docCommandVoid1("lowers the left hand.", "Transition time in seconds (double)")));

  addCommand("addTaskEnergy", makeCommandVoid1(*this, &InverseDynamicsBalanceController::addTaskEnergy,
                                               docCommandVoid1("Add the energy task.", "Transition time in seconds (double)")));
  addCommand("removeTaskEnergy",
             makeCommandVoid1(*this, &InverseDynamicsBalanceController::removeTaskEnergy,
                              docCommandVoid1("Remove the energy task.", "Transition time in seconds (double)")));
  addCommand("setEnergyTank",
             makeCommandVoid1(*this, &InverseDynamicsBalanceController::setEnergyTank,
                              docCommandVoid1("Set the value of the energyTank", "Value of the tank in Joule (double)")));

}

Vector InverseDynamicsBalanceController::actFrame(pinocchio::SE3 frame, Vector vec) {
  Vector res;
  pinocchio::SE3 vectorSE3, resSE3;
  vectorSE3.translation(vec.head<3>());
  if (vec.size() == 12){ // Feet Positions -> directly have Rotation Matrix
    res.resize(12);
    res.head<3>() = frame.rotation() * (vec.head<3>() - frame.translation());
    MatrixRotation R;
    R.row(0) = vec.segment(3, 3);
    R.row(1) = vec.segment(6, 3);
    R.row(2) = vec.segment(9, 3);
    Vector3 euler = R.eulerAngles(2, 1, 0).reverse();
    R = (Eigen::AngleAxisd(-euler(2), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-euler(1), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX()))
         .toRotationMatrix();
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> M_ordered(R);
    Eigen::Map<dg::Vector> vectorRotation(M_ordered.data(), M_ordered.size());
    res.tail<9>() = vectorRotation;

  } else if (vec.size() == 3){ // COM
    res.resize(3);
    res = frame.rotation() * (vec - frame.translation());
  } else{
    SEND_MSG("ERROR on actFrame() wrong size of vector : " + toString(vec.size()), MSG_TYPE_ERROR);
  }
  return res;
}

void InverseDynamicsBalanceController::updateComOffset() {
  const Vector3& com = m_robot->com(m_invDyn->data());
  m_com_offset = m_zmp - com;
  m_com_offset(2) = 0.0;
  SEND_MSG("CoM offset updated: " + toString(m_com_offset), MSG_TYPE_INFO);
}

void InverseDynamicsBalanceController::setControlOutputType(const std::string& type) {
  for (int i = 0; i < CONTROL_OUTPUT_SIZE; i++)
    if (type == ControlOutput_s[i]) {
      m_ctrlMode = (ControlOutput)i;
      sotDEBUG(25) << "Control output type: " << ControlOutput_s[i] << endl;
      return;
    }
  sotDEBUG(25) << "Unrecognized control output type: " << type << endl;
}

void InverseDynamicsBalanceController::removeRightFootContact(const double& transitionTime) {
  if (m_contactState == DOUBLE_SUPPORT) {
    SEND_MSG("Remove right foot contact in " + toString(transitionTime) + " s", MSG_TYPE_ERROR);
    bool res = m_invDyn->removeRigidContact(m_contactRF->name(), transitionTime);
    if (!res) {
      const HQPData& hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
      SEND_MSG("Error while remove right foot contact." + tsid::solvers::HQPDataToString(hqpData, false),
               MSG_TYPE_ERROR);
    }
    const double& w_feet = m_w_feetSIN.accessCopy();
    m_invDyn->addMotionTask(*m_taskRF, w_feet, 1);
    if (transitionTime > m_dt) {
      m_contactState = LEFT_SUPPORT_TRANSITION;
      m_contactTransitionTime = m_t + transitionTime;
    } else {
      m_contactState = LEFT_SUPPORT;
    }
  }
}

void InverseDynamicsBalanceController::removeLeftFootContact(const double& transitionTime) {
  if (m_contactState == DOUBLE_SUPPORT) {
    SEND_MSG("Remove left foot contact in " + toString(transitionTime) + " s", MSG_TYPE_ERROR);
    bool res = m_invDyn->removeRigidContact(m_contactLF->name(), transitionTime);
    if (!res) {
      const HQPData& hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
      SEND_MSG("Error while remove right foot contact." + tsid::solvers::HQPDataToString(hqpData, false),
               MSG_TYPE_ERROR);
    }
    const double& w_feet = m_w_feetSIN.accessCopy();
    m_invDyn->addMotionTask(*m_taskLF, w_feet, 1);
    if (transitionTime > m_dt) {
      m_contactState = RIGHT_SUPPORT_TRANSITION;
      m_contactTransitionTime = m_t + transitionTime;
    } else {
      m_contactState = RIGHT_SUPPORT;
    }
  }
}

void InverseDynamicsBalanceController::addTaskRightHand(/*const double& transitionTime*/) {
  if (m_rightHandState == TASK_RIGHT_HAND_OFF) {
    SEND_MSG("Adds right hand SE3 task in " /*+toString(transitionTime)+" s"*/, MSG_TYPE_INFO);
    const double& w_hands = m_w_handsSIN.accessCopy();
    m_invDyn->addMotionTask(*m_taskRH, w_hands, 1);
  }
  /*if(transitionTime>m_dt)
  {
    m_rightHandState = TASK_RIGHT_HAND_TRANSITION;
    m_handsTransitionTime = m_t + transitionTime;
  }
  else
    m_rightHandState = TASK_RIGHT_HAND_ON;*/
  m_rightHandState = TASK_RIGHT_HAND_ON;
}

void InverseDynamicsBalanceController::addTaskLeftHand(/*const double& transitionTime*/) {
  if (m_leftHandState == TASK_LEFT_HAND_OFF) {
    SEND_MSG("Raise left hand in " /*+toString(transitionTime)+" s"*/, MSG_TYPE_INFO);
    const double& w_hands = m_w_handsSIN.accessCopy();
    m_invDyn->addMotionTask(*m_taskLH, w_hands, 1);
  }
  /*if(transitionTime>m_dt)
  {
    m_leftHandState = TASK_LEFT_HAND_TRANSITION;
    m_handsTransitionTime = m_t + transitionTime;
  }
  else
    m_leftHandState = TASK_LEFT_HAND_ON;*/
  m_leftHandState = TASK_LEFT_HAND_ON;
}

void InverseDynamicsBalanceController::addRightFootContact(const double& transitionTime) {
  if (m_contactState == LEFT_SUPPORT) {
    SEND_MSG("Add right foot contact in " + toString(transitionTime) + " s", MSG_TYPE_ERROR);
    const double& w_forces = m_w_forcesSIN.accessCopy();
    // TrajectorySample traj_ref = m_taskRF->getReference();
    pinocchio::SE3 ref;
    // vectorToSE3(traj_ref.pos, ref);
    ref = m_robot->position(m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
    m_contactRF->setReference(ref);
    Vector forceRef = Vector::Zero(6);
    forceRef[2] = 450.0;
    m_contactRF->setForceReference(forceRef);
    m_invDyn->removeTask(m_taskRF->name(), transitionTime);
    bool res = m_invDyn->addRigidContact(*m_contactRF, w_forces, 1e3, 1);
    if (!res) {
      const HQPData& hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
      SEND_MSG("Error while adding right foot contact." + tsid::solvers::HQPDataToString(hqpData, false),
               MSG_TYPE_ERROR);
    }
    m_contactState = DOUBLE_SUPPORT;
  }
}

void InverseDynamicsBalanceController::addLeftFootContact(const double& transitionTime) {
  if (m_contactState == RIGHT_SUPPORT) {
    SEND_MSG("Add left foot contact in " + toString(transitionTime) + " s", MSG_TYPE_ERROR);
    const double& w_forces = m_w_forcesSIN.accessCopy();
    // TrajectorySample traj_ref = m_taskLF->getReference();
    pinocchio::SE3 ref;
    // vectorToSE3(traj_ref.pos, ref);
    ref = m_robot->position(m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
    m_contactLF->setReference(ref);
    m_invDyn->removeTask(m_taskLF->name(), transitionTime);
    Vector forceRef = Vector::Zero(6);
    forceRef[2] = 450.0;
    m_contactLF->setForceReference(forceRef);
    bool res = m_invDyn->addRigidContact(*m_contactLF, w_forces, 1e3, 1);
    if (!res) {
      const HQPData& hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
      SEND_MSG("Error while adding left foot contact." + tsid::solvers::HQPDataToString(hqpData, false),
               MSG_TYPE_ERROR);
    }    
    m_contactState = DOUBLE_SUPPORT;
  }
}

void InverseDynamicsBalanceController::removeTaskRightHand(const double& transitionTime) {
  if (m_rightHandState == TASK_RIGHT_HAND_ON) {
    SEND_MSG("Removes right hand SE3 task in " + toString(transitionTime) + " s", MSG_TYPE_INFO);
    m_invDyn->removeTask(m_taskRH->name(), transitionTime);
    m_rightHandState = TASK_RIGHT_HAND_OFF;
  }
}

void InverseDynamicsBalanceController::removeTaskLeftHand(const double& transitionTime) {
  if (m_leftHandState == TASK_LEFT_HAND_ON) {
    SEND_MSG("Removes left hand SE3 task in " + toString(transitionTime) + " s", MSG_TYPE_INFO);
    m_invDyn->removeTask(m_taskLH->name(), transitionTime);
    m_leftHandState = TASK_LEFT_HAND_OFF;
  }
}

void InverseDynamicsBalanceController::addTaskEnergy(const double& transitionTime) {
  m_invDyn->addEnergyTask(*m_taskEnergy, 20, 1, transitionTime);
}

void InverseDynamicsBalanceController::removeTaskEnergy(const double& transitionTime) {
  m_invDyn->removeTask(m_taskEnergy->name(), transitionTime);
}

void InverseDynamicsBalanceController::setEnergyTank(const double& tankValue) {
  m_taskEnergy->set_E_tank(tankValue);
}

void InverseDynamicsBalanceController::init(const double& dt, const std::string& robotRef) {
  if (dt <= 0.0) return SEND_MSG("Init failed: Timestep must be positive", MSG_TYPE_ERROR);

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
  //        const Eigen::VectorXd w_forceReg = m_weight_contact_forcesSIN(0);
  const dg::sot::Vector6d& kp_contact = m_kp_constraintsSIN(0);
  const dg::sot::Vector6d& kd_contact = m_kd_constraintsSIN(0);
  const Eigen::Vector3d& kp_com = m_kp_comSIN(0);
  const Eigen::Vector3d& kd_com = m_kd_comSIN(0);
  const Eigen::Vector3d& kp_am = m_kp_amSIN(0);
  const Eigen::Vector3d& kd_am = m_kd_amSIN(0);
  const dg::sot::Vector6d& kd_hands = m_kd_handsSIN(0);
  const dg::sot::Vector6d& kp_hands = m_kp_handsSIN(0);
  const dg::sot::Vector6d& kp_feet = m_kp_feetSIN(0);
  const dg::sot::Vector6d& kd_feet = m_kd_feetSIN(0);
  const VectorN& kp_posture = m_kp_postureSIN(0);
  const VectorN& kd_posture = m_kd_postureSIN(0);
  const dg::sot::Vector6d& kp_base_orientation = m_kp_base_orientationSIN(0);
  const dg::sot::Vector6d& kd_base_orientation = m_kd_base_orientationSIN(0);

  assert(kp_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  assert(kd_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  m_w_hands = m_w_handsSIN(0);
  m_w_com = m_w_comSIN(0);
  m_w_am = m_w_amSIN(0);
  m_w_posture = m_w_postureSIN(0);
  m_w_base_orientation = m_w_base_orientationSIN(0);
  const double& w_forces = m_w_forcesSIN(0);
  //        const double & w_torques = m_w_torquesSIN(0);
  const double& mu = m_muSIN(0);
  const double& fMin = m_f_minSIN(0);
  const double& fMaxRF = m_f_max_right_footSIN(0);
  const double& fMaxLF = m_f_max_left_footSIN(0);

  try {
    vector<string> package_dirs;
    m_robot = new robots::RobotWrapper(m_robot_util->m_urdf_filename, package_dirs, pinocchio::JointModelFreeFlyer());

    assert(m_robot->nv() >= 6);
    m_robot_util->m_nbJoints = m_robot->nv() - 6;

    if (m_rotor_inertiasSIN.isPlugged() && m_gear_ratiosSIN.isPlugged()){
      const VectorN& rotor_inertias_sot = m_rotor_inertiasSIN(0);
      const VectorN& gear_ratios_sot = m_gear_ratiosSIN(0);
      assert(rotor_inertias_sot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
      assert(gear_ratios_sot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
      Vector rotor_inertias_urdf(rotor_inertias_sot.size());
      Vector gear_ratios_urdf(gear_ratios_sot.size());
      m_robot_util->joints_sot_to_urdf(rotor_inertias_sot, rotor_inertias_urdf);
      m_robot_util->joints_sot_to_urdf(gear_ratios_sot, gear_ratios_urdf);
      m_robot->rotor_inertias(rotor_inertias_urdf);
      m_robot->gear_ratios(gear_ratios_urdf);
    }   
    
    m_q_sot.setZero(m_robot->nv());
    m_v_sot.setZero(m_robot->nv());
    m_dv_sot.setZero(m_robot->nv());
    m_tau_sot.setZero(m_robot->nv() - 6);
    m_f.setZero(24);
    m_JF.setZero(m_robot->nv());
    m_q_urdf.setZero(m_robot->nq());
    m_v_urdf.setZero(m_robot->nv());
    m_dv_urdf.setZero(m_robot->nv());
    m_J_RF.setZero(6, m_robot->nv());
    m_J_LF.setZero(6, m_robot->nv());

    m_previous_vel.setZero(m_robot->nv());
    m_previous_q.setZero(m_robot->nv());
    m_previous_energy = 0.0;
    m_estim_data = pinocchio::Data(m_robot->model());

    m_invDyn = new InverseDynamicsFormulationAccForce("invdyn", *m_robot);

    // CONTACT 6D TASKS
    m_contactRF = new Contact6d("contact_rfoot", *m_robot, m_robot_util->m_foot_util.m_Right_Foot_Frame_Name,
                                contactPoints, contactNormal, mu, fMin, fMaxRF);
    m_contactRF->Kp(kp_contact);
    m_contactRF->Kd(kd_contact);
    Vector forceRef = Vector::Zero(6);
    forceRef[2] = 450.0;
    m_contactRF->setForceReference(forceRef);
    m_invDyn->addRigidContact(*m_contactRF, w_forces, 1e3, 1);

    m_contactLF = new Contact6d("contact_lfoot", *m_robot, m_robot_util->m_foot_util.m_Left_Foot_Frame_Name,
                                contactPoints, contactNormal, mu, fMin, fMaxLF);
    m_contactLF->Kp(kp_contact);
    m_contactLF->Kd(kd_contact);
    m_contactLF->setForceReference(forceRef);
    m_invDyn->addRigidContact(*m_contactLF, w_forces, 1e3, 1);

   if (m_f_ref_left_footSIN.isPlugged() && m_f_ref_right_footSIN.isPlugged()) {
     m_contactLF->setRegularizationTaskWeightVector(Vector6::Ones());
     m_contactRF->setRegularizationTaskWeightVector(Vector6::Ones());
   }

    // TASK COM
    m_taskCom = new TaskComEquality("task-com", *m_robot);
    m_taskCom->Kp(kp_com);
    m_taskCom->Kd(kd_com);
    if (m_ctrlMode == CONTROL_OUTPUT_VELOCITY){
      Eigen::VectorXd mask_com_height(3);
      mask_com_height << 0, 0, 1;
      m_taskCom->setMask(mask_com_height);

      m_taskComAdm  = new TaskComEquality("task-com-adm", *m_robot);
      m_taskComAdm->Kp(kp_com);
      m_taskComAdm->Kd(kd_com);
      Eigen::VectorXd mask_com_adm(3);
      mask_com_adm << 1, 1, 0;
      m_taskComAdm->setMask(mask_com_adm);
      m_invDyn->addMotionTask(*m_taskComAdm, m_w_com, 1);
    }
    // Eigen::VectorXd mask_com(3);
    // mask_com << 1, 1, 0;
    // m_taskCom->setMask(mask_com);
    m_invDyn->addMotionTask(*m_taskCom, m_w_com, 1);

    // TASK ANGULAR MOMENTUM
    m_taskAM = new TaskAMEquality("task-am", *m_robot);
    m_taskAM->Kp(kp_am);
    m_taskAM->Kd(kd_am);
    m_invDyn->addMotionTask(*m_taskAM, m_w_am, 1);

    // TASK BASE ORIENTATION
    m_taskWaist = new TaskSE3Equality("task-waist", *m_robot, "root_joint");
    m_taskWaist->Kp(kp_base_orientation);
    m_taskWaist->Kd(kd_base_orientation);
    // Add a Mask to the task which will select the vector dimensions on which the task will act.
    // In this case the waist configuration is a vector 6d (position and orientation -> SE3)
    // Here we set a mask = [0 0 0 1 1 1] so the task on the waist will act on the orientation of the robot
    Eigen::VectorXd mask_orientation(6);
    mask_orientation << 0, 0, 0, 1, 1, 1;
    m_taskWaist->setMask(mask_orientation);
    // Add the task to the HQP with weight = 1.0, priority level = 1 (in the cost function) and a transition duration = 0.0
    m_invDyn->addMotionTask(*m_taskWaist, m_w_base_orientation, 1);

    // FEET TASKS (not added yet to the invDyn pb, only when contacts are removed)
    m_taskRF = new TaskSE3Equality("task-rf", *m_robot, m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
    m_taskRF->Kp(kp_feet);
    m_taskRF->Kd(kd_feet);

    m_taskLF = new TaskSE3Equality("task-lf", *m_robot, m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);
    m_taskLF->Kp(kp_feet);
    m_taskLF->Kd(kd_feet);

    // POSTURE TASK
    m_taskPosture = new TaskJointPosture("task-posture", *m_robot);
    m_taskPosture->Kp(kp_posture);
    m_taskPosture->Kd(kd_posture);
    //Eigen::VectorXd mask_armR(m_robot->nv() - 6);
    //mask_armR = Vector::Ones(m_robot->nv() - 6);
    //mask_armR[25] = 0.0;
    //mask_armR.segment<8>(22) = Vector::Zero(8);
    
    //m_taskPosture->setMask(mask_armR);

    m_invDyn->addMotionTask(*m_taskPosture, m_w_posture, 1);

    // ACTUATION BOUNDS TASK
    Vector tau_max = 0.8 * m_robot->model().effortLimit.tail(m_robot->nv() - 6);
    std::cout << "################# tau_max : " << tau_max << " ############"  << std::endl;
    m_taskActBounds = new TaskActuationBounds("task-actuation-bounds", *m_robot);
    m_taskActBounds->setBounds(-tau_max, tau_max);
    m_invDyn->addActuationTask(*m_taskActBounds, 1.0, 0);

    // JOINT BOUNDS TASK
    m_taskJointBounds = new TaskJointBounds("task-joint-bounds", *m_robot, dt);
    Vector v_max = 0.8 * m_robot->model().velocityLimit.tail(m_robot->nv() - 6);
    m_taskJointBounds->setVelocityBounds(-v_max, v_max);
    //m_invDyn->addMotionTask(*m_taskJointBounds, 1.0, 0);

    // TASK CoP
    // m_taskCoP = new TaskCopEquality("task-cop", *m_robot);
    // m_invDyn->addForceTask(*m_taskCoP, 0.0, 1, 0);

    // HANDS TASKS (not added yet to the invDyn pb, only when the command addTaskXHand is called)
    m_taskRH = new TaskSE3Equality("task-rh", *m_robot, "gripper_right_base_link"); //m_robot_util->m_hand_util.m_Right_Hand_Frame_Name);
    m_taskRH->Kp(kp_hands);
    m_taskRH->Kd(kd_hands);
    Eigen::VectorXd mask_hands = Vector::Ones(6);
    mask_hands.tail(3) = Vector::Zero(3);
    m_taskRH->setMask(mask_hands);

    m_taskLH = new TaskSE3Equality("task-lh", *m_robot, m_robot_util->m_hand_util.m_Left_Hand_Frame_Name);
    m_taskLH->Kp(kp_hands);
    m_taskLH->Kd(kd_hands);
    m_taskLH->setMask(mask_hands);

    const VectorN6& q_robot = m_qSIN(0);
    assert(q_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
    const VectorN6& v_robot = m_vSIN(0);
    assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));

    // ENERGY TASK
    m_taskEnergy = new TaskEnergy("task-energy", *m_robot, q_robot, v_robot, dt, 0.003);
    Vector K_energy(m_robot_util->m_nbJoints + 6);
    //K_energy = 20.0 * Vector::Ones(m_robot_util->m_nbJoints + 6);
    K_energy.head<6>() = 0.0 * Vector::Ones(6);
    K_energy.tail(m_robot_util->m_nbJoints) = kp_posture;
    m_taskEnergy->K(K_energy);
    m_invDyn->addEnergyTask(*m_taskEnergy, 10, 1);

    // TRAJECTORIES INIT
    m_sampleCom = TrajectorySample(3);
    if (m_ctrlMode == CONTROL_OUTPUT_VELOCITY){
      m_sampleComAdm = TrajectorySample(3);
    }
    m_sampleAM = TrajectorySample(3);
    m_sampleWaist = TrajectorySample(6);
    m_samplePosture = TrajectorySample(m_robot->nv() - 6);
    m_sampleRH = TrajectorySample(6);
    m_sampleLH = TrajectorySample(6);
    m_sampleEnergy = TrajectorySample(m_robot->nv());

    m_frame_id_rf = (int)m_robot->model().getFrameId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
    m_frame_id_lf = (int)m_robot->model().getFrameId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);

    m_frame_id_rh = (int)m_robot->model().getFrameId("gripper_right_base_link");//m_robot_util->m_hand_util.m_Right_Hand_Frame_Name);
    m_frame_id_lh = (int)m_robot->model().getFrameId(m_robot_util->m_hand_util.m_Left_Hand_Frame_Name);
    
    m_robot_util->config_sot_to_urdf(q_robot, m_q_urdf);
    m_robot_util->velocity_sot_to_urdf(m_q_urdf, v_robot, m_v_urdf);
    pinocchio::Data data = pinocchio::Data(m_robot->model());
    pinocchio::centerOfMass(m_robot->model(), data, m_q_urdf, m_v_urdf);
    Vector3 com_estim = data.com[0];
    m_omega = std::sqrt(9.81 / com_estim[2]);

    // int waistFrameId = (int)m_robot->model().getFrameId("root_joint");
    // pinocchio::SE3 se3Waist;
    // m_robot->framePosition(data, waistFrameId, se3Waist);
    // SEND_MSG("se3Waist: " + toString(se3Waist), MSG_TYPE_ERROR);
    m_transformFrameCom = pinocchio::SE3::Identity();
    const Vector3& com_file = m_com_ref_posSIN(0);
    if (std::abs(com_file.sum() - com_estim.sum()) > 0.001){
      m_transformFrameCom.translation() = com_file - com_estim;
      SEND_MSG("m_transformFrameCom: " + toString(m_transformFrameCom), MSG_TYPE_ERROR);   
    }

    m_transformFrameFeet = pinocchio::SE3::Identity();
    const VectorN& foot_file = m_lf_ref_posSIN(0);
    pinocchio::SE3 oMi;
    m_robot->framePosition(data, m_frame_id_lf, oMi);
    Eigen::Matrix<double, 12, 1> left_foot;
    tsid::math::SE3ToVector(oMi, left_foot);
    if (std::abs(foot_file.sum() - left_foot.sum()) > 0.001){
      m_transformFrameFeet.translation() = foot_file.head<3>() - left_foot.head<3>() ;
      SEND_MSG("m_transformFrameFeet: " + toString(m_transformFrameFeet), MSG_TYPE_ERROR);   
    }

    // COM OFFSET
    if (m_com_measuredSIN.isPlugged()){
      const dg::Vector& com_measured = m_com_measuredSIN(0);
      assert(com_measured.size() == 3);
      SEND_MSG("COM_measured: " + toString(com_measured), MSG_TYPE_ERROR);   
      m_com_offset = com_measured - com_estim;
      SEND_MSG("Update COM: " + toString(m_com_offset), MSG_TYPE_ERROR);      
    }

    m_hqpSolver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST, "eiquadprog-fast");
    m_hqpSolver->resize(m_invDyn->nVar(), m_invDyn->nEq(), m_invDyn->nIn());
    m_hqpSolver_60_36_34 =
        SolverHQPFactory::createNewSolver<60, 36, 34>(SOLVER_HQP_EIQUADPROG_RT, "eiquadprog_rt_60_36_34");
    m_hqpSolver_48_30_17 =
        SolverHQPFactory::createNewSolver<48, 30, 17>(SOLVER_HQP_EIQUADPROG_RT, "eiquadprog_rt_48_30_17");
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
  if (s.size() != static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints)) s.resize(m_robot_util->m_nbJoints);

  const Eigen::VectorXd& active_joints_sot = m_active_jointsSIN(iter);
  if (m_enabled == false) {
    if (active_joints_sot.any()) {
      /* from all OFF to some ON */
      m_enabled = true;

      s = active_joints_sot;
      Eigen::VectorXd active_joints_urdf(m_robot_util->m_nbJoints);
      m_robot_util->joints_sot_to_urdf(active_joints_sot, active_joints_urdf);
      //            joints_sot_to_urdf(active_joints_sot, active_joints_urdf);

      m_taskBlockedJoints = new TaskJointPosture("task-posture", *m_robot);
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
      //m_invDyn->addMotionTask(*m_taskBlockedJoints, 1.0, 0);
    }
  } else if (!active_joints_sot.any()) {
    /* from some ON to all OFF */
    m_enabled = false;
  }
  if (m_enabled == false)
    for (unsigned int i = 0; i < m_robot_util->m_nbJoints; i++) s(i) = false;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(tau_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal tau_des before initialization!");
    return s;
  }
  if (s.size() != static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints)) s.resize(m_robot_util->m_nbJoints);

  getProfiler().start(PROFILE_TAU_DES_COMPUTATION);

  // use reference contact wrenches (if plugged) to determine contact phase
  if (m_f_ref_left_footSIN.isPlugged() && m_f_ref_right_footSIN.isPlugged()) {
    const Vector6& f_ref_left_foot = m_f_ref_left_footSIN(iter);
    const Vector6& f_ref_right_foot = m_f_ref_right_footSIN(iter);
    m_contactLF->setForceReference(f_ref_left_foot);
    m_contactRF->setForceReference(f_ref_right_foot);

    if (m_contactState == DOUBLE_SUPPORT) {
      if (f_ref_left_foot.norm() < ZERO_FORCE_THRESHOLD) {
        removeLeftFootContact(0.0);
      } else if (f_ref_right_foot.norm() < ZERO_FORCE_THRESHOLD) {        
        removeRightFootContact(0.0);
      }
    } else if (m_contactState == LEFT_SUPPORT && f_ref_right_foot.norm() > ZERO_FORCE_THRESHOLD) {
      addRightFootContact(0.0);
    } else if (m_contactState == RIGHT_SUPPORT && f_ref_left_foot.norm() > ZERO_FORCE_THRESHOLD) {
      addLeftFootContact(0.0);
    }
  }
  // use reference phases (if plugged) to determine contact phase
  if (m_ref_phaseSIN.isPlugged()) {
    ContactState ref_phase = ContactState(m_ref_phaseSIN(iter));

    if (m_contactState == DOUBLE_SUPPORT && ref_phase != DOUBLE_SUPPORT) {
      if (ref_phase == LEFT_SUPPORT) {
        removeRightFootContact(0.0);
      } else {        
        removeLeftFootContact(0.0);
      }
    } else if (m_contactState == LEFT_SUPPORT && ref_phase == DOUBLE_SUPPORT) {
      addRightFootContact(0.0);
    } else if (m_contactState == RIGHT_SUPPORT && ref_phase == DOUBLE_SUPPORT) {
      addLeftFootContact(0.0);
    }
  }

  if (m_contactState == RIGHT_SUPPORT_TRANSITION && m_t >= m_contactTransitionTime) {
    m_contactState = RIGHT_SUPPORT;
  } else if (m_contactState == LEFT_SUPPORT_TRANSITION && m_t >= m_contactTransitionTime) {
    m_contactState = LEFT_SUPPORT;
  }
  /*if(m_rightHandState == TASK_RIGHT_HAND_TRANSITION && m_t >= m_handsTransitionTime)
  {
        m_rightHandState = TASK_RIGHT_HAND_ON;
  }
  if(m_leftHandState == TASK_LEFT_HAND_TRANSITION && m_t >= m_handsTransitionTime)
  {
        m_leftHandState = TASK_LEFT_HAND_ON;
  }*/

  getProfiler().start(PROFILE_READ_INPUT_SIGNALS);
  m_w_feetSIN(iter);
  m_active_joints_checkedSINNER(iter);
  const VectorN6& q_sot = m_qSIN(iter);
  assert(q_sot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN6& v_sot = m_vSIN(iter);
  assert(v_sot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));

  const Vector3& x_com_ref = m_com_ref_posSIN(iter);
  const Vector3& dx_com_ref = m_com_ref_velSIN(iter);
  const Vector3& ddx_com_ref = m_com_ref_accSIN(iter);
  const Vector3& L_am_ref = m_am_ref_LSIN(iter);
  const Vector3& dL_am_ref = m_am_ref_dLSIN(iter);
  const VectorN& x_waist_ref = m_base_orientation_ref_posSIN(iter);
  const Vector6& dx_waist_ref = m_base_orientation_ref_velSIN(iter);
  const Vector6& ddx_waist_ref = m_base_orientation_ref_accSIN(iter);
  const VectorN& q_ref = m_posture_ref_posSIN(iter);
  assert(q_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& dq_ref = m_posture_ref_velSIN(iter);
  assert(dq_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& ddq_ref = m_posture_ref_accSIN(iter);
  assert(ddq_ref.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  const Vector6& kp_contact = m_kp_constraintsSIN(iter);
  const Vector6& kd_contact = m_kd_constraintsSIN(iter);
  const Vector3& kp_com = m_kp_comSIN(iter);
  const Vector3& kd_com = m_kd_comSIN(iter);
  const Vector3& kp_am = m_kp_amSIN(iter);
  const Vector3& kd_am = m_kd_amSIN(iter);
  const dg::sot::Vector6d& kp_base_orientation = m_kp_base_orientationSIN(0);
  const dg::sot::Vector6d& kd_base_orientation = m_kd_base_orientationSIN(0);

  const VectorN& kp_posture = m_kp_postureSIN(iter);
  assert(kp_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& kd_posture = m_kd_postureSIN(iter);
  assert(kd_posture.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  // const VectorN& kp_pos = m_kp_posSIN(iter);
  // assert(kp_pos.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  // const VectorN& kd_pos = m_kd_posSIN(iter);
  // assert(kd_pos.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  /*const double & w_hands = m_w_handsSIN(iter);*/
  const double& w_com = m_w_comSIN(iter);
  const double& w_am = m_w_amSIN(iter);
  const double& w_posture = m_w_postureSIN(iter);
  const double& w_forces = m_w_forcesSIN(iter);
  const double & w_base_orientation = m_w_base_orientationSIN(iter);

  getProfiler().stop(PROFILE_READ_INPUT_SIGNALS);

  if (m_contactState == LEFT_SUPPORT || m_contactState == LEFT_SUPPORT_TRANSITION) {
    const Eigen::Matrix<double, 12, 1>& x_rf_ref = m_rf_ref_posSIN(iter);
    const Vector6& dx_rf_ref = m_rf_ref_velSIN(iter);
    const Vector6& ddx_rf_ref = m_rf_ref_accSIN(iter);
    const Vector6& kp_feet = m_kp_feetSIN(iter);
    const Vector6& kd_feet = m_kd_feetSIN(iter);
    m_sampleRF.pos = actFrame(m_transformFrameFeet, x_rf_ref);
    m_sampleRF.vel = dx_rf_ref;
    m_sampleRF.acc = ddx_rf_ref;
    m_taskRF->setReference(m_sampleRF);
    m_taskRF->Kp(kp_feet);
    m_taskRF->Kd(kd_feet);

  } else if (m_contactState == RIGHT_SUPPORT || m_contactState == RIGHT_SUPPORT_TRANSITION) {
    const Eigen::Matrix<double, 12, 1>& x_lf_ref = m_lf_ref_posSIN(iter);
    const Vector6& dx_lf_ref = m_lf_ref_velSIN(iter);
    const Vector6& ddx_lf_ref = m_lf_ref_accSIN(iter);
    const Vector6& kp_feet = m_kp_feetSIN(iter);
    const Vector6& kd_feet = m_kd_feetSIN(iter);
    m_sampleLF.pos = actFrame(m_transformFrameFeet, x_lf_ref);
    m_sampleLF.vel = dx_lf_ref;
    m_sampleLF.acc = ddx_lf_ref;
    m_taskLF->setReference(m_sampleLF);
    m_taskLF->Kp(kp_feet);
    m_taskLF->Kd(kd_feet);
  }

  // if (m_contactState != DOUBLE_SUPPORT) {
  //   Vector3 zmpDes = (x_com_ref - m_com_offset) - ddx_com_ref/(m_omega*m_omega);
  //   zmpDes[2] = 0.0;
  //   m_taskCoP->setReference(zmpDes);
  //   m_invDyn->updateTaskWeight(m_taskCoP->name(), 1e-2);
  //   //m_invDyn->addForceTask(*m_taskCoP, 1e-3, 1, 0);
  // } else {
  //   m_invDyn->updateTaskWeight(m_taskCoP->name(), 0.0);
  //   // m_invDyn->removeTask(m_taskCoP->name(), 0.0);
  // }

  if (m_rightHandState == TASK_RIGHT_HAND_ON /*|| m_rightHandState == TASK_RIGHT_HAND_TRANSITION*/) {
    // std::cout << "m_rightHandState == TASK_RIGHT_HAND_ON" << std::endl;
    const Eigen::Matrix<double, 12, 1>& x_rh_ref = m_rh_ref_posSIN(iter);
    const Vector6& dx_rh_ref = m_rh_ref_velSIN(iter);
    const Vector6& ddx_rh_ref = m_rh_ref_accSIN(iter);
    const Vector6& kp_hands = m_kp_handsSIN(iter);
    const Vector6& kd_hands = m_kd_handsSIN(iter);
    Vector6 const_vel = 0.01 * Vector::Ones(6);
    m_sampleRH.pos = x_rh_ref;
    // m_sampleRH.vel = dx_rh_ref;
    // m_sampleRH.acc = ddx_rh_ref;
    m_sampleRH.vel = dx_rh_ref;
    m_taskRH->setReference(m_sampleRH);
    m_taskRH->Kp(kp_hands);
    m_taskRH->Kd(kd_hands);
  }
  if (m_leftHandState == TASK_LEFT_HAND_ON /*|| m_leftHandState == TASK_LEFT_HAND_TRANSITION*/) {
    const Eigen::Matrix<double, 12, 1>& x_lh_ref = m_lh_ref_posSIN(iter);
    const Vector6& dx_lh_ref = m_lh_ref_velSIN(iter);
    const Vector6& ddx_lh_ref = m_lh_ref_accSIN(iter);
    const Vector6& kp_hands = m_kp_handsSIN(iter);
    const Vector6& kd_hands = m_kd_handsSIN(iter);
    m_sampleLH.pos = x_lh_ref;
    m_sampleLH.vel = dx_lh_ref;
    m_sampleLH.acc = ddx_lh_ref;
    m_taskLH->setReference(m_sampleLH);
    m_taskLH->Kp(kp_hands);
    m_taskLH->Kd(kd_hands);
  }

  getProfiler().start(PROFILE_PREPARE_INV_DYN);
  
  if (m_ctrlMode == CONTROL_OUTPUT_VELOCITY){
    const Vector3& x_com_adm_ref = m_com_adm_ref_posSIN(iter);
    const Vector3& dx_com_adm_ref = m_com_adm_ref_velSIN(iter);
    const Vector3& ddx_com_adm_ref = m_com_adm_ref_accSIN(iter);
    m_sampleComAdm.pos = x_com_adm_ref;
    m_sampleComAdm.vel = dx_com_adm_ref;
    m_sampleCom.acc = ddx_com_adm_ref;
    m_taskComAdm->setReference(m_sampleComAdm);
    m_taskComAdm->Kp(kp_com);
    m_taskComAdm->Kd(kd_com);
    if (m_w_com != w_com) {
      //          SEND_MSG("Change w_com from "+toString(m_w_com)+" to "+toString(w_com), MSG_TYPE_INFO);
      m_w_com = w_com;
      m_invDyn->updateTaskWeight(m_taskComAdm->name(), w_com);
    }
  }
  m_sampleCom.pos = actFrame(m_transformFrameCom, x_com_ref) - m_com_offset; // x_com_ref - m_com_offset; //
  m_sampleCom.vel = dx_com_ref;
  m_sampleCom.acc = ddx_com_ref;
  m_taskCom->setReference(m_sampleCom);
  m_taskCom->Kp(kp_com);
  m_taskCom->Kd(kd_com);
  if (m_w_com != w_com) {
    //          SEND_MSG("Change w_com from "+toString(m_w_com)+" to "+toString(w_com), MSG_TYPE_INFO);
    m_w_com = w_com;
    m_invDyn->updateTaskWeight(m_taskCom->name(), w_com);
  }

  m_sampleAM.vel = L_am_ref;
  m_sampleAM.acc = dL_am_ref;
  m_taskAM->setReference(m_sampleAM);
  m_taskAM->Kp(kp_am);
  m_taskAM->Kd(kd_am);
  if (m_w_am != w_am) {
    //          SEND_MSG("Change w_am from "+toString(m_w_am)+" to "+toString(w_am), MSG_TYPE_INFO);
    m_w_am = w_am;
    m_invDyn->updateTaskWeight(m_taskAM->name(), w_am);
  }

  // const Eigen::Matrix<double, 12, 1>& x_lf_ref = m_lf_ref_posSIN(iter);
  // const Vector6& dx_lf_ref = m_lf_ref_velSIN(iter);
  // const Vector6& ddx_lf_ref = m_lf_ref_accSIN(iter);
  // const Eigen::Matrix<double, 12, 1>& x_rf_ref = m_rf_ref_posSIN(iter);
  // const Vector6& dx_rf_ref = m_rf_ref_velSIN(iter);
  // const Vector6& ddx_rf_ref = m_rf_ref_accSIN(iter);
  // Eigen::Matrix<double, 12, 1> sum_x = x_lf_ref + x_rf_ref;
  // m_sampleWaist.pos = 0.5 * actFrame(m_transformFrameFeet, sum_x); //x_waist_ref;
  // m_sampleWaist.vel = 0.5 * (dx_lf_ref + dx_rf_ref); //dx_waist_ref;
  // m_sampleWaist.acc = 0.5 * (ddx_lf_ref + ddx_rf_ref); //ddx_waist_ref;
  // VectorN waist_ref = x_waist_ref;
  // Vector6 dwaist_ref = dx_waist_ref;
  // Vector6 ddwaist_ref = ddx_waist_ref;
  // waist_ref.head<3>() = x_com_ref.head<3>() - m_transformFrameCom.translation() - m_com_offset;
  // dwaist_ref.head<3>() = dx_com_ref.head<3>();
  // ddwaist_ref.head<3>() = ddx_com_ref.head<3>();
  // m_sampleWaist.pos = waist_ref;
  // m_sampleWaist.vel = dwaist_ref;
  // m_sampleWaist.acc = ddx_waist_ref;
  // VectorN waist_ref = x_waist_ref;
  // waist_ref.head<3>() = x_com_ref.head<3>() - m_transformFrameCom.translation() - 2*m_com_offset;
  m_sampleWaist.pos = x_waist_ref;
  m_sampleWaist.vel = dx_waist_ref;
  m_sampleWaist.acc = ddx_waist_ref;
  m_taskWaist->setReference(m_sampleWaist);
  m_taskWaist->Kp(kp_base_orientation);
  m_taskWaist->Kd(kd_base_orientation);
  if (m_w_base_orientation != w_base_orientation) {
    m_w_base_orientation = w_base_orientation;
    m_invDyn->updateTaskWeight(m_taskWaist->name(), w_base_orientation);
  }

  m_robot_util->joints_sot_to_urdf(q_ref, m_samplePosture.pos);
  m_robot_util->joints_sot_to_urdf(dq_ref, m_samplePosture.vel);
  m_robot_util->joints_sot_to_urdf(ddq_ref, m_samplePosture.acc);
  m_taskPosture->setReference(m_samplePosture);
  m_taskPosture->Kp(kp_posture);
  m_taskPosture->Kd(kd_posture);
  if (m_w_posture != w_posture) {
    //          SEND_MSG("Change posture from "+toString(m_w_posture)+" to "+toString(w_posture), MSG_TYPE_INFO);
    m_w_posture = w_posture;
    m_invDyn->updateTaskWeight(m_taskPosture->name(), w_posture);
  }

  const VectorN& tau_measured = m_tau_measuredSIN(iter);
  assert(tau_measured.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const Vector& q_ref_final = m_ref_pos_finalSIN(iter);
  assert(q_ref_final.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  // Vector q_ref_final_urdf;
  // q_ref_final_urdf.setZero(39);
  // m_robot_util->config_sot_to_urdf(q_ref_final, q_ref_final_urdf);

  m_sampleEnergy.pos = q_ref_final;
  // m_sampleEnergy.vel = m_v_sot;
  m_sampleEnergy.acc = m_JF;
  // m_sampleEnergy.acc = Vector::Ones(38);
  // m_sampleEnergy.acc.head<6>() = 0.0 * Vector::Ones(6);
  // m_sampleEnergy.acc.tail(m_robot_util->m_nbJoints) = tau_measured - m_tau_sot;
  // m_robot_util->joints_sot_to_urdf(ddq_ref, m_sampleEnergy.acc);
  m_taskEnergy->setReference(m_sampleEnergy);

  /*m_sampleRH.pos = x_rh_ref;
  m_sampleRH.vel = dx_rh_ref;
  m_sampleRH.acc = ddx_rh_ref;
  m_taskRH->setReference(m_sampleRH);
  m_taskRH->Kp(kp_hands);
  m_taskRH->Kd(kd_hands);*/

  const double& fMin = m_f_minSIN(0);
  const double& fMaxRF = m_f_max_right_footSIN(iter);
  const double& fMaxLF = m_f_max_left_footSIN(iter);
  m_contactLF->setMinNormalForce(fMin);
  m_contactRF->setMinNormalForce(fMin);
  m_contactLF->setMaxNormalForce(fMaxLF);
  m_contactRF->setMaxNormalForce(fMaxRF);
  m_contactLF->Kp(kp_contact);
  m_contactLF->Kd(kd_contact);
  m_contactRF->Kp(kp_contact);
  m_contactRF->Kd(kd_contact);
  m_invDyn->updateRigidContactWeights(m_contactLF->name(), w_forces);
  m_invDyn->updateRigidContactWeights(m_contactRF->name(), w_forces);

  if (m_firstTime) {
    m_firstTime = false;
    m_robot_util->config_sot_to_urdf(q_sot, m_q_urdf);
    m_robot_util->velocity_sot_to_urdf(m_q_urdf, v_sot, m_v_urdf);
    m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
    m_previous_q = q_sot;
    //          m_robot->computeAllTerms(m_invDyn->data(), q, v);
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
  } else if (m_ctrlMode == CONTROL_OUTPUT_TORQUE){
    // In velocity close the TSID loop on itself (v_des, q_des), in torque on the (q,v) of the robot.
    m_robot_util->config_sot_to_urdf(q_sot, m_q_urdf);
    m_robot_util->velocity_sot_to_urdf(m_q_urdf, v_sot, m_v_urdf);
  }

  m_timeLast = static_cast<unsigned int>(iter);

  const HQPData& hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);

  getProfiler().stop(PROFILE_PREPARE_INV_DYN);

  getProfiler().start(PROFILE_HQP_SOLUTION);
  SolverHQPBase* solver = m_hqpSolver;
  if (m_invDyn->nVar() == 60 && m_invDyn->nEq() == 36 && m_invDyn->nIn() == 34) {
    solver = m_hqpSolver_60_36_34;
    getStatistics().store("solver fixed size 60_36_34", 1.0);
  } else if (m_invDyn->nVar() == 48 && m_invDyn->nEq() == 30 && m_invDyn->nIn() == 17) {
    solver = m_hqpSolver_48_30_17;
    getStatistics().store("solver fixed size 48_30_17", 1.0);
  } else
    getStatistics().store("solver dynamic size", 1.0);

  const HQPOutput& sol = solver->solve(hqpData);
  getProfiler().stop(PROFILE_HQP_SOLUTION);

  if (sol.status != HQP_STATUS_OPTIMAL) {
    SEND_ERROR_STREAM_MSG("HQP solver failed to find a solution at iter : "+ toString(iter) + "error " + toString(sol.status));
    SEND_ERROR_STREAM_MSG(tsid::solvers::HQPDataToString(hqpData, false));
    SEND_ERROR_STREAM_MSG("q=" + toString(q_sot.transpose()));
    SEND_ERROR_STREAM_MSG("v=" + toString(v_sot.transpose()));
    s.setZero();
    return s;
  }

  getStatistics().store("active inequalities", static_cast<double>(sol.activeSet.size()));
  getStatistics().store("solver iterations", sol.iterations);
  if (ddx_com_ref.norm() > 1e-3)
    getStatistics().store("com ff ratio", ddx_com_ref.norm() / m_taskCom->getConstraint().vector().norm());

  m_dv_urdf = m_invDyn->getAccelerations(sol);
  m_JF = m_invDyn->getJContactForces(sol);
  m_robot_util->velocity_urdf_to_sot(m_q_urdf, m_dv_urdf, m_dv_sot);
  Eigen::Matrix<double, 12, 1> tmp;
  if (m_invDyn->getContactForces(m_contactRF->name(), sol, tmp)) m_f_RF = m_contactRF->getForceGeneratorMatrix() * tmp;
  if (m_invDyn->getContactForces(m_contactLF->name(), sol, tmp)) m_f_LF = m_contactLF->getForceGeneratorMatrix() * tmp;
  m_robot_util->joints_urdf_to_sot(m_invDyn->getActuatorForces(sol), m_tau_sot);

  getProfiler().stop(PROFILE_TAU_DES_COMPUTATION);
  m_t += m_dt;

  s = m_tau_sot;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(M, dynamicgraph::Matrix) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal M before initialization!");
    return s;
  }
  if (s.cols() != m_robot->nv() || s.rows() != m_robot->nv()) s.resize(m_robot->nv(), m_robot->nv());
  m_tau_desSOUT(iter);
  s = m_robot->mass(m_invDyn->data());
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dv_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dv_des before initialization!");
    return s;
  }
  if (s.size() != m_robot->nv()) s.resize(m_robot->nv());
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

DEFINE_SIGNAL_OUT_FUNCTION(tau_pd_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal tau_pd_des before initialization!");
    return s;
  }
  if (s.size() != static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints))
    s.resize(m_robot_util->m_nbJoints);

  const VectorN& kp_pos = m_kp_posSIN(iter);
  assert(kp_pos.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& kd_pos = m_kd_posSIN(iter);
  assert(kd_pos.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  VectorN kp_tau = m_kp_tauSIN(iter);
  assert(kp_tau.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& kff_tau = m_kff_tauSIN(iter);
  assert(kff_tau.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  const VectorN& kff_dq = m_kff_dqSIN(iter);
  assert(kff_dq.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  const VectorN6& q_robot = m_qSIN(iter);
  assert(q_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN6& v_robot = m_vSIN(iter);
  assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN& tau_measured = m_tau_measuredSIN(iter);
  assert(tau_measured.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));

  m_q_desSOUT(iter);
  if (tau_measured.sum() < 100) {
    kp_tau.setZero();
  }

  s = kff_tau.cwiseProduct(m_tau_sot);
   // + kp_tau.cwiseProduct(tau_measured - m_tau_sot) +
   //    kff_dq.cwiseProduct(m_v_sot.tail(m_robot_util->m_nbJoints)) +
   //    kp_pos.cwiseProduct(m_q_sot.tail(m_robot_util->m_nbJoints) - q_robot.tail(m_robot_util->m_nbJoints)) +
   //    kd_pos.cwiseProduct(m_v_sot.tail(m_robot_util->m_nbJoints) - v_robot.tail(m_robot_util->m_nbJoints));

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(f_des_right_foot, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal f_des_right_foot before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);
  m_tau_desSOUT(iter);
  if (m_contactState == LEFT_SUPPORT) {
    s.setZero();
    return s;
  }
  s = m_f_RF;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(f_des_left_foot, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal f_des_left_foot before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);
  m_tau_desSOUT(iter);
  if (m_contactState == RIGHT_SUPPORT) {
    s.setZero();
    return s;
  }
  s = m_f_LF;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(com_acc_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal com_acc_des before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);
  m_tau_desSOUT(iter);
  s = m_taskCom->getDesiredAcceleration();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(com_acc, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal com_acc before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);
  m_tau_desSOUT(iter);
  s = m_taskCom->getAcceleration(m_dv_urdf);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(am_dL_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal am_dL_des before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);
  m_tau_desSOUT(iter);
  s = m_taskAM->getDesiredMomentumDerivative();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(am_dL, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal am_dL before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);
  m_tau_desSOUT(iter);
  s = m_taskAM->getdMomentum(m_dv_urdf);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(am_L, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal am_L before initialization!");
    return s;
  }
  if (s.size() != 3) s.resize(3);
  m_tau_desSOUT(iter);
  s = m_taskAM->momentum();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp_des_right_foot_local, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des_right_foot_local before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);

  m_f_des_right_footSOUT(iter);
  if (fabs(m_f_RF(2) > 1.0)) {
    m_zmp_des_RF_local(0) = -m_f_RF(4) / m_f_RF(2);
    m_zmp_des_RF_local(1) = m_f_RF(3) / m_f_RF(2);
    m_zmp_des_RF_local(2) = 0.0;
  } else
    m_zmp_des_RF_local.setZero();

  s = m_zmp_des_RF_local.head<2>();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp_des_left_foot_local, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des_left_foot_local before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);
  m_f_des_left_footSOUT(iter);
  if (fabs(m_f_LF(2) > 1.0)) {
    m_zmp_des_LF_local(0) = -m_f_LF(4) / m_f_LF(2);
    m_zmp_des_LF_local(1) = m_f_LF(3) / m_f_LF(2);
    m_zmp_des_LF_local(2) = 0.0;
  } else
    m_zmp_des_LF_local.setZero();

  s = m_zmp_des_LF_local.head<2>();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp_des_right_foot, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des_right_foot before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);
  m_f_des_right_footSOUT(iter);
  pinocchio::SE3 H_rf = m_robot->position(
      m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
  if (fabs(m_f_RF(2) > 1.0)) {
    m_zmp_des_RF(0) = -m_f_RF(4) / m_f_RF(2);
    m_zmp_des_RF(1) = m_f_RF(3) / m_f_RF(2);
    m_zmp_des_RF(2) = -H_rf.translation()(2);
  } else
    m_zmp_des_RF.setZero();

  m_zmp_des_RF = H_rf.act(m_zmp_des_RF);
  s = m_zmp_des_RF.head<2>();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp_des_left_foot, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des_left_foot before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);
  m_f_des_left_footSOUT(iter);
  pinocchio::SE3 H_lf = m_robot->position(
      m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
  if (fabs(m_f_LF(2) > 1.0)) {
    m_zmp_des_LF(0) = -m_f_LF(4) / m_f_LF(2);
    m_zmp_des_LF(1) = m_f_LF(3) / m_f_LF(2);
    m_zmp_des_LF(2) = -H_lf.translation()(2);
  } else
    m_zmp_des_LF.setZero();

  m_zmp_des_LF = H_lf.act(m_zmp_des_LF);
  s = m_zmp_des_LF.head<2>();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);
  m_zmp_des_left_footSOUT(iter);
  m_zmp_des_right_footSOUT(iter);

  m_zmp_des = (m_f_RF(2) * m_zmp_des_RF + m_f_LF(2) * m_zmp_des_LF) / (m_f_LF(2) + m_f_RF(2));
  // s = m_taskCoP->getReference().head<2>(); //m_zmp_des.head<2>();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp_ref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_ref before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);
  const Vector6& f_LF = m_f_ref_left_footSIN(iter);
  const Vector6& f_RF = m_f_ref_right_footSIN(iter);

  pinocchio::SE3 H_lf = m_robot->position(
      m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
  Vector3 zmp_LF, zmp_RF;
  if (fabs(f_LF(2) > 1.0)) {
    zmp_LF(0) = -f_LF(4) / f_LF(2);
    zmp_LF(1) = f_LF(3) / f_LF(2);
    zmp_LF(2) = -H_lf.translation()(2);
  } else
    zmp_LF.setZero();
  zmp_LF = H_lf.act(zmp_LF);

  pinocchio::SE3 H_rf = m_robot->position(
      m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
  if (fabs(f_RF(2) > 1.0)) {
    zmp_RF(0) = -f_RF(4) / f_RF(2);
    zmp_RF(1) = f_RF(3) / f_RF(2);
    zmp_RF(2) = -H_rf.translation()(2);
  } else
    zmp_RF.setZero();
  zmp_RF = H_rf.act(zmp_RF);

  if (f_LF(2) + f_RF(2) != 0.0) s = (f_RF(2) * zmp_RF.head<2>() + f_LF(2) * zmp_LF.head<2>()) / (f_LF(2) + f_RF(2));

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp_right_foot, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_right_foot before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);
  const Vector6& f = m_wrench_right_footSIN(iter);
  assert(f.size() == 6);
  pinocchio::SE3 H_rf = m_robot->position(
      m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
  if (fabs(f(2) > 1.0)) {
    m_zmp_RF(0) = -f(4) / f(2);
    m_zmp_RF(1) = f(3) / f(2);
    m_zmp_RF(2) = -H_rf.translation()(2);
  } else
    m_zmp_RF.setZero();

  m_zmp_RF = H_rf.act(m_zmp_RF);
  s = m_zmp_RF.head<2>();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp_left_foot, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_left_foot before initialization!");
    return s;
  }
  if (s.size() != 2) s.resize(2);
  const Vector6& f = m_wrench_left_footSIN(iter);
  pinocchio::SE3 H_lf = m_robot->position(
      m_invDyn->data(), m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
  if (fabs(f(2) > 1.0)) {
    m_zmp_LF(0) = -f(4) / f(2);
    m_zmp_LF(1) = f(3) / f(2);
    m_zmp_LF(2) = -H_lf.translation()(2);
  } else
    m_zmp_LF.setZero();

  m_zmp_LF = H_lf.act(m_zmp_LF);
  s = m_zmp_LF.head<2>();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(zmp, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal zmp before initialization!");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  if (s.size() != 3) s.resize(3);
  const Vector6& f_LF = m_wrench_left_footSIN(iter);
  const Vector6& f_RF = m_wrench_right_footSIN(iter);
  m_zmp_left_footSOUT(iter);
  m_zmp_right_footSOUT(iter);

  if (f_LF(2) + f_RF(2) > 1.0) m_zmp = (f_RF(2) * m_zmp_RF + f_LF(2) * m_zmp_LF) / (f_LF(2) + f_RF(2));
  s = m_zmp.head<2>();
  s(2) = 0.0;
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
  pinocchio::SE3 invCom = m_transformFrameCom;
  invCom.translation() = -m_transformFrameCom.translation();
  s = actFrame(invCom, com + m_com_offset);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(com_est, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal com_est before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  if (s.size() != 3) s.resize(3);

  const VectorN6& q_sot = m_qSIN(iter);
  assert(q_sot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN6& v_sot = m_vSIN(iter);
  assert(v_sot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));

  Vector q_base_estimator, v_base_estimator;
  q_base_estimator.setZero(m_robot->nq());
  v_base_estimator.setZero(m_robot->nv());
  m_robot_util->config_sot_to_urdf(q_sot, q_base_estimator);
  m_robot_util->velocity_sot_to_urdf(q_base_estimator, v_sot, v_base_estimator);

  pinocchio::Data data = pinocchio::Data(m_robot->model());
  pinocchio::centerOfMass(m_robot->model(), data, q_base_estimator, v_base_estimator);
  s = data.com[0];
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(com_vel, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal com_vel before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  if (s.size() != 3) s.resize(3);
  const Vector3& com_vel = m_robot->com_vel(m_invDyn->data());
  s = com_vel;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dcm, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal dcm before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  if (s.size() != 3) s.resize(3);
  const Vector3& com = m_comSOUT(iter);
  const Vector3& com_vel = m_com_velSOUT(iter);
  s = com + com_vel / m_omega;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(base_orientation, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal base_orientation before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  m_tau_desSOUT(iter);
  pinocchio::SE3 oMi;
  int frame_id_waist = (int)m_robot->model().getFrameId("root_joint");
  m_robot->framePosition(m_invDyn->data(), frame_id_waist, oMi);
  s.resize(12);
  tsid::math::SE3ToVector(oMi, s);
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
  s.resize(12);
  m_robot->framePosition(m_invDyn->data(), m_frame_id_lf, oMi);
  tsid::math::SE3ToVector(oMi, s);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_foot_pos_quat, dynamicgraph::Vector) {
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
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_foot_pos_ref_quat, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal left_foot_pos before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  // m_tau_desSOUT(iter);
  dg::Vector x_lf_ref = m_lf_ref_posSIN(iter);
  x_lf_ref = actFrame(m_transformFrameFeet, x_lf_ref);
  pinocchio::SE3 oMi;
  oMi.translation(x_lf_ref.head<3>() );
  oMi.rotation( Eigen::Map<const Eigen::Matrix<double,3,3> >(&x_lf_ref(3), 3, 3) );
  s.resize(7);
  // m_robot->framePosition(m_invDyn->data(), m_frame_id_lf, oMi);
  tsid::math::SE3ToXYZQUAT(oMi, s);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(lf_est, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal lf_est before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  if (s.size() != 3) s.resize(3);

  m_tau_desSOUT(iter);
  pinocchio::SE3 lf_se3;
  s.resize(12);
  m_robot->framePosition(m_estim_data, m_frame_id_lf, lf_se3);
  tsid::math::SE3ToVector(lf_se3, s);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_hand_pos, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal left hand_pos before initialization!");
    return s;
  }
  m_tau_desSOUT(iter);
  pinocchio::SE3 oMi;
  s.resize(12);
  m_robot->framePosition(m_invDyn->data(), m_frame_id_lh, oMi);
  tsid::math::SE3ToVector(oMi, s);
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
  s.resize(12);
  m_robot->framePosition(m_invDyn->data(), m_frame_id_rf, oMi);
  tsid::math::SE3ToVector(oMi, s);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_foot_pos_quat, dynamicgraph::Vector) {
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
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_foot_pos_ref_quat, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal rigt_foot_pos before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  // m_tau_desSOUT(iter);
  dg::Vector x_rf_ref = m_rf_ref_posSIN(iter);
  x_rf_ref = actFrame(m_transformFrameFeet, x_rf_ref);
  pinocchio::SE3 oMi;
  oMi.translation(x_rf_ref.head<3>() );
  oMi.rotation( Eigen::Map<const Eigen::Matrix<double,3,3> >(&x_rf_ref(3), 3, 3) );
  s.resize(7);
  // m_robot->framePosition(m_invDyn->data(), m_frame_id_rf, oMi);
  tsid::math::SE3ToXYZQUAT(oMi, s);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(rf_est, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal rf_est before initialization! iter:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  if (s.size() != 3) s.resize(3);

  m_tau_desSOUT(iter);
  pinocchio::SE3 rf_se3;
  s.resize(12);
  m_robot->framePosition(m_estim_data, m_frame_id_rf, rf_se3);
  tsid::math::SE3ToVector(rf_se3, s);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_hand_pos, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal right_hand_pos before initialization!");
    return s;
  }
  m_tau_desSOUT(iter);
  pinocchio::SE3 oMi;
  s.resize(12);
  m_robot->framePosition(m_invDyn->data(), m_frame_id_rh, oMi);
  tsid::math::SE3ToVector(oMi, s);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_foot_vel, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal left_foot_vel before initialization!");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  pinocchio::Motion v;
  m_robot->frameVelocity(m_invDyn->data(), m_frame_id_lf, v);
  s = v.toVector();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_hand_vel, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    std::ostringstream oss("Cannot compute signal left_hand_vel before initialization!:");
    oss << iter;
    SEND_WARNING_STREAM_MSG(oss.str());
    return s;
  }
  pinocchio::Motion v;
  m_robot->frameVelocity(m_invDyn->data(), m_frame_id_lh, v);
  s = v.toVector();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_foot_vel, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal right_foot_vel before initialization! iter:" + toString(iter));
    return s;
  }
  pinocchio::Motion v;
  m_robot->frameVelocity(m_invDyn->data(), m_frame_id_rf, v);
  s = v.toVector();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_hand_vel, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal right_hand_vel before initialization! " + toString(iter));
    return s;
  }
  pinocchio::Motion v;
  m_robot->frameVelocity(m_invDyn->data(), m_frame_id_rh, v);
  s = v.toVector();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_foot_acc, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal left_foot_acc before initialization! " + toString(iter));
    return s;
  }
  m_tau_desSOUT(iter);
  if (m_contactState == RIGHT_SUPPORT)
    s = m_taskLF->getAcceleration(m_dv_urdf);
  else
    s = m_contactLF->getMotionTask().getAcceleration(m_dv_urdf);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_hand_acc, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal left_hand_acc before initialization!");
    return s;
  }
  m_tau_desSOUT(iter);
  s = m_contactLH->getMotionTask().getAcceleration(m_dv_urdf);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_foot_acc, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal right_foot_acc before initialization!");
    return s;
  }
  m_tau_desSOUT(iter);
  if (m_contactState == LEFT_SUPPORT)
    s = m_taskRF->getAcceleration(m_dv_urdf);
  else
    s = m_contactRF->getMotionTask().getAcceleration(m_dv_urdf);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_hand_acc, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal right_hand_acc before initialization!");
    return s;
  }
  m_tau_desSOUT(iter);
  s = m_contactRH->getMotionTask().getAcceleration(m_dv_urdf);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(left_foot_acc_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal left_foot_acc_des before initialization!");
    return s;
  }
  m_tau_desSOUT(iter);
  if (m_contactState == RIGHT_SUPPORT)
    s = m_taskLF->getDesiredAcceleration();
  else
    s = m_contactLF->getMotionTask().getDesiredAcceleration();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(right_foot_acc_des, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal right_foot_acc_des before initialization!");
    return s;
  }
  m_tau_desSOUT(iter);
  if (m_contactState == LEFT_SUPPORT)
    s = m_taskRF->getDesiredAcceleration();
  else
    s = m_contactRF->getMotionTask().getDesiredAcceleration();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(energy, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal energy before initialization!");
    return s;
  }
  //const VectorN6& q_robot = m_qSIN(iter);
  //assert(q_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  //const VectorN6& v_robot = m_vSIN(iter);
  //assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  //const VectorN& kp_posture = m_kp_postureSIN(iter);
  // assert(K_energy.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  //const Vector& q_ref_final = m_ref_pos_finalSIN(iter);
  //assert(q_ref_final.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  // std::cout << "##################### INV DYN ENERGY ################################" << std::endl;
  // std::cout << "q: "  << q_robot << std::endl;
  // std::cout << "v: "  << v_robot << std::endl;
  //Vector K_energy(m_robot_util->m_nbJoints + 6);
  //K_energy = 20.0 * Vector::Ones(m_robot_util->m_nbJoints + 6);
  //K_energy.head<6>() = 0.0 * Vector::Ones(6);
  //K_energy.tail(m_robot_util->m_nbJoints) = kp_posture;
  //m_q_desSOUT(iter);
  //Vector error_pos = q_robot - q_ref_final;
  // std::cout << "############## mass matrix: " << m_invDyn->data().M.bottomRows(m_robot_util->m_nbJoints)  << " ############" << std::endl;
  //s = 0.5 * v_robot.transpose() * m_robot->mass(m_invDyn->data()) * v_robot;
  //s += 0.5 * error_pos.transpose() * K_energy.cwiseProduct(error_pos);
  s = m_taskEnergy->get_H();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(energy_derivative, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal energy_derivative before initialization!");
    return s;
  }
  // double energy = m_energySOUT(iter);
  // std::cout << "############## previous_energy: " << previous_energy << " ############" << std::endl;
  // std::cout << "############## energy: " << energy << " ############" << std::endl;
  // s = (energy - m_previous_energy)/m_dt;
  // m_q_desSOUT(iter);
  // double E_c = m_taskEnergy->get_E_c();
  // double E_p = m_taskEnergy->get_E_p();
  // s = E_c + E_p;
  // m_previous_energy = energy;
  // Vector BK = m_taskEnergy->get_BK_vector();
  // double BKv = BK.transpose() * m_previous_vel;
  // s = BKv/m_dt;
  s = m_taskEnergy->get_dH();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(energy_tank, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal energy_tank before initialization!");
    return s;
  }
  //double m_time_preview = 0.003;
  //const VectorN6& q_robot = m_qSIN(iter);
  //assert(q_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  //const VectorN6& v_robot = m_vSIN(iter);
  //assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  // //const VectorN6& previous_vel = m_vSIN(iter-1);
  // //std::cout << "############## previous_vel: " << previous_vel << " ############" << std::endl;
  // //assert(previous_vel.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));  
  //const VectorN& kp_posture = m_kp_postureSIN(iter);
  // //assert(K_energy.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  //const Vector& q_ref_final = m_ref_pos_finalSIN(iter);
  //assert(q_ref_final.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));  

  //Vector K_energy(m_robot_util->m_nbJoints + 6);
  //K_energy = 20.0 * Vector::Ones(m_robot_util->m_nbJoints + 6);
  // //K_energy.head<6>() = 0.0 * Vector::Ones(6);
  // //K_energy.tail(m_robot_util->m_nbJoints) = kp_posture;
  //m_q_desSOUT(iter);

  //Vector error_pos = q_robot - q_ref_final;
  //Vector error_pos_prev = q_ref_final - m_previous_q;
  //double time_ratio = (m_time_preview * m_time_preview)/(2*m_dt);
  //Vector v_des_t = error_pos_prev * time_ratio;
  //Vector a_des_t = ((error_pos_prev/m_dt) - m_previous_vel) * time_ratio;

  //m_previous_q = q_robot;
  //m_previous_vel = v_robot;

  // E_c
  // s = v_robot.transpose() * m_robot->mass(m_invDyn->data()) * v_robot;
  // dE_p
  //s = (error_pos*m_time_preview + v_des_t).transpose() * K_energy.cwiseProduct(m_previous_vel/m_dt + m_dv_sot);
  // s = v_robot.transpose() * K_energy.cwiseProduct(error_pos);
  // dE_c
  //s += ((1/m_dt) * (v_robot*m_time_preview) + a_des_t).transpose() * m_robot->mass(m_invDyn->data()) * m_dv_sot;
  s = m_taskEnergy->get_E_tank();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(energy_bound, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal energy_bound before initialization!");
    return s;
  }
  const VectorN6& v_robot = m_vSIN(iter);
  assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  const VectorN& tau_measured = m_tau_measuredSIN(iter);
  // assert(tau_measured.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints));
  m_tau_desSOUT(iter);
  s = - v_robot.tail(m_robot_util->m_nbJoints).transpose() * tau_measured;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(task_energy_const, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal task_energy_const before initialization!");
    return s;
  }
  m_q_desSOUT(iter);
  const VectorN6& v_robot = m_vSIN(iter);
  assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  // Vector A = m_taskEnergy->get_A();
  // Vector BK = m_taskEnergy->get_BK_vector();
  double vJF = v_robot.transpose() * m_JF;
  double AMdv = (- v_robot.transpose() * m_robot->mass(m_invDyn->data()))* m_dv_sot; //  + BK.transpose()
  s = AMdv + vJF;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(task_energy_bound, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal task_energy_bound before initialization!");
    return s;
  }
  m_q_desSOUT(iter);
  const VectorN6& v_robot = m_vSIN(iter);
  assert(v_robot.size() == static_cast<Eigen::VectorXd::Index>(m_robot_util->m_nbJoints + 6));
  // double vJF = v_robot.transpose() * m_JF;
  double lowB = m_taskEnergy->get_lowerBound() + v_robot.transpose() * m_robot->nonLinearEffects(m_invDyn->data());
  s = lowB;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(task_energy_alpha, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal task_energy_alpha before initialization!");
    return s;
  }
  s = m_taskEnergy->get_alpha();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(task_energy_beta, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal task_energy_beta before initialization!");
    return s;
  }
  s = m_taskEnergy->get_beta();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(task_energy_gamma, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal task_energy_gamma before initialization!");
    return s;
  }
  s = m_taskEnergy->get_gamma();
  return s;
}


DEFINE_SIGNAL_OUT_FUNCTION(task_energy_S, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal task_energy_S before initialization!");
    return s;
  }
  s = m_taskEnergy->get_S();
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(task_energy_A, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal task_energy_A before initialization!");
    return s;
  }
  Vector A = m_taskEnergy->get_A_vector();
  s = A.sum();
  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void InverseDynamicsBalanceController::display(std::ostream& os) const {
  os << "InverseDynamicsBalanceController " << getName();
  try {
    getProfiler().report_all(3, os);
    getStatistics().report_all(1, os);
    os << "QP size: nVar " << m_invDyn->nVar() << " nEq " << m_invDyn->nEq() << " nIn " << m_invDyn->nIn() << "\n";
  } catch (ExceptionSignal e) {
  }
}
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph
