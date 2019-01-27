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

#define EIGEN_RUNTIME_NO_MALLOC

#include <sot/torque_control/inverse-dynamics-balance-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>

#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-rt.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/math/utils.hpp>

#include <boost/test/unit_test.hpp>

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
      using namespace tsid;
      using namespace tsid::trajectories;
      using namespace tsid::math;
      using namespace tsid::contacts;
      using namespace tsid::tasks;
      using namespace tsid::solvers;

      typedef SolverHQuadProgRT<60,36,34> SolverHQuadProgRT60x36x34;
      typedef SolverHQuadProgRT<48,30,17> SolverHQuadProgRT48x30x17;


#define REQUIRE_FINITE(A) assert(is_finite(A))

      //Size to be aligned                "-------------------------------------------------------"
#define PROFILE_TAU_DES_COMPUTATION "InvDynBalCtrl: desired tau"
#define PROFILE_HQP_SOLUTION        "InvDynBalCtrl: HQP"
#define PROFILE_PREPARE_INV_DYN     "InvDynBalCtrl: prepare inv-dyn"
#define PROFILE_READ_INPUT_SIGNALS  "InvDynBalCtrl: read input signals"

#define ZERO_FORCE_THRESHOLD 1e-3

#define INPUT_SIGNALS         m_com_ref_posSIN \
  << m_com_ref_velSIN \
  << m_com_ref_accSIN \
  << m_rf_ref_posSIN \
  << m_rf_ref_velSIN \
  << m_rf_ref_accSIN \
  << m_lf_ref_posSIN \
  << m_lf_ref_velSIN \
  << m_lf_ref_accSIN \
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
  << m_kp_feetSIN \
  << m_kd_feetSIN \
  << m_kp_postureSIN \
  << m_kd_postureSIN \
  << m_kp_posSIN \
  << m_kd_posSIN \
  << m_w_comSIN \
  << m_w_feetSIN \
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
  << m_tau_estimatedSIN \
  << m_qSIN \
  << m_vSIN \
  << m_wrench_baseSIN \
  << m_active_jointsSIN \
  << m_wrench_left_footSIN  \
  << m_wrench_right_footSIN

#define OUTPUT_SIGNALS        m_tau_desSOUT \
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
  << m_com_velSOUT \
  << m_com_accSOUT \
  << m_com_acc_desSOUT \
  << m_base_orientationSOUT \
  << m_right_foot_posSOUT \
  << m_left_foot_posSOUT \
  << m_right_foot_velSOUT \
  << m_left_foot_velSOUT \
  << m_right_foot_accSOUT \
  << m_left_foot_accSOUT \
  << m_right_foot_acc_desSOUT \
  << m_left_foot_acc_desSOUT \
  << m_dv_desSOUT \
  << m_MSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef InverseDynamicsBalanceController EntityClassName;

      typedef Eigen::Matrix<double,2,1> Vector2;
      typedef Eigen::Matrix<double,Eigen::Dynamic,1> VectorN;
      typedef Eigen::Matrix<double,Eigen::Dynamic,1> VectorN6;
      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(InverseDynamicsBalanceController,
                                         "InverseDynamicsBalanceController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      InverseDynamicsBalanceController::
          InverseDynamicsBalanceController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(com_ref_pos,                 dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(com_ref_vel,                 dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(com_ref_acc,                 dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(rf_ref_pos,                  dynamicgraph::Vector)
    	    ,CONSTRUCT_SIGNAL_IN(rf_ref_vel,                  dynamicgraph::Vector)
    	    ,CONSTRUCT_SIGNAL_IN(rf_ref_acc,                  dynamicgraph::Vector)
    	    ,CONSTRUCT_SIGNAL_IN(lf_ref_pos,                  dynamicgraph::Vector)
    	    ,CONSTRUCT_SIGNAL_IN(lf_ref_vel,                  dynamicgraph::Vector)
    	    ,CONSTRUCT_SIGNAL_IN(lf_ref_acc,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_pos,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_vel,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_acc,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_pos,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_vel,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_acc,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(f_ref_right_foot,            dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(f_ref_left_foot,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_base_orientation,         dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_base_orientation,         dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_constraints,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_constraints,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_com,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_com,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_feet,                     dynamicgraph::Vector)
    	    ,CONSTRUCT_SIGNAL_IN(kd_feet,                     dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_posture,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_posture,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_pos,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_pos,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_com,                       double)
     	    ,CONSTRUCT_SIGNAL_IN(w_feet,                      double)
            ,CONSTRUCT_SIGNAL_IN(w_posture,                   double)
            ,CONSTRUCT_SIGNAL_IN(w_base_orientation,          double)
            ,CONSTRUCT_SIGNAL_IN(w_torques,                   double)
            ,CONSTRUCT_SIGNAL_IN(w_forces,                    double)
            ,CONSTRUCT_SIGNAL_IN(mu,                          double)
            ,CONSTRUCT_SIGNAL_IN(f_min,                       double)
            ,CONSTRUCT_SIGNAL_IN(f_max_right_foot,            double)
            ,CONSTRUCT_SIGNAL_IN(f_max_left_foot,             double)
            ,CONSTRUCT_SIGNAL_IN(dt_joint_pos_limits,         double)
            ,CONSTRUCT_SIGNAL_IN(weight_contact_forces,       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(contact_points,              dynamicgraph::Matrix)
            ,CONSTRUCT_SIGNAL_IN(contact_normal,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(rotor_inertias,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(gear_ratios,                 dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(tau_max,                     dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(q_min,                       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(q_max,                       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(dq_max,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(ddq_max,                     dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(tau_estimated,               dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(q,                           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(v,                           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(wrench_base,                 dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(wrench_left_foot,            dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(wrench_right_foot,           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(active_joints,               dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_OUT(tau_des,                    dynamicgraph::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(f_des_right_foot,           dynamicgraph::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(f_des_left_foot,            dynamicgraph::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(zmp_des_right_foot,         dynamicgraph::Vector, m_f_des_right_footSOUT)
            ,CONSTRUCT_SIGNAL_OUT(zmp_des_left_foot,          dynamicgraph::Vector, m_f_des_left_footSOUT)
            ,CONSTRUCT_SIGNAL_OUT(zmp_des_right_foot_local,   dynamicgraph::Vector, m_f_des_right_footSOUT)
            ,CONSTRUCT_SIGNAL_OUT(zmp_des_left_foot_local,    dynamicgraph::Vector, m_f_des_left_footSOUT)
            ,CONSTRUCT_SIGNAL_OUT(zmp_des,                    dynamicgraph::Vector, m_zmp_des_left_footSOUT<<
                                                                                    m_zmp_des_right_footSOUT)
            ,CONSTRUCT_SIGNAL_OUT(zmp_ref,                    dynamicgraph::Vector, m_f_ref_left_footSIN<<
                                                                                    m_f_ref_right_footSIN)
            ,CONSTRUCT_SIGNAL_OUT(zmp_right_foot,             dg::Vector, m_wrench_right_footSIN)
            ,CONSTRUCT_SIGNAL_OUT(zmp_left_foot,              dg::Vector, m_wrench_left_footSIN)
            ,CONSTRUCT_SIGNAL_OUT(zmp,                        dg::Vector, m_wrench_left_footSIN<<
                                                                          m_wrench_right_footSIN<<
                                                                          m_zmp_left_footSOUT<<
                                                                          m_zmp_right_footSOUT)
            ,CONSTRUCT_SIGNAL_OUT(dv_des,                     dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(M,                          dg::Matrix, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(com,                        dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(com_vel,                    dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(com_acc,                    dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(com_acc_des,                dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(base_orientation,           dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(left_foot_pos,              dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(right_foot_pos,             dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(left_foot_vel,              dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(right_foot_vel,             dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(left_foot_acc,              dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(right_foot_acc,             dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(left_foot_acc_des,          dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(right_foot_acc_des,         dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_INNER(active_joints_checked,    dg::Vector, m_active_jointsSIN)
            ,m_initSucceeded(false)
            ,m_enabled(false)
            ,m_t(0.0)
            ,m_firstTime(true)
            ,m_timeLast(0)
            ,m_contactState(DOUBLE_SUPPORT)
	    ,m_robot_util(RefVoidRobotUtil())
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        m_zmp_des_RF.setZero();
        m_zmp_des_LF.setZero();
        m_zmp_des_RF_local.setZero();
        m_zmp_des_LF_local.setZero();
        m_zmp_des.setZero();
        m_zmp_RF.setZero();
        m_zmp_LF.setZero();
        m_zmp.setZero();
        m_com_offset.setZero();
        m_v_RF_int.setZero();
        m_v_LF_int.setZero();

        /* Commands. */
        addCommand("init",
                   makeCommandVoid2(*this, &InverseDynamicsBalanceController::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "Time period in seconds (double)",
						    "Robot reference (string)")));

        addCommand("updateComOffset",
                   makeCommandVoid0(*this, &InverseDynamicsBalanceController::updateComOffset,
                                    docCommandVoid0("Update the offset on the CoM based on the CoP measurement.")));

        addCommand("removeRightFootContact",
                   makeCommandVoid1(*this, &InverseDynamicsBalanceController::removeRightFootContact,
                                    docCommandVoid1("Remove the contact at the right foot.",
                                                    "Transition time in seconds (double)")));

        addCommand("removeLeftFootContact",
                   makeCommandVoid1(*this, &InverseDynamicsBalanceController::removeLeftFootContact,
                                    docCommandVoid1("Remove the contact at the left foot.",
                                                    "Transition time in seconds (double)")));


      }

      void InverseDynamicsBalanceController::updateComOffset()
      {
        const Vector3 & com = m_robot->com(m_invDyn->data());
        m_com_offset = m_zmp - com;
        m_com_offset(2) = 0.0;
        SEND_MSG("CoM offset updated: "+toString(m_com_offset), MSG_TYPE_INFO);
      }

      void InverseDynamicsBalanceController::removeRightFootContact(const double& transitionTime)
      {
        if(m_contactState == DOUBLE_SUPPORT)
        {
          SEND_MSG("Remove right foot contact in "+toString(transitionTime)+" s", MSG_TYPE_INFO);
          bool res = m_invDyn->removeRigidContact(m_contactRF->name(), transitionTime);
          if(!res)
          {
            const HQPData & hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
            SEND_MSG("Error while remove right foot contact."+tsid::solvers::HQPDataToString(hqpData, false), MSG_TYPE_ERROR);
          }
          const double & w_feet = m_w_feetSIN.accessCopy();
          m_invDyn->addMotionTask(*m_taskRF, w_feet, 1);
          if(transitionTime>m_dt)
          {
            m_contactState = LEFT_SUPPORT_TRANSITION;
            m_contactTransitionTime = m_t + transitionTime;
          }
          else
            m_contactState = LEFT_SUPPORT;
        }
      }

      void InverseDynamicsBalanceController::removeLeftFootContact(const double& transitionTime)
      {
        if(m_contactState == DOUBLE_SUPPORT)
        {
          SEND_MSG("Remove left foot contact in "+toString(transitionTime)+" s", MSG_TYPE_INFO);
          bool res = m_invDyn->removeRigidContact(m_contactLF->name(), transitionTime);
          if(!res)
          {
            const HQPData & hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
            SEND_MSG("Error while remove right foot contact."+tsid::solvers::HQPDataToString(hqpData, false), MSG_TYPE_ERROR);
          }
          const double & w_feet = m_w_feetSIN.accessCopy();
          m_invDyn->addMotionTask(*m_taskLF, w_feet, 1);
          if(transitionTime>m_dt)
          {
            m_contactState = RIGHT_SUPPORT_TRANSITION;
            m_contactTransitionTime = m_t + transitionTime;
          }
          else
            m_contactState = RIGHT_SUPPORT;
        }
      }

      void InverseDynamicsBalanceController::addRightFootContact(const double& transitionTime)
      {
        if(m_contactState == LEFT_SUPPORT)
        {
          SEND_MSG("Add right foot contact in "+toString(transitionTime)+" s", MSG_TYPE_INFO);
          m_invDyn->addRigidContact(*m_contactRF);
          m_invDyn->removeTask(m_taskRF->name(), transitionTime);
          m_contactState = DOUBLE_SUPPORT;
        }
      }

      void InverseDynamicsBalanceController::addLeftFootContact(const double& transitionTime)
      {
        if(m_contactState == RIGHT_SUPPORT)
        {
          SEND_MSG("Add left foot contact in "+toString(transitionTime)+" s", MSG_TYPE_INFO);
          m_invDyn->addRigidContact(*m_contactLF);
          m_invDyn->removeTask(m_taskLF->name(), transitionTime);
          m_contactState = DOUBLE_SUPPORT;
        }
      }

      void InverseDynamicsBalanceController::init(const double& dt,
						  const std::string& robotRef)
      {
        if(dt<=0.0)
          return SEND_MSG("Init failed: Timestep must be positive", MSG_TYPE_ERROR);

        /* Retrieve m_robot_util  informations */
        std::string localName(robotRef);
        if (isNameInRobotUtil(localName))
          m_robot_util = getRobotUtil(localName);
        else
        {
          SEND_MSG("You should have an entity controller manager initialized before",MSG_TYPE_ERROR);
          return;
        }

        const Eigen::Matrix<double, 3, 4>& contactPoints = m_contact_pointsSIN(0);
        const Eigen::Vector3d& contactNormal = m_contact_normalSIN(0);
//        const Eigen::VectorXd w_forceReg = m_weight_contact_forcesSIN(0);
        const Eigen::Vector6d& kp_contact = m_kp_constraintsSIN(0);
        const Eigen::Vector6d& kd_contact = m_kd_constraintsSIN(0);
        const Eigen::Vector3d& kp_com = m_kp_comSIN(0);
        const Eigen::Vector3d& kd_com = m_kd_comSIN(0);
        const Eigen::Vector6d& kp_feet = m_kp_feetSIN(0);
        const Eigen::Vector6d& kd_feet = m_kd_feetSIN(0);
        const VectorN& kp_posture = m_kp_postureSIN(0);
        const VectorN& kd_posture = m_kd_postureSIN(0);
        const VectorN& rotor_inertias_sot = m_rotor_inertiasSIN(0);
        const VectorN& gear_ratios_sot = m_gear_ratiosSIN(0);

        assert(kp_posture.size()==m_robot_util->m_nbJoints);
        assert(kd_posture.size()==m_robot_util->m_nbJoints);
        assert(rotor_inertias_sot.size()==m_robot_util->m_nbJoints);
        assert(gear_ratios_sot.size()==m_robot_util->m_nbJoints);

        m_w_com = m_w_comSIN(0);
        m_w_posture = m_w_postureSIN(0);
        const double & w_forces = m_w_forcesSIN(0);
//        const double & w_base_orientation = m_w_base_orientationSIN(0);
//        const double & w_torques = m_w_torquesSIN(0);
        const double & mu = m_muSIN(0);
        const double & fMin = m_f_minSIN(0);
        const double & fMaxRF = m_f_max_right_footSIN(0);
        const double & fMaxLF = m_f_max_left_footSIN(0);

        try
        {
          vector<string> package_dirs;
          m_robot = new robots::RobotWrapper(m_robot_util->m_urdf_filename,
					     package_dirs,
					     pinocchio::JointModelFreeFlyer());

          assert(m_robot->nv()>=6);
	  m_robot_util->m_nbJoints = m_robot->nv()-6;

          Vector rotor_inertias_urdf(rotor_inertias_sot.size());
          Vector gear_ratios_urdf(gear_ratios_sot.size());
          m_robot_util->joints_sot_to_urdf(rotor_inertias_sot, rotor_inertias_urdf);
          m_robot_util->joints_sot_to_urdf(gear_ratios_sot, gear_ratios_urdf);
          m_robot->rotor_inertias(rotor_inertias_urdf);
          m_robot->gear_ratios(gear_ratios_urdf);

          m_dv_sot.setZero(m_robot->nv());
          m_tau_sot.setZero(m_robot->nv()-6);
          m_f.setZero(24);
          m_q_urdf.setZero(m_robot->nq());
          m_v_urdf.setZero(m_robot->nv());
          m_J_RF.setZero(6, m_robot->nv());
          m_J_LF.setZero(6, m_robot->nv());

          m_invDyn = new InverseDynamicsFormulationAccForce("invdyn", *m_robot);

          m_contactRF = new Contact6d("contact_rfoot", *m_robot,
				      m_robot_util->m_foot_util.m_Right_Foot_Frame_Name,
                                      contactPoints, contactNormal,
                                      mu, fMin, fMaxRF, w_forces);
          m_contactRF->Kp(kp_contact);
          m_contactRF->Kd(kd_contact);
          m_invDyn->addRigidContact(*m_contactRF);

          m_contactLF = new Contact6d("contact_lfoot", *m_robot,
				      m_robot_util->m_foot_util.m_Left_Foot_Frame_Name,
                                      contactPoints, contactNormal,
                                      mu, fMin, fMaxLF, w_forces);
          m_contactLF->Kp(kp_contact);
          m_contactLF->Kd(kd_contact);
          m_invDyn->addRigidContact(*m_contactLF);

          if(m_f_ref_left_footSIN.isPlugged() && m_f_ref_right_footSIN.isPlugged())
          {
            m_contactLF->setRegularizationTaskWeightVector(Vector6::Ones());
            m_contactRF->setRegularizationTaskWeightVector(Vector6::Ones());
          }

          m_taskCom = new TaskComEquality("task-com", *m_robot);
          m_taskCom->Kp(kp_com);
          m_taskCom->Kd(kd_com);
          m_invDyn->addMotionTask(*m_taskCom, m_w_com, 1);

          m_taskRF = new TaskSE3Equality("task-rf", *m_robot, m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
          m_taskRF->Kp(kp_feet);
          m_taskRF->Kd(kd_feet);

          m_taskLF = new TaskSE3Equality("task-lf", *m_robot, m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);
          m_taskLF->Kp(kp_feet);
          m_taskLF->Kd(kd_feet);

          m_taskPosture = new TaskJointPosture("task-posture", *m_robot);
          m_taskPosture->Kp(kp_posture);
          m_taskPosture->Kd(kd_posture);
          m_invDyn->addMotionTask(*m_taskPosture, m_w_posture, 1);

          m_sampleCom = TrajectorySample(3);
          m_samplePosture = TrajectorySample(m_robot->nv()-6);

          m_frame_id_rf = (int)m_robot->model().getFrameId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
          m_frame_id_lf = (int)m_robot->model().getFrameId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);

          m_hqpSolver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                          "eiquadprog-fast");
          m_hqpSolver->resize(m_invDyn->nVar(), m_invDyn->nEq(), m_invDyn->nIn());
          m_hqpSolver_60_36_34 = SolverHQPFactory::createNewSolver<60,36,34>(SOLVER_HQP_EIQUADPROG_RT,
                                                                             "eiquadprog_rt_60_36_34");
          m_hqpSolver_48_30_17 = SolverHQPFactory::createNewSolver<48,30,17>(SOLVER_HQP_EIQUADPROG_RT,
                                                                             "eiquadprog_rt_48_30_17");
        }
        catch (const std::exception& e)
        {
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
      DEFINE_SIGNAL_INNER_FUNCTION(active_joints_checked, dynamicgraph::Vector)
      {
        if(s.size()!=m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);

        const Eigen::VectorXd& active_joints_sot = m_active_jointsSIN(iter);
        if (m_enabled == false)
        {
          if (active_joints_sot.any())
          {

            /* from all OFF to some ON */
            m_enabled = true ;

            s = active_joints_sot;
            Eigen::VectorXd active_joints_urdf(m_robot_util->m_nbJoints);
            m_robot_util->joints_sot_to_urdf(active_joints_sot, active_joints_urdf);
//            joints_sot_to_urdf(active_joints_sot, active_joints_urdf);

            m_taskBlockedJoints = new TaskJointPosture("task-posture", *m_robot);
            Eigen::VectorXd blocked_joints(m_robot_util->m_nbJoints);
            for(unsigned int i=0; i<m_robot_util->m_nbJoints; i++)
              if(active_joints_urdf(i)==0.0)
                blocked_joints(i) = 1.0;
              else
                blocked_joints(i) = 0.0;
            SEND_MSG("Blocked joints: "+toString(blocked_joints.transpose()), MSG_TYPE_INFO);
            m_taskBlockedJoints->mask(blocked_joints);
            TrajectorySample ref_zero(m_robot_util->m_nbJoints);
            m_taskBlockedJoints->setReference(ref_zero);
            m_invDyn->addMotionTask(*m_taskBlockedJoints, 1.0, 0);
          }
        }
        else if (!active_joints_sot.any())
        {
          /* from some ON to all OFF */
          m_enabled = false ;
        }
        if (m_enabled == false)
          for(int i=0; i<m_robot_util->m_nbJoints; i++)
            s(i)=false;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(tau_des,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tau_des before initialization!");
          return s;
        }
        if(s.size()!=m_robot_util->m_nbJoints)
          s.resize(m_robot_util->m_nbJoints);

        getProfiler().start(PROFILE_TAU_DES_COMPUTATION);

        // use reference contact wrenches (if plugged) to determine contact phase
        if(m_f_ref_left_footSIN.isPlugged() && m_f_ref_right_footSIN.isPlugged())
        {
          const Vector6 & f_ref_left_foot  = m_f_ref_left_footSIN(iter);
          const Vector6 & f_ref_right_foot = m_f_ref_right_footSIN(iter);
          m_contactLF->setForceReference(f_ref_left_foot);
          m_contactRF->setForceReference(f_ref_right_foot);

          if(m_contactState == DOUBLE_SUPPORT)
          {
            if(f_ref_left_foot.norm() < ZERO_FORCE_THRESHOLD)
            {
              removeLeftFootContact(0.0);
            }
            else if(f_ref_right_foot.norm() < ZERO_FORCE_THRESHOLD)
            {
              removeRightFootContact(0.0);
            }
          }
          else if(m_contactState == LEFT_SUPPORT && f_ref_right_foot.norm()>ZERO_FORCE_THRESHOLD)
          {
            addRightFootContact(0.0);
          }
          else if(m_contactState == RIGHT_SUPPORT && f_ref_left_foot.norm()>ZERO_FORCE_THRESHOLD)
          {
            addLeftFootContact(0.0);
          }
        }

        if(m_contactState == RIGHT_SUPPORT_TRANSITION && m_t >= m_contactTransitionTime)
        {
          m_contactState = RIGHT_SUPPORT;
        }
        else if(m_contactState == LEFT_SUPPORT_TRANSITION && m_t >= m_contactTransitionTime)
        {
          m_contactState = LEFT_SUPPORT;
        }

        getProfiler().start(PROFILE_READ_INPUT_SIGNALS);
        m_w_feetSIN(iter);
        m_active_joints_checkedSINNER(iter);
        const VectorN6& q_sot = m_qSIN(iter);
        assert(q_sot.size()==m_robot_util->m_nbJoints+6);
        const VectorN6& v_sot = m_vSIN(iter);
        assert(v_sot.size()==m_robot_util->m_nbJoints+6);
        const Vector3& x_com_ref =   m_com_ref_posSIN(iter);
        const Vector3& dx_com_ref =  m_com_ref_velSIN(iter);
        const Vector3& ddx_com_ref = m_com_ref_accSIN(iter);
        const VectorN& q_ref =   m_posture_ref_posSIN(iter);
        assert(q_ref.size()==m_robot_util->m_nbJoints);
        const VectorN& dq_ref =  m_posture_ref_velSIN(iter);
        assert(dq_ref.size()==m_robot_util->m_nbJoints);
        const VectorN& ddq_ref = m_posture_ref_accSIN(iter);
        assert(ddq_ref.size()==m_robot_util->m_nbJoints);

        const Vector6& kp_contact = m_kp_constraintsSIN(iter);
        const Vector6& kd_contact = m_kd_constraintsSIN(iter);
        const Vector3& kp_com = m_kp_comSIN(iter);
        const Vector3& kd_com = m_kd_comSIN(iter);

        const VectorN& kp_posture = m_kp_postureSIN(iter);
        assert(kp_posture.size()==m_robot_util->m_nbJoints);
        const VectorN& kd_posture = m_kd_postureSIN(iter);
        assert(kd_posture.size()==m_robot_util->m_nbJoints);
        const VectorN& kp_pos = m_kp_posSIN(iter);
        assert(kp_pos.size()==m_robot_util->m_nbJoints);
        const VectorN& kd_pos = m_kd_posSIN(iter);
        assert(kd_pos.size()==m_robot_util->m_nbJoints);

        const double & w_com = m_w_comSIN(iter);
        const double & w_posture = m_w_postureSIN(iter);
        const double & w_forces = m_w_forcesSIN(iter);

        if(m_contactState == LEFT_SUPPORT || m_contactState == LEFT_SUPPORT_TRANSITION)
        {
          const Eigen::Matrix<double,12,1>& x_rf_ref = m_rf_ref_posSIN(iter);
          const Vector6& dx_rf_ref =  m_rf_ref_velSIN(iter);
          const Vector6& ddx_rf_ref = m_rf_ref_accSIN(iter);
          const Vector6& kp_feet = m_kp_feetSIN(iter);
          const Vector6& kd_feet = m_kd_feetSIN(iter);
          m_sampleRF.pos = x_rf_ref;
          m_sampleRF.vel = dx_rf_ref;
          m_sampleRF.acc = ddx_rf_ref;
          m_taskRF->setReference(m_sampleRF);
          m_taskRF->Kp(kp_feet);
          m_taskRF->Kd(kd_feet);
        }
        else if(m_contactState==RIGHT_SUPPORT || m_contactState==RIGHT_SUPPORT_TRANSITION)
        {
          const Eigen::Matrix<double,12,1>& x_lf_ref = m_lf_ref_posSIN(iter);
          const Vector6& dx_lf_ref = m_lf_ref_velSIN(iter);
          const Vector6& ddx_lf_ref = m_lf_ref_accSIN(iter);
          const Vector6& kp_feet = m_kp_feetSIN(iter);
          const Vector6& kd_feet = m_kd_feetSIN(iter);
          m_sampleLF.pos = x_lf_ref;
          m_sampleLF.vel = dx_lf_ref;
          m_sampleLF.acc = ddx_lf_ref;
          m_taskLF->setReference(m_sampleLF);
          m_taskLF->Kp(kp_feet);
          m_taskLF->Kd(kd_feet);
        }

        getProfiler().stop(PROFILE_READ_INPUT_SIGNALS);

        getProfiler().start(PROFILE_PREPARE_INV_DYN);
        m_robot_util->config_sot_to_urdf(q_sot, m_q_urdf);
        m_robot_util->velocity_sot_to_urdf(m_q_urdf, v_sot, m_v_urdf);

        m_sampleCom.pos = x_com_ref - m_com_offset;
        m_sampleCom.vel = dx_com_ref;
        m_sampleCom.acc = ddx_com_ref;
        m_taskCom->setReference(m_sampleCom);
        m_taskCom->Kp(kp_com);
        m_taskCom->Kd(kd_com);
        if(m_w_com != w_com)
        {
//          SEND_MSG("Change w_com from "+toString(m_w_com)+" to "+toString(w_com), MSG_TYPE_INFO);
          m_w_com = w_com;
          m_invDyn->updateTaskWeight(m_taskCom->name(), w_com);
        }

        m_robot_util->joints_sot_to_urdf(q_ref, m_samplePosture.pos);
        m_robot_util->joints_sot_to_urdf(dq_ref, m_samplePosture.vel);
        m_robot_util->joints_sot_to_urdf(ddq_ref, m_samplePosture.acc);
        m_taskPosture->setReference(m_samplePosture);
        m_taskPosture->Kp(kp_posture);
        m_taskPosture->Kd(kd_posture);
        if(m_w_posture != w_posture)
        {
//          SEND_MSG("Change posture from "+toString(m_w_posture)+" to "+toString(w_posture), MSG_TYPE_INFO);
          m_w_posture = w_posture;
          m_invDyn->updateTaskWeight(m_taskPosture->name(), w_posture);
        }

        const double & fMin = m_f_minSIN(0);
        const double & fMaxRF = m_f_max_right_footSIN(iter);
        const double & fMaxLF = m_f_max_left_footSIN(iter);
        m_contactLF->setMinNormalForce(fMin);
        m_contactRF->setMinNormalForce(fMin);
        m_contactLF->setMaxNormalForce(fMaxLF);
        m_contactRF->setMaxNormalForce(fMaxRF);
        m_contactLF->Kp(kp_contact);
        m_contactLF->Kd(kd_contact);
        m_contactLF->setRegularizationTaskWeight(w_forces);
        m_contactRF->Kp(kp_contact);
        m_contactRF->Kd(kd_contact);
        m_contactRF->setRegularizationTaskWeight(w_forces);

        if(m_firstTime)
        {
          m_firstTime = false;
          m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
          //          m_robot->computeAllTerms(m_invDyn->data(), q, v);
          pinocchio::SE3 H_lf = m_robot->position(m_invDyn->data(),
                                            m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
          m_contactLF->setReference(H_lf);
          SEND_MSG("Setting left foot reference to "+toString(H_lf), MSG_TYPE_DEBUG);

          pinocchio::SE3 H_rf = m_robot->position(m_invDyn->data(),
                                            m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
          m_contactRF->setReference(H_rf);
          SEND_MSG("Setting right foot reference to "+toString(H_rf), MSG_TYPE_DEBUG);
        }
        else if(m_timeLast != iter-1)
        {
          SEND_MSG("Last time "+toString(m_timeLast)+" is not current time-1: "+toString(iter), MSG_TYPE_ERROR);
          if(m_timeLast == iter)
          {
            s = m_tau_sot;
            return s;
          }
        }
        m_timeLast = iter;

        const HQPData & hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
        getProfiler().stop(PROFILE_PREPARE_INV_DYN);

        getProfiler().start(PROFILE_HQP_SOLUTION);
        SolverHQPBase * solver = m_hqpSolver;
        if(m_invDyn->nVar()==60 && m_invDyn->nEq()==36 && m_invDyn->nIn()==34)
        {
          solver = m_hqpSolver_60_36_34;
          getStatistics().store("solver fixed size 60_36_34", 1.0);
        }
        else if(m_invDyn->nVar()==48 && m_invDyn->nEq()==30 && m_invDyn->nIn()==17)
        {
          solver = m_hqpSolver_48_30_17;
          getStatistics().store("solver fixed size 48_30_17", 1.0);
        }
        else
          getStatistics().store("solver dynamic size", 1.0);

        const HQPOutput & sol = solver->solve(hqpData);
        getProfiler().stop(PROFILE_HQP_SOLUTION);

        if(sol.status!=HQP_STATUS_OPTIMAL)
        {
          SEND_ERROR_STREAM_MSG("HQP solver failed to find a solution: "+toString(sol.status));
          SEND_DEBUG_STREAM_MSG(tsid::solvers::HQPDataToString(hqpData, false));
          SEND_DEBUG_STREAM_MSG("q="+toString(q_sot.transpose(),1,5));
          SEND_DEBUG_STREAM_MSG("v="+toString(v_sot.transpose(),1,5));
          s.setZero();
          return s;
        }

        getStatistics().store("active inequalities", sol.activeSet.size());
        getStatistics().store("solver iterations", sol.iterations);
        if(ddx_com_ref.norm()>1e-3)
          getStatistics().store("com ff ratio", ddx_com_ref.norm()/m_taskCom->getConstraint().vector().norm());

        m_dv_urdf = m_invDyn->getAccelerations(sol);
        m_robot_util->velocity_urdf_to_sot(m_q_urdf, m_dv_urdf, m_dv_sot);
        Eigen::Matrix<double,12,1> tmp;
        if(m_invDyn->getContactForces(m_contactRF->name(), sol, tmp))
          m_f_RF = m_contactRF->getForceGeneratorMatrix() * tmp;
        if(m_invDyn->getContactForces(m_contactLF->name(), sol, tmp))
          m_f_LF = m_contactLF->getForceGeneratorMatrix() * tmp;
        m_robot_util->joints_urdf_to_sot(m_invDyn->getActuatorForces(sol), m_tau_sot);

        m_tau_sot += kp_pos.cwiseProduct(q_ref-q_sot.tail(m_robot_util->m_nbJoints)) +
                     kd_pos.cwiseProduct(dq_ref-v_sot.tail(m_robot_util->m_nbJoints));

        getProfiler().stop(PROFILE_TAU_DES_COMPUTATION);
        m_t += m_dt;

        s = m_tau_sot;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(M,dynamicgraph::Matrix)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal M before initialization!");
          return s;
        }
        if(s.cols()!=m_robot->nv() || s.rows()!=m_robot->nv())
          s.resize(m_robot->nv(), m_robot->nv());
        m_tau_desSOUT(iter);
        s = m_robot->mass(m_invDyn->data());
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dv_des,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal dv_des before initialization!");
          return s;
        }
        if(s.size()!=m_robot->nv())
          s.resize(m_robot->nv());
        m_tau_desSOUT(iter);
        s = m_dv_sot;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(f_des_right_foot,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal f_des_right_foot before initialization!");
          return s;
        }
        if(s.size()!=6)
          s.resize(6);
        m_tau_desSOUT(iter);
        if(m_contactState == LEFT_SUPPORT)
        {
          s.setZero();
          return s;
        }
        s = m_f_RF;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(f_des_left_foot,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal f_des_left_foot before initialization!");
          return s;
        }
        if(s.size()!=6)
          s.resize(6);
        m_tau_desSOUT(iter);
        if(m_contactState == RIGHT_SUPPORT)
        {
          s.setZero();
          return s;
        }
        s = m_f_LF;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(com_acc_des, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com_acc_des before initialization!");
          return s;
        }
        if(s.size()!=3)
          s.resize(3);
        m_tau_desSOUT(iter);
        s = m_taskCom->getDesiredAcceleration();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(com_acc, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com_acc before initialization!");
          return s;
        }
        if(s.size()!=3)
          s.resize(3);
        m_tau_desSOUT(iter);
        s = m_taskCom->getAcceleration(m_dv_urdf);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp_des_right_foot_local,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des_right_foot_local before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);

        m_f_des_right_footSOUT(iter);
        if(fabs(m_f_RF(2)>1.0))
        {
          m_zmp_des_RF_local(0) = -m_f_RF(4) / m_f_RF(2);
          m_zmp_des_RF_local(1) =  m_f_RF(3) / m_f_RF(2);
          m_zmp_des_RF_local(2) = 0.0;
        }
        else
          m_zmp_des_RF_local.setZero();

        s = m_zmp_des_RF_local.head<2>();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp_des_left_foot_local,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des_left_foot_local before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);
        m_f_des_left_footSOUT(iter);
        if(fabs(m_f_LF(2)>1.0))
        {
          m_zmp_des_LF_local(0) = -m_f_LF(4) / m_f_LF(2);
          m_zmp_des_LF_local(1) =  m_f_LF(3) / m_f_LF(2);
          m_zmp_des_LF_local(2) = 0.0;
        }
        else
          m_zmp_des_LF_local.setZero();

        s = m_zmp_des_LF_local.head<2>();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp_des_right_foot,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des_right_foot before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);
        m_f_des_right_footSOUT(iter);
        pinocchio::SE3 H_rf = m_robot->position(m_invDyn->data(),
                                          m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
        if(fabs(m_f_RF(2)>1.0))
        {
          m_zmp_des_RF(0) = -m_f_RF(4) / m_f_RF(2);
          m_zmp_des_RF(1) =  m_f_RF(3) / m_f_RF(2);
          m_zmp_des_RF(2) = -H_rf.translation()(2);
        }
        else
          m_zmp_des_RF.setZero();

        m_zmp_des_RF = H_rf.act(m_zmp_des_RF);
        s = m_zmp_des_RF.head<2>();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp_des_left_foot,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des_left_foot before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);
        m_f_des_left_footSOUT(iter);
        pinocchio::SE3 H_lf = m_robot->position(m_invDyn->data(),
                                          m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
        if(fabs(m_f_LF(2)>1.0))
        {
          m_zmp_des_LF(0) = -m_f_LF(4) / m_f_LF(2);
          m_zmp_des_LF(1) =  m_f_LF(3) / m_f_LF(2);
          m_zmp_des_LF(2) = -H_lf.translation()(2);
        }
        else
          m_zmp_des_LF.setZero();

        m_zmp_des_LF = H_lf.act(m_zmp_des_LF);
        s = m_zmp_des_LF.head<2>();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp_des,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_des before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);
        m_zmp_des_left_footSOUT(iter);
        m_zmp_des_right_footSOUT(iter);

        m_zmp_des = (m_f_RF(2)*m_zmp_des_RF + m_f_LF(2)*m_zmp_des_LF) / (m_f_LF(2)+m_f_RF(2));
        s = m_zmp_des.head<2>();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp_ref,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_ref before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);
        const Vector6 & f_LF = m_f_ref_left_footSIN(iter);
        const Vector6 & f_RF = m_f_ref_right_footSIN(iter);

        pinocchio::SE3 H_lf = m_robot->position(m_invDyn->data(),
                                          m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
        Vector3 zmp_LF, zmp_RF;
        if(fabs(f_LF(2)>1.0))
        {
          zmp_LF(0) = -f_LF(4) / f_LF(2);
          zmp_LF(1) =  f_LF(3) / f_LF(2);
          zmp_LF(2) = -H_lf.translation()(2);
        }
        else
          zmp_LF.setZero();
        zmp_LF = H_lf.act(zmp_LF);

        pinocchio::SE3 H_rf = m_robot->position(m_invDyn->data(),
                                          m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
        if(fabs(f_RF(2)>1.0))
        {
          zmp_RF(0) = -f_RF(4) / f_RF(2);
          zmp_RF(1) =  f_RF(3) / f_RF(2);
          zmp_RF(2) = -H_rf.translation()(2);
        }
        else
          zmp_RF.setZero();
        zmp_RF = H_rf.act(zmp_RF);

        if(f_LF(2)+f_RF(2) != 0.0)
          s = (f_RF(2)*zmp_RF.head<2>() + f_LF(2)*zmp_LF.head<2>()) / (f_LF(2)+f_RF(2));

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp_right_foot,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_right_foot before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);
        const Vector6& f = m_wrench_right_footSIN(iter);
        assert(f.size()==6);
        pinocchio::SE3 H_rf = m_robot->position(m_invDyn->data(),
                                          m_robot->model().getJointId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
        if(fabs(f(2)>1.0))
        {
          m_zmp_RF(0) = -f(4) / f(2);
          m_zmp_RF(1) =  f(3) / f(2);
          m_zmp_RF(2) = -H_rf.translation()(2);
        }
        else
          m_zmp_RF.setZero();

        m_zmp_RF = H_rf.act(m_zmp_RF);
        s = m_zmp_RF.head<2>();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp_left_foot,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp_left_foot before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);
        const Vector6& f = m_wrench_left_footSIN(iter);
        pinocchio::SE3 H_lf = m_robot->position(m_invDyn->data(),
                                          m_robot->model().getJointId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
        if(fabs(f(2)>1.0))
        {
          m_zmp_LF(0) = -f(4) / f(2);
          m_zmp_LF(1) =  f(3) / f(2);
          m_zmp_LF(2) = -H_lf.translation()(2);
        }
        else
          m_zmp_LF.setZero();

        m_zmp_LF = H_lf.act(m_zmp_LF);
        s = m_zmp_LF.head<2>();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp before initialization!");
          return s;
        }
        if(s.size()!=2)
          s.resize(2);
        const Vector6& f_LF = m_wrench_left_footSIN(iter);
        const Vector6& f_RF = m_wrench_right_footSIN(iter);
        m_zmp_left_footSOUT(iter);
        m_zmp_right_footSOUT(iter);

        if(f_LF(2)+f_RF(2) > 1.0)
          m_zmp = (f_RF(2)*m_zmp_RF + f_LF(2)*m_zmp_LF) / (f_LF(2)+f_RF(2));
        s = m_zmp.head<2>();
        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(com,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com before initialization!");
          return s;
        }
        if(s.size()!=3)
          s.resize(3);
        const Vector3 & com = m_robot->com(m_invDyn->data());
        s = com + m_com_offset;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(com_vel,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com_vel before initialization!");
          return s;
        }
        if(s.size()!=3)
          s.resize(3);
        const Vector3 & com_vel = m_robot->com_vel(m_invDyn->data());
        s = com_vel;
        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(base_orientation,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal base_orientation before initialization!");
          return s;
        }
        /*
         * Code
         */
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(left_foot_pos, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal left_foot_pos before initialization!");
          return s;
        }
        m_tau_desSOUT(iter);
        pinocchio::SE3 oMi;
        s.resize(12);
        m_robot->framePosition(m_invDyn->data(), m_frame_id_lf, oMi);
	tsid::math::SE3ToVector(oMi, s);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(right_foot_pos, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal right_foot_pos before initialization!");
          return s;
        }
        m_tau_desSOUT(iter);
        pinocchio::SE3 oMi;
        s.resize(12);
        m_robot->framePosition(m_invDyn->data(), m_frame_id_rf, oMi);
	tsid::math::SE3ToVector(oMi, s);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(left_foot_vel, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal left_foot_vel before initialization!");
          return s;
        }
        pinocchio::Motion v;
        m_robot->frameVelocity(m_invDyn->data(), m_frame_id_lf, v);
        s = v.toVector();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(right_foot_vel, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal right_foot_vel before initialization!");
          return s;
        }
        pinocchio::Motion v;
        m_robot->frameVelocity(m_invDyn->data(), m_frame_id_rf, v);
        s = v.toVector();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(left_foot_acc, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal left_foot_acc before initialization!");
          return s;
        }
        m_tau_desSOUT(iter);
        if(m_contactState == RIGHT_SUPPORT)
          s = m_taskLF->getAcceleration(m_dv_urdf);
        else
          s = m_contactLF->getMotionTask().getAcceleration(m_dv_urdf);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(right_foot_acc, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal right_foot_acc before initialization!");
          return s;
        }
        m_tau_desSOUT(iter);
        if(m_contactState == LEFT_SUPPORT)
          s = m_taskRF->getAcceleration(m_dv_urdf);
        else
          s = m_contactRF->getMotionTask().getAcceleration(m_dv_urdf);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(left_foot_acc_des, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal left_foot_acc_des before initialization!");
          return s;
        }
        m_tau_desSOUT(iter);
        if(m_contactState == RIGHT_SUPPORT)
          s = m_taskLF->getDesiredAcceleration();
        else
          s = m_contactLF->getMotionTask().getDesiredAcceleration();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(right_foot_acc_des, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal right_foot_acc_des before initialization!");
          return s;
        }
        m_tau_desSOUT(iter);
        if(m_contactState == LEFT_SUPPORT)
          s = m_taskRF->getDesiredAcceleration();
        else
          s = m_contactRF->getMotionTask().getDesiredAcceleration();
        return s;
      }


      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void InverseDynamicsBalanceController::display(std::ostream& os) const
      {
        os << "InverseDynamicsBalanceController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
          getStatistics().report_all(1, os);
          os<<"QP size: nVar "<<m_invDyn->nVar()<<" nEq "<<m_invDyn->nEq()<<" nIn "<<m_invDyn->nIn()<<"\n";
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

