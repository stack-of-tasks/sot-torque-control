# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph import plug
from dynamic_graph.sot.core.switch import Switch
from dynamic_graph.sot.torque_control.force_torque_estimator import ForceTorqueEstimator
from dynamic_graph.sot.torque_control.numerical_difference import NumericalDifference
from dynamic_graph.sot.torque_control.joint_torque_controller import JointTorqueController
from dynamic_graph.sot.torque_control.joint_trajectory_generator import JointTrajectoryGenerator
from dynamic_graph.sot.torque_control.nd_trajectory_generator import NdTrajectoryGenerator
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.current_controller import CurrentController
from dynamic_graph.sot.torque_control.inverse_dynamics_controller import InverseDynamicsController
from dynamic_graph.sot.torque_control.admittance_controller import AdmittanceController
from dynamic_graph.sot.torque_control.position_controller import PositionController
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import NJ
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import *
from dynamic_graph.sot.torque_control.utils.sot_utils import Bunch
from dynamic_graph.sot.torque_control.utils.filter_utils import create_butter_lp_filter_Wn_05_N_3
#from dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains import *

def create_encoders(robot):
    from dynamic_graph.sot.core import Selec_of_vector
    encoders = Selec_of_vector('qn')
    plug(robot.device.robotState,     encoders.sin);
    encoders.selec(6,NJ+6);
    return encoders

def create_base_estimator(robot, dt, conf, robot_name="robot"):    
    from dynamic_graph.sot.torque_control.base_estimator import BaseEstimator
    base_estimator = BaseEstimator('base_estimator');
    plug(robot.encoders.sout,               base_estimator.joint_positions);
    #plug(robot.device.forceRLEG,            base_estimator.forceRLEG);
    #plug(robot.device.forceLLEG,            base_estimator.forceLLEG);
    plug(robot.filters.ft_LF_filter.x_filtered, base_estimator.forceLLEG)
    plug(robot.filters.ft_RF_filter.x_filtered, base_estimator.forceRLEG)
    plug(robot.filters.ft_LF_filter.dx,         base_estimator.dforceLLEG)
    plug(robot.filters.ft_RF_filter.dx,         base_estimator.dforceRLEG)
    plug(robot.filters.estimator_kin.dx,            base_estimator.joint_velocities);
    plug(robot.imu_filter.imu_quat,         base_estimator.imu_quaternion);
    #plug(robot.imu_offset_compensation.accelerometer_out, base_estimator.accelerometer);
    #plug(robot.imu_offset_compensation.gyrometer_out,     base_estimator.gyroscope);
    plug(robot.filters.gyro_filter.x_filtered,             base_estimator.gyroscope);
    plug(robot.filters.acc_filter.x_filtered,              base_estimator.accelerometer);
    base_estimator.K_fb_feet_poses.value = conf.K_fb_feet_poses;
    try:
        base_estimator.w_lf_in.value = conf.w_lf_in;
        base_estimator.w_rf_in.value = conf.w_rf_in;
    except:
        pass;
    
    base_estimator.set_imu_weight(conf.w_imu);
    base_estimator.set_stiffness_right_foot(conf.K);
    base_estimator.set_stiffness_left_foot(conf.K);
    base_estimator.set_zmp_std_dev_right_foot(conf.std_dev_zmp)
    base_estimator.set_zmp_std_dev_left_foot(conf.std_dev_zmp)
    base_estimator.set_normal_force_std_dev_right_foot(conf.std_dev_fz)
    base_estimator.set_normal_force_std_dev_left_foot(conf.std_dev_fz)
    base_estimator.set_zmp_margin_right_foot(conf.zmp_margin)
    base_estimator.set_zmp_margin_left_foot(conf.zmp_margin)
    base_estimator.set_normal_force_margin_right_foot(conf.normal_force_margin)
    base_estimator.set_normal_force_margin_left_foot(conf.normal_force_margin)
    base_estimator.set_right_foot_sizes(conf.RIGHT_FOOT_SIZES)
    base_estimator.set_left_foot_sizes(conf.LEFT_FOOT_SIZES)
    
    base_estimator.init(dt, robot_name);
    return base_estimator;
    
def create_imu_offset_compensation(robot, dt):
    from dynamic_graph.sot.torque_control.imu_offset_compensation import ImuOffsetCompensation
    imu_offset_compensation = ImuOffsetCompensation('imu_offset_comp');
    plug(robot.device.accelerometer, imu_offset_compensation.accelerometer_in);
    plug(robot.device.gyrometer,     imu_offset_compensation.gyrometer_in);
    imu_offset_compensation.init(dt);
    return imu_offset_compensation;

def create_imu_filter(robot, dt):
    from dynamic_graph.sot.torque_control.madgwickahrs import MadgwickAHRS
    imu_filter = MadgwickAHRS('imu_filter');
    imu_filter.init(dt);
    plug(robot.imu_offset_compensation.accelerometer_out, imu_filter.accelerometer);
    plug(robot.imu_offset_compensation.gyrometer_out,     imu_filter.gyroscope);
    return imu_filter;

def create_com_traj_gen(conf, dt):
    com_traj_gen = NdTrajectoryGenerator("com_traj_gen");
    com_traj_gen.initial_value.value = conf.COM_DES;
    com_traj_gen.init(dt,3);
    return com_traj_gen ;

def create_force_traj_gen(name, initial_value, dt):
    force_traj_gen = NdTrajectoryGenerator(name);
    force_traj_gen.initial_value.value = initial_value;
    force_traj_gen.init(dt,6);
    return force_traj_gen ;

def create_trajectory_switch():
    traj_sync = Switch("traj_sync");
    return traj_sync ;

def connect_synchronous_trajectories(switch, list_of_traj_gens):
  for traj_gen in list_of_traj_gens:
    plug(switch.out, traj_gen.trigger);

def create_free_flyer_locator(ent, robot_name="robot"):
    from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
    ff_locator = FreeFlyerLocator("ffLocator");
    plug(ent.device.robotState,             ff_locator.base6d_encoders);
    plug(ent.filters.estimator_kin.dx,              ff_locator.joint_velocities);
    try:
        plug(ff_locator.base6dFromFoot_encoders, ent.dynamic.position);
    except:
        print "[WARNING] Could not connect to dynamic entity, probably because you are in simulation"
        pass;
    ff_locator.init(robot_name);
    return ff_locator;
    
def create_flex_estimator(robot, dt=0.001):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce
    flex_est = HRP2ModelBaseFlexEstimatorIMUForce(robot, useMocap=False, dt=dt);
    flex_est.setOn(False);
    flex_est.interface.setExternalContactPresence(False);
    flex_est.interface.enabledContacts_lf_rf_lh_rh.value=(1,1,0,0);
    plug(robot.ff_locator.v, flex_est.leftFootVelocity.sin2);
    plug(robot.ff_locator.v, flex_est.rightFootVelocity.sin2);
    plug(robot.ff_locator.v, flex_est.inputVel.sin2);
    plug(robot.ff_locator.v, flex_est.DCom.sin2);
    return flex_est;
    
def create_floatingBase(robot):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import FromLocalToGLobalFrame 
    floatingBase = FromLocalToGLobalFrame(robot.flex_est, "FloatingBase")
    plug(robot.ff_locator.freeflyer_aa, floatingBase.sinPos);

    from dynamic_graph.sot.core import Selec_of_vector
    base_vel_no_flex = Selec_of_vector('base_vel_no_flex');
    plug(robot.ff_locator.v, base_vel_no_flex.sin);
    base_vel_no_flex.selec(0, 6);
    plug(base_vel_no_flex.sout,   floatingBase.sinVel);
    return floatingBase
    
def create_position_controller(robot, gains, dt=0.001, robot_name="robot"):
    posCtrl = PositionController('pos_ctrl')
    posCtrl.Kp.value = tuple(gains.kp_pos[round(dt,3)]);
    posCtrl.Kd.value = tuple(gains.kd_pos[round(dt,3)]);
    posCtrl.Ki.value = tuple(gains.ki_pos[round(dt,3)]);
    posCtrl.dqRef.value = NJ*(0.0,);
    plug(robot.device.robotState,             posCtrl.base6d_encoders);
    try:  # this works only in simulation
        plug(robot.device.jointsVelocities,    posCtrl.jointsVelocities);
    except:
        plug(robot.filters.estimator_kin.dx, posCtrl.jointsVelocities);
        pass;
    plug(posCtrl.pwmDes,                robot.device.control);
    try:
        plug(robot.traj_gen.q,       posCtrl.qRef);
    except:
        pass;
    posCtrl.init(dt, robot_name);
    return posCtrl;

def create_trajectory_generator(device, dt=0.001, robot_name="robot"):
    jtg = JointTrajectoryGenerator("jtg");
    plug(device.robotState,             jtg.base6d_encoders);
    jtg.init(dt, robot_name);
    return jtg;

def create_estimators(robot, conf, motor_params, dt):
    from dynamic_graph.sot.torque_control.utils.filter_utils import create_chebi1_checby2_series_filter
    robot.filters_sg = Bunch()
    filters = Bunch();

    estimator_ft = ForceTorqueEstimator("estimator_ft");

    # create low-pass filter for motor currents
    filters.current_filter = create_butter_lp_filter_Wn_05_N_3('current_filter', dt, NJ)

    use_sav_gol = True;
    if(use_sav_gol):
        #filters.current_filter = NumericalDifference("current_filter");
        robot.filters_sg.ft_RF_filter = NumericalDifference("ft_RF_sg_filter");
        robot.filters_sg.ft_LF_filter = NumericalDifference("ft_LF_sg_filter");
        robot.filters_sg.ft_RH_filter = NumericalDifference("ft_RH_sg_filter");
        robot.filters_sg.ft_LH_filter = NumericalDifference("ft_LH_sg_filter");
        robot.filters_sg.acc_filter = NumericalDifference("dv_sg_filter");
        robot.filters_sg.gyro_filter = NumericalDifference("w_sg_filter");
        robot.filters_sg.estimator_kin = NumericalDifference("estimator_kin_sg");
    
        #robot.filters_sg.current_filter.init(dt,NJ, conf.DELAY_CURRENT*dt,1)
        robot.filters_sg.ft_RF_filter.init(dt, 6, conf.DELAY_FORCE*dt,1)
        robot.filters_sg.ft_LF_filter.init(dt, 6, conf.DELAY_FORCE*dt,1)
        robot.filters_sg.ft_RH_filter.init(dt, 6, conf.DELAY_FORCE*dt,1)
        robot.filters_sg.ft_LH_filter.init(dt, 6, conf.DELAY_FORCE*dt,1)
        robot.filters_sg.gyro_filter.init(dt, 3, conf.DELAY_GYRO*dt,1)
        robot.filters_sg.acc_filter.init(dt, 3, conf.DELAY_ACC*dt,1)
        robot.filters_sg.estimator_kin.init(dt,NJ, conf.DELAY_ENC*dt,2);

        plug(robot.encoders.sout,                             robot.filters_sg.estimator_kin.x);
        plug(robot.imu_offset_compensation.accelerometer_out, robot.filters_sg.acc_filter.x);
        plug(robot.imu_offset_compensation.gyrometer_out,     robot.filters_sg.gyro_filter.x);
        plug(robot.device.forceRLEG,                          robot.filters_sg.ft_RF_filter.x);
        plug(robot.device.forceLLEG,                          robot.filters_sg.ft_LF_filter.x);
        plug(robot.device.forceRARM,                          robot.filters_sg.ft_RH_filter.x);
        plug(robot.device.forceLARM,                          robot.filters_sg.ft_LH_filter.x);

    filters.ft_RF_filter  = create_chebi1_checby2_series_filter("ft_RF_filter", dt, 6);
    filters.ft_LF_filter  = create_chebi1_checby2_series_filter("ft_LF_filter", dt, 6);
    filters.ft_RH_filter  = create_chebi1_checby2_series_filter("ft_RH_filter", dt, 6);
    filters.ft_LH_filter  = create_chebi1_checby2_series_filter("ft_LH_filter", dt, 6);
    filters.acc_filter    = create_chebi1_checby2_series_filter("dv_filter", dt, 3);
    filters.gyro_filter   = create_chebi1_checby2_series_filter("w_filter", dt, 3);
    filters.estimator_kin = create_chebi1_checby2_series_filter("estimator_kin", dt, NJ);

    plug(robot.encoders.sout,                             filters.estimator_kin.x);
    plug(robot.device.robotState,                         estimator_ft.base6d_encoders);

    plug(robot.imu_offset_compensation.accelerometer_out, filters.acc_filter.x);
    plug(robot.imu_offset_compensation.gyrometer_out,     filters.gyro_filter.x);
    plug(robot.device.forceRLEG,                          filters.ft_RF_filter.x);
    plug(robot.device.forceLLEG,                          filters.ft_LF_filter.x);
    plug(robot.device.forceRARM,                          filters.ft_RH_filter.x);
    plug(robot.device.forceLARM,                          filters.ft_LH_filter.x);
    plug(robot.device.currents,                           filters.current_filter.x);

    plug(filters.acc_filter.x_filtered,                   estimator_ft.accelerometer);
    plug(filters.gyro_filter.x_filtered,                  estimator_ft.gyro);
    plug(filters.gyro_filter.dx,                          estimator_ft.dgyro);
    plug(filters.ft_RF_filter.x_filtered,                 estimator_ft.ftSensRightFoot);
    plug(filters.ft_LF_filter.x_filtered,                 estimator_ft.ftSensLeftFoot);
    plug(filters.ft_RH_filter.x_filtered,                 estimator_ft.ftSensRightHand);
    plug(filters.ft_LH_filter.x_filtered,                 estimator_ft.ftSensLeftHand);
    plug(filters.current_filter.x_filtered,               estimator_ft.current);

    plug(filters.estimator_kin.x_filtered, estimator_ft.q_filtered);
    plug(filters.estimator_kin.dx,         estimator_ft.dq_filtered);
    plug(filters.estimator_kin.ddx,        estimator_ft.ddq_filtered);
    try:
        plug(robot.traj_gen.dq,       estimator_ft.dqRef);
        plug(robot.traj_gen.ddq,      estimator_ft.ddqRef);
    except:
        pass;
    estimator_ft.wCurrentTrust.value     = tuple(NJ*[conf.CURRENT_TORQUE_ESTIMATION_TRUST,])
    estimator_ft.saturationCurrent.value = tuple(NJ*[conf.SATURATION_CURRENT,])

    estimator_ft.motorParameterKt_p.value  = tuple(motor_params.Kt_p)
    estimator_ft.motorParameterKt_n.value  = tuple(motor_params.Kt_n)
    estimator_ft.motorParameterKf_p.value  = tuple(motor_params.Kf_p)
    estimator_ft.motorParameterKf_n.value  = tuple(motor_params.Kf_n)
    estimator_ft.motorParameterKv_p.value  = tuple(motor_params.Kv_p)
    estimator_ft.motorParameterKv_n.value  = tuple(motor_params.Kv_n)
    estimator_ft.motorParameterKa_p.value  = tuple(motor_params.Ka_p)
    estimator_ft.motorParameterKa_n.value  = tuple(motor_params.Ka_n)

    estimator_ft.rotor_inertias.value = motor_params.ROTOR_INERTIAS;
    estimator_ft.gear_ratios.value    = motor_params.GEAR_RATIOS;

    estimator_ft.init(True);

    return (estimator_ft, filters);
        
def create_torque_controller(robot, conf, motor_params, dt=0.001, robot_name="robot"):
    torque_ctrl = JointTorqueController("jtc");
    plug(robot.filters.estimator_kin.dx,                torque_ctrl.jointsVelocities);
    plug(robot.filters.estimator_kin.ddx,               torque_ctrl.jointsAccelerations);
    plug(robot.estimator_ft.jointsTorques,              torque_ctrl.jointsTorques);
    torque_ctrl.jointsTorquesDesired.value              = NJ*(0.0,);
    torque_ctrl.jointsTorquesDerivative.value           = NJ*(0.0,);
    torque_ctrl.dq_des.value                            = NJ*(0.0,);
    torque_ctrl.KpTorque.value                          = tuple(conf.k_p_torque);
    torque_ctrl.KdTorque.value                          = tuple(conf.k_d_torque);
    torque_ctrl.KiTorque.value                          = tuple(conf.k_i_torque);
    torque_ctrl.KdVel.value                             = tuple(conf.k_d_vel);
    torque_ctrl.torque_integral_saturation.value        = tuple(conf.torque_integral_saturation);
    torque_ctrl.coulomb_friction_compensation_percentage.value = NJ*(conf.COULOMB_FRICTION_COMPENSATION_PERCENTAGE,);

    torque_ctrl.motorParameterKt_p.value  = tuple(motor_params.Kt_p)
    torque_ctrl.motorParameterKt_n.value  = tuple(motor_params.Kt_n)
    torque_ctrl.motorParameterKf_p.value  = tuple(motor_params.Kf_p)
    torque_ctrl.motorParameterKf_n.value  = tuple(motor_params.Kf_n)
    torque_ctrl.motorParameterKv_p.value  = tuple(motor_params.Kv_p)
    torque_ctrl.motorParameterKv_n.value  = tuple(motor_params.Kv_n)
    torque_ctrl.motorParameterKa_p.value  = tuple(motor_params.Ka_p)
    torque_ctrl.motorParameterKa_n.value  = tuple(motor_params.Ka_n)
    torque_ctrl.polySignDq.value          = NJ*(conf.poly_sign_dq,); 
    torque_ctrl.init(dt, robot_name);
    return torque_ctrl;
   
def create_balance_controller(robot, conf, motor_params, dt, robot_name='robot'):
    from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");

    try:
        plug(robot.base_estimator.q, ctrl.q);
        plug(robot.base_estimator.v, ctrl.v);
    except:        
        plug(robot.ff_locator.base6dFromFoot_encoders, ctrl.q);
        plug(robot.ff_locator.v, ctrl.v);
        
    try:
        from dynamic_graph.sot.core import Selec_of_vector
        robot.ddq_des = Selec_of_vector('ddq_des')
        plug(ctrl.dv_des, robot.ddq_des.sin);
        robot.ddq_des.selec(6,NJ+6);
        plug(robot.ddq_des.sout, robot.estimator_ft.ddqRef);
    except:
        print "WARNING: Could not connect dv_des from BalanceController to ForceTorqueEstimator";

    plug(robot.estimator_ft.contactWrenchRightSole, ctrl.wrench_right_foot);
    plug(robot.estimator_ft.contactWrenchLeftSole,  ctrl.wrench_left_foot);
    plug(ctrl.tau_des,                              robot.torque_ctrl.jointsTorquesDesired);
    plug(ctrl.dq_admittance,                        robot.torque_ctrl.dq_des);
    plug(ctrl.tau_des,                              robot.estimator_ft.tauDes);

    plug(ctrl.right_foot_pos,         robot.rf_traj_gen.initial_value);
    plug(robot.rf_traj_gen.x,         ctrl.rf_ref_pos);
    plug(robot.rf_traj_gen.dx,        ctrl.rf_ref_vel);
    plug(robot.rf_traj_gen.ddx,       ctrl.rf_ref_acc);

    plug(ctrl.left_foot_pos,          robot.lf_traj_gen.initial_value);
    plug(robot.lf_traj_gen.x,         ctrl.lf_ref_pos);
    plug(robot.lf_traj_gen.dx,        ctrl.lf_ref_vel);
    plug(robot.lf_traj_gen.ddx,       ctrl.lf_ref_acc);
    
    plug(robot.traj_gen.q,                        ctrl.posture_ref_pos);
    plug(robot.traj_gen.dq,                       ctrl.posture_ref_vel);
    plug(robot.traj_gen.ddq,                      ctrl.posture_ref_acc);
    plug(robot.com_traj_gen.x,                    ctrl.com_ref_pos);
    plug(robot.com_traj_gen.dx,                   ctrl.com_ref_vel);
    plug(robot.com_traj_gen.ddx,                  ctrl.com_ref_acc);

    plug(robot.rf_force_traj_gen.x,               ctrl.f_ref_right_foot);
    plug(robot.lf_force_traj_gen.x,               ctrl.f_ref_left_foot);

    # rather than giving to the controller the values of gear ratios and rotor inertias
    # it is better to compute directly their product in python and pass the result
    # to the C++ entity, because otherwise we get a loss of precision
#    ctrl.rotor_inertias.value = conf.ROTOR_INERTIAS;
#    ctrl.gear_ratios.value = conf.GEAR_RATIOS;
    ctrl.rotor_inertias.value = tuple([g*g*r for (g,r) in
                                       zip(motor_params.GEAR_RATIOS, motor_params.ROTOR_INERTIAS)])
    ctrl.gear_ratios.value = NJ*(1.0,);
    ctrl.contact_normal.value = conf.FOOT_CONTACT_NORMAL;
    ctrl.contact_points.value = conf.RIGHT_FOOT_CONTACT_POINTS;
    ctrl.f_min.value = conf.fMin;
    ctrl.f_max_right_foot.value = conf.fMax;
    ctrl.f_max_left_foot.value =  conf.fMax;
    ctrl.mu.value = conf.mu[0];
    ctrl.weight_contact_forces.value = (1e2, 1e2, 1e0, 1e3, 1e3, 1e3);
    ctrl.kp_com.value = 3*(conf.kp_com,);
    ctrl.kd_com.value = 3*(conf.kd_com,);
    ctrl.kp_constraints.value = 6*(conf.kp_constr,);
    ctrl.kd_constraints.value = 6*(conf.kd_constr,);
    ctrl.kp_feet.value = 6*(conf.kp_feet,);
    ctrl.kd_feet.value = 6*(conf.kd_feet,);
    ctrl.kp_posture.value = conf.kp_posture;
    ctrl.kd_posture.value = conf.kd_posture;
    ctrl.kp_pos.value = conf.kp_pos;
    ctrl.kd_pos.value = conf.kd_pos;
    ctrl.kp_admittance.value = conf.kp_admittance;
    ctrl.ki_admittance.value = conf.ki_admittance;

    ctrl.w_com.value = conf.w_com;
    ctrl.w_feet.value = conf.w_feet;
    ctrl.w_forces.value = conf.w_forces;
    ctrl.w_posture.value = conf.w_posture;
    ctrl.w_base_orientation.value = conf.w_base_orientation;
    ctrl.w_torques.value = conf.w_torques;
    
    ctrl.init(dt, robot_name);
    
    return ctrl;
    
def create_inverse_dynamics(robot, conf, motor_params, dt=0.001):
    inv_dyn_ctrl = InverseDynamicsController("inv_dyn");
    plug(robot.device.robotState,             inv_dyn_ctrl.base6d_encoders);
    plug(robot.filters.estimator_kin.dx,              inv_dyn_ctrl.jointsVelocities);
    plug(robot.traj_gen.q,                    inv_dyn_ctrl.qRef);
    plug(robot.traj_gen.dq,                   inv_dyn_ctrl.dqRef);
    plug(robot.traj_gen.ddq,                  inv_dyn_ctrl.ddqRef);
    plug(robot.estimator_ft.contactWrenchRightSole,   inv_dyn_ctrl.fRightFoot);
    plug(robot.estimator_ft.contactWrenchLeftSole,    inv_dyn_ctrl.fLeftFoot);
    plug(robot.estimator_ft.contactWrenchRightHand,   inv_dyn_ctrl.fRightHand);
    plug(robot.estimator_ft.contactWrenchLeftHand,    inv_dyn_ctrl.fLeftHand);
    plug(robot.traj_gen.fRightFoot,           inv_dyn_ctrl.fRightFootRef);
    plug(robot.traj_gen.fLeftFoot,            inv_dyn_ctrl.fLeftFootRef);
    plug(robot.traj_gen.fRightHand,           inv_dyn_ctrl.fRightHandRef);
    plug(robot.traj_gen.fLeftHand,            inv_dyn_ctrl.fLeftHandRef);
    plug(robot.estimator_ft.baseAngularVelocity, inv_dyn_ctrl.baseAngularVelocity);
    plug(robot.estimator_ft.baseAcceleration,    inv_dyn_ctrl.baseAcceleration);
    plug(inv_dyn_ctrl.tauDes,           robot.torque_ctrl.jointsTorquesDesired);
    plug(inv_dyn_ctrl.tauDes,           robot.estimator_ft.tauDes);
    plug(robot.estimator_ft.dynamicsError,       inv_dyn_ctrl.dynamicsError);
    
    inv_dyn_ctrl.dynamicsErrorGain.value = (NJ+6)*(0.0,);
    inv_dyn_ctrl.Kp.value = tuple(conf.k_s); # joint proportional conf
    inv_dyn_ctrl.Kd.value = tuple(conf.k_d); # joint derivative conf
    inv_dyn_ctrl.Kf.value = tuple(conf.k_f); # force proportional conf
    inv_dyn_ctrl.Ki.value = tuple(conf.k_i); # force integral conf
    inv_dyn_ctrl.rotor_inertias.value = motor_params.ROTOR_INERTIAS;
    inv_dyn_ctrl.gear_ratios.value    = motor_params.GEAR_RATIOS;
    inv_dyn_ctrl.controlledJoints.value = NJ*(1.0,);
    inv_dyn_ctrl.init(dt);
    return inv_dyn_ctrl;
        
def create_ctrl_manager(conf, motor_params, dt, robot_name='robot'):
    ctrl_manager = ControlManager("ctrl_man");        

    ctrl_manager.tau_predicted.value    = NJ*(0.0,);
    ctrl_manager.i_measured.value       = NJ*(0.0,);
    ctrl_manager.tau_max.value          = NJ*(conf.TAU_MAX,);
    ctrl_manager.i_max.value            = NJ*(conf.CURRENT_MAX,);
    ctrl_manager.u_max.value            = NJ*(conf.CTRL_MAX,);
    
    # Init should be called before addCtrlMode 
    # because the size of state vector must be known.
    ctrl_manager.init(dt, conf.urdfFileName, robot_name)

    # Set the map from joint name to joint ID
    for key in conf.mapJointNameToID:
      ctrl_manager.setNameToId(key,conf.mapJointNameToID[key])
            
    # Set the map joint limits for each id
    for key in conf.mapJointLimits:
      ctrl_manager.setJointLimitsFromId(key,conf.mapJointLimits[key][0], \
                              conf.mapJointLimits[key][1])
          
    # Set the force limits for each id
    for key in conf.mapForceIdToForceLimits:
      ctrl_manager.setForceLimitsFromId(key,tuple(conf.mapForceIdToForceLimits[key][0]), \
                              tuple(conf.mapForceIdToForceLimits[key][1]))

    # Set the force sensor id for each sensor name
    for key in conf.mapNameToForceId:
      ctrl_manager.setForceNameToForceId(key,conf.mapNameToForceId[key])

    # Set the map from the urdf joint list to the sot joint list
    ctrl_manager.setJointsUrdfToSot(conf.urdftosot)

    # Set the foot frame name
    for key in conf.footFrameNames:
      ctrl_manager.setFootFrameName(key,conf.footFrameNames[key])
    
    # Set IMU hosting joint name
    ctrl_manager.setImuJointName(conf.ImuJointName)
    
    ctrl_manager.setRightFootForceSensorXYZ(conf.rightFootSensorXYZ);
    ctrl_manager.setRightFootSoleXYZ(conf.rightFootSoleXYZ);

    return ctrl_manager;

def connect_ctrl_manager(robot):    
    # connect to device    
    plug(robot.device.currents,               robot.ctrl_manager.i_measured);
    plug(robot.estimator_ft.jointsTorques,    robot.ctrl_manager.tau);
    robot.ctrl_manager.addCtrlMode("pos");
    robot.ctrl_manager.addCtrlMode("torque");    
    plug(robot.torque_ctrl.u,                           robot.ctrl_manager.ctrl_torque);
    plug(robot.pos_ctrl.pwmDes,                         robot.ctrl_manager.ctrl_pos);
    plug(robot.ctrl_manager.joints_ctrl_mode_torque,    robot.inv_dyn.active_joints);
    robot.ctrl_manager.setCtrlMode("all", "pos");
    plug(robot.ctrl_manager.u_safe,                     robot.current_ctrl.i_des);
    return;
    
def create_current_controller(robot, conf, motor_params, dt, robot_name='robot'):
    current_ctrl = CurrentController("current_ctrl");        

    current_ctrl.i_max.value                                = NJ*(conf.CURRENT_MAX,);
    current_ctrl.u_max.value                                = NJ*(conf.CTRL_MAX,);
    current_ctrl.u_saturation.value                         = NJ*(conf.CTRL_SATURATION,);
    current_ctrl.percentage_dead_zone_compensation.value    = tuple(conf.percentage_dead_zone_compensation);
    current_ctrl.percentage_bemf_compensation.value         = tuple(conf.percentage_bemf_compensation);
    current_ctrl.i_sensor_offsets_low_level.value           = tuple(conf.current_sensor_offsets_low_level);
    current_ctrl.i_max_dead_zone_compensation.value         = tuple(conf.i_max_dz_comp);
    current_ctrl.in_out_gain.value                          = NJ*(conf.IN_OUT_GAIN,);
    current_ctrl.kp_current.value                           = tuple(conf.kp_current);
    current_ctrl.ki_current.value                           = tuple(conf.ki_current);
    current_ctrl.bemf_factor.value                          = motor_params.K_bemf;
    current_ctrl.dead_zone_offsets.value                    = motor_params.deadzone;
    current_ctrl.i_sens_gains.value                         = motor_params.cur_sens_gains;
    # connect to other entities
    plug(robot.filters.current_filter.x_filtered,   current_ctrl.i_measured)
    plug(robot.filters.estimator_kin.dx,            current_ctrl.dq);
    plug(current_ctrl.u_safe,                       robot.device.control);
    # initialize    
    current_ctrl.init(dt, robot_name, conf.CURRENT_OFFSET_ITERS)
    
    return current_ctrl;


def create_admittance_ctrl(robot, dt=0.001):
    admit_ctrl = AdmittanceController("adm_ctrl");
    plug(robot.device.robotState,             admit_ctrl.base6d_encoders);
    plug(robot.filters.estimator_kin.dx,    admit_ctrl.jointsVelocities);
    plug(robot.estimator_ft.contactWrenchRightSole,   admit_ctrl.fRightFoot);
    plug(robot.estimator_ft.contactWrenchLeftSole,    admit_ctrl.fLeftFoot);
    plug(robot.estimator_ft.contactWrenchRightHand,   admit_ctrl.fRightHand);
    plug(robot.estimator_ft.contactWrenchLeftHand,    admit_ctrl.fLeftHand);
    plug(robot.traj_gen.fRightFoot,           admit_ctrl.fRightFootRef);
    plug(robot.traj_gen.fLeftFoot,            admit_ctrl.fLeftFootRef);
    plug(robot.traj_gen.fRightHand,           admit_ctrl.fRightHandRef);
    plug(robot.traj_gen.fLeftHand,            admit_ctrl.fLeftHandRef);
    
    admit_ctrl.damping.value = 4*(0.05,);
    admit_ctrl.Kd.value = NJ*(0,);
    kf = -0.0005;
    km = -0.008;
    admit_ctrl.Kf.value = 3*(kf,)+3*(km,)+3*(kf,)+3*(km,)+3*(kf,)+3*(km,)+3*(kf,)+3*(km,);
    
    robot.ctrl_manager.addCtrlMode("adm");
    plug(admit_ctrl.qDes,                       robot.ctrl_manager.ctrl_adm);
    plug(robot.ctrl_manager.joints_ctrl_mode_adm, admit_ctrl.controlledJoints);
    
    admit_ctrl.init(dt);
    return admit_ctrl;

def create_topic(ros_import, signal, name, robot=None, entity=None, data_type='vector', sleep_time=0.1):
    ros_import.add(data_type, name+'_ros', name);
    plug(signal, ros_import.signal(name+'_ros'));
    if(entity is not None and robot is not None):
        robot.device.before.addDownsampledSignal(entity.name+'.'+signal.name.split('::')[-1], 1);
    from time import sleep
    sleep(sleep_time);
    

def create_ros_topics(robot):
    from dynamic_graph.ros import RosPublish
    ros = RosPublish('rosPublish');
    try:
        create_topic(ros, robot.device.robotState,      'robotState');
        create_topic(ros, robot.device.gyrometer,       'gyrometer');
        create_topic(ros, robot.device.accelerometer,   'accelerometer');
        create_topic(ros, robot.device.forceRLEG,       'forceRLEG');
        create_topic(ros, robot.device.forceLLEG,       'forceLLEG');
        create_topic(ros, robot.device.currents,        'currents');
#        create_topic(ros, robot.device.forceRARM,       'forceRARM');
#        create_topic(ros, robot.device.forceLARM,       'forceLARM');
        robot.device.after.addDownsampledSignal('rosPublish.trigger',1);
    except:
        pass;
    
    try:
        create_topic(ros, robot.filters.estimator_kin.dx,                            'jointsVelocities');
        create_topic(ros, robot.estimator_ft.contactWrenchLeftSole,          'contactWrenchLeftSole');
        create_topic(ros, robot.estimator_ft.contactWrenchRightSole,         'contactWrenchRightSole');
        create_topic(ros, robot.estimator_ft.jointsTorques,                  'jointsTorques');
#        create_topic(ros, robot.estimator.jointsTorquesFromInertiaModel,  'jointsTorquesFromInertiaModel');
#        create_topic(ros, robot.estimator.jointsTorquesFromMotorModel,    'jointsTorquesFromMotorModel');
#        create_topic(ros, robot.estimator.currentFiltered,                'currentFiltered');
    except:
        pass;

    try:
        create_topic(ros, robot.torque_ctrl.u, 'i_des_torque_ctrl');
    except:
        pass;

    try:
        create_topic(ros, robot.traj_gen.q,   'q_ref');
#        create_topic(ros, robot.traj_gen.dq,  'dq_ref');
#        create_topic(ros, robot.traj_gen.ddq, 'ddq_ref');
    except:
        pass;

    try:
        create_topic(ros, robot.ctrl_manager.pwmDes,                  'i_des');
        create_topic(ros, robot.ctrl_manager.pwmDesSafe,              'i_des_safe');
#        create_topic(ros, robot.ctrl_manager.signOfControlFiltered,   'signOfControlFiltered');
#        create_topic(ros, robot.ctrl_manager.signOfControl,           'signOfControl');
    except:
        pass;

    try:
        create_topic(ros, robot.inv_dyn.tau_des, 'tau_des');
    except:
        pass;

    try:
        create_topic(ros, robot.ff_locator.base6dFromFoot_encoders,        'base6dFromFoot_encoders');
    except:
        pass;

    try:
        create_topic(ros, robot.floatingBase.soutPos, 'floatingBase_pos');
    except:
        pass;
    
    return ros;
    
    
def addTrace(tracer, entity, signalName):
    """
    Add a signal to a tracer
    """
    signal = '{0}.{1}'.format(entity.name, signalName);
    filename = '{0}-{1}'.format(entity.name, signalName);
    tracer.add(signal, filename);
    
def addSignalsToTracer(tracer, device):
    addTrace(tracer,device,'robotState');
    addTrace(tracer,device,'gyrometer');
    addTrace(tracer,device,'accelerometer');
    addTrace(tracer,device,'forceRLEG');
    addTrace(tracer,device,'forceLLEG');
    addTrace(tracer,device,'forceRARM');
    addTrace(tracer,device,'forceLARM');
    addTrace(tracer,device,'control');
    addTrace(tracer,device,'currents');


def create_tracer(device, traj_gen=None, estimator_ft=None, estimator_kin=None,
                  inv_dyn=None, torque_ctrl=None):
    tracer = TracerRealTime('motor_id_trace');
    tracer.setBufferSize(80*(2**20));
    tracer.open('/tmp/','dg_','.dat');
    device.after.addSignal('{0}.triger'.format(tracer.name));

    addSignalsToTracer(tracer, device);
        
    with open('/tmp/dg_info.dat', 'a') as f:
        if(estimator_ft!=None):
            #f.write('Estimator F/T sensors delay: {0}\n'.format(robot.estimator_ft.getDelayFTsens()));
            f.write('Estimator use reference velocities: {0}\n'.format(robot.estimator_ft.getUseRefJointVel()));
            f.write('Estimator use reference accelerations: {0}\n'.format(robot.estimator_ft.getUseRefJointAcc()));
            #f.write('Estimator accelerometer delay: {0}\n'.format(robot.estimator_ft.getDelayAcc()));
            #f.write('Estimator gyroscope delay: {0}\n'.format(robot.estimator_ft.getDelayGyro()));
            f.write('Estimator use raw encoders: {0}\n'.format(robot.estimator_ft.getUseRawEncoders()));
            f.write('Estimator use f/t sensors: {0}\n'.format(robot.estimator_ft.getUseFTsensors()));
            f.write('Estimator f/t sensor offsets: {0}\n'.format(robot.estimator_ft.getFTsensorOffsets()));
        if(estimator_kin!=None):
            f.write('Estimator encoder delay: {0}\n'.format(robot.filters.estimator_kin.getDelay()));
        if(inv_dyn!=None):
            f.write('Inv dyn Ks: {0}\n'.format(inv_dyn.Kp.value));
            f.write('Inv dyn Kd: {0}\n'.format(inv_dyn.Kd.value));
            f.write('Inv dyn Kf: {0}\n'.format(inv_dyn.Kf.value));
            f.write('Inv dyn Ki: {0}\n'.format(inv_dyn.Ki.value));
        if(torque_ctrl!=None):
            f.write('Torque ctrl KpTorque: {0}\n'.format (robot.torque_ctrl.KpTorque.value ));
    f.close();
    return tracer;

def reset_tracer(device,tracer):
    from time import sleep
    tracer.stop();
    sleep(0.2);
    tracer.dump();
    sleep(0.2);
    tracer.close();
    sleep(0.2);
    tracer.clear();
    sleep(0.2);
    tracer = create_tracer(device);
    return tracer;
