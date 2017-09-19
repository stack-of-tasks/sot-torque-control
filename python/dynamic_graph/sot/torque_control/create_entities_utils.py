# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph import plug
from dynamic_graph.sot.torque_control.force_torque_estimator import ForceTorqueEstimator
from dynamic_graph.sot.torque_control.numerical_difference import NumericalDifference as VelAccEstimator
from dynamic_graph.sot.torque_control.joint_torque_controller import JointTorqueController
from dynamic_graph.sot.torque_control.joint_trajectory_generator import JointTrajectoryGenerator
from dynamic_graph.sot.torque_control.nd_trajectory_generator import NdTrajectoryGenerator
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.inverse_dynamics_controller import InverseDynamicsController
from dynamic_graph.sot.torque_control.admittance_controller import AdmittanceController
from dynamic_graph.sot.torque_control.position_controller import PositionController
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import NJ
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import *
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
    plug(robot.device.forceRLEG,            base_estimator.forceRLEG);
    plug(robot.device.forceLLEG,            base_estimator.forceLLEG);
    plug(robot.estimator_kin.dx,            base_estimator.joint_velocities);
    plug(robot.imu_filter.imu_quat,         base_estimator.imu_quaternion);
    plug(robot.imu_offset_compensation.accelerometer_out, base_estimator.accelerometer);
    
    base_estimator.K_fb_feet_poses.value = conf.K_fb_feet_poses;
    
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

def create_free_flyer_locator(ent, robot_name="robot"):
    from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
    ff_locator = FreeFlyerLocator("ffLocator");
    plug(ent.device.robotState,             ff_locator.base6d_encoders);
    plug(ent.estimator_kin.dx,              ff_locator.joint_velocities);
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
        plug(robot.estimator_kin.dx, posCtrl.jointsVelocities);
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
    estimator_kin = VelAccEstimator("estimator_kin");
    estimator_ft = ForceTorqueEstimator("estimator_ft");

    plug(robot.encoders.sout,                             estimator_kin.x);
    plug(robot.device.robotState,                         estimator_ft.base6d_encoders);
    plug(robot.imu_offset_compensation.accelerometer_out, estimator_ft.accelerometer);
    plug(robot.imu_offset_compensation.gyrometer_out,     estimator_ft.gyroscope);
    plug(robot.device.forceRLEG,                          estimator_ft.ftSensRightFoot);
    plug(robot.device.forceLLEG,                          estimator_ft.ftSensLeftFoot);
    plug(robot.device.forceRARM,                          estimator_ft.ftSensRightHand);
    plug(robot.device.forceLARM,                          estimator_ft.ftSensLeftHand);
    plug(robot.device.currents,                           estimator_ft.currentMeasure);

    plug(estimator_kin.x_filtered, estimator_ft.q_filtered);
    plug(estimator_kin.dx,         estimator_ft.dq_filtered);
    plug(estimator_kin.ddx,        estimator_ft.ddq_filtered);
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

    estimator_ft.init(dt, conf.DELAY_ACC*dt, conf.DELAY_GYRO*dt, conf.DELAY_FORCE*dt, conf.DELAY_CURRENT*dt, True);
    estimator_kin.init(dt,NJ, conf.DELAY_ENC*dt);
    
    return (estimator_ft, estimator_kin);
        
def create_torque_controller(robot, conf, motor_params, dt=0.001, robot_name="robot"):
    torque_ctrl = JointTorqueController("jtc");
    plug(robot.device.robotState,               torque_ctrl.base6d_encoders);
    plug(robot.estimator_kin.dx,                torque_ctrl.jointsVelocities);
    plug(robot.estimator_kin.ddx,               torque_ctrl.jointsAccelerations);
    plug(robot.estimator_ft.jointsTorques,      torque_ctrl.jointsTorques);
    plug(robot.estimator_ft.currentFiltered,    torque_ctrl.measuredCurrent);
    torque_ctrl.jointsTorquesDesired.value              = NJ*(0.0,);
    torque_ctrl.KpTorque.value                          = tuple(conf.k_p_torque);
    torque_ctrl.KiTorque.value                          = NJ*(0.0,);
    torque_ctrl.KpCurrent.value                         = tuple(conf.k_p_current);
    torque_ctrl.KiCurrent.value                         = NJ*(0.0,);
    torque_ctrl.k_tau.value                             = tuple(conf.k_tau);
    torque_ctrl.k_v.value                               = tuple(conf.k_v);
    torque_ctrl.frictionCompensationPercentage.value    = NJ*(conf.FRICTION_COMPENSATION_PERCENTAGE,);

    torque_ctrl.motorParameterKt_p.value  = tuple(motor_params.Kt_p)
    torque_ctrl.motorParameterKt_n.value  = tuple(motor_params.Kt_n)
    torque_ctrl.motorParameterKf_p.value  = tuple(motor_params.Kf_p)
    torque_ctrl.motorParameterKf_n.value  = tuple(motor_params.Kf_n)
    torque_ctrl.motorParameterKv_p.value  = tuple(motor_params.Kv_p)
    torque_ctrl.motorParameterKv_n.value  = tuple(motor_params.Kv_n)
    torque_ctrl.motorParameterKa_p.value  = tuple(motor_params.Ka_p)
    torque_ctrl.motorParameterKa_n.value  = tuple(motor_params.Ka_n)
    torque_ctrl.polySignDq.value          = NJ*(3,); 
    torque_ctrl.init(dt, robot_name);
    return torque_ctrl;
   
def create_balance_controller(robot, conf, dt, robot_name='robot'):
    from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");

    try:
        plug(robot.base_estimator.q, ctrl.q);
        plug(robot.base_estimator.v, ctrl.v);
    except:        
        plug(robot.ff_locator.base6dFromFoot_encoders, ctrl.q);
        plug(robot.ff_locator.v, ctrl.v);

    plug(robot.estimator_ft.contactWrenchRightSole, ctrl.wrench_right_foot);
    plug(robot.estimator_ft.contactWrenchLeftSole,  ctrl.wrench_left_foot);
    plug(ctrl.tau_des,                              robot.torque_ctrl.jointsTorquesDesired);
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

    # rather than giving to the controller the values of gear ratios and rotor inertias
    # it is better to compute directly their product in python and pass the result
    # to the C++ entity, because otherwise we get a loss of precision
#    ctrl.rotor_inertias.value = conf.ROTOR_INERTIAS;
#    ctrl.gear_ratios.value = conf.GEAR_RATIOS;
    ctrl.rotor_inertias.value = tuple([g*g*r for (g,r) in zip(conf.GEAR_RATIOS, conf.ROTOR_INERTIAS)])
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

    ctrl.w_com.value = conf.w_com;
    ctrl.w_feet.value = conf.w_feet;
    ctrl.w_forces.value = conf.w_forces;
    ctrl.w_posture.value = conf.w_posture;
    ctrl.w_base_orientation.value = conf.w_base_orientation;
    ctrl.w_torques.value = conf.w_torques;
    
    ctrl.init(dt, robot_name);
    
    return ctrl;
    
def create_inverse_dynamics(robot, gains, dt=0.001):
    inv_dyn_ctrl = InverseDynamicsController("inv_dyn");
    plug(robot.device.robotState,             inv_dyn_ctrl.base6d_encoders);
    plug(robot.estimator_kin.dx,              inv_dyn_ctrl.jointsVelocities);
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
    plug(inv_dyn_ctrl.tauFF,            robot.torque_ctrl.tauFF);
    plug(inv_dyn_ctrl.tauFB,            robot.torque_ctrl.tauFB);
    plug(inv_dyn_ctrl.tauDes,           robot.estimator_ft.tauDes);
    plug(robot.estimator_ft.dynamicsError,       inv_dyn_ctrl.dynamicsError);
    
    inv_dyn_ctrl.dynamicsErrorGain.value = (NJ+6)*(0.0,);
    inv_dyn_ctrl.Kp.value = tuple(gains.k_s); # joint proportional gains
    inv_dyn_ctrl.Kd.value = tuple(gains.k_d); # joint derivative gains
    inv_dyn_ctrl.Kf.value = tuple(gains.k_f); # force proportional gains
    inv_dyn_ctrl.Ki.value = tuple(gains.k_i); # force integral gains
    inv_dyn_ctrl.controlledJoints.value = NJ*(1.0,);
    inv_dyn_ctrl.init(dt);
    return inv_dyn_ctrl;
        
def create_ctrl_manager(conf, dt, robot_name='robot'):
    ctrl_manager = ControlManager("ctrl_man");        

#    plug(ent.torque_ctrl.predictedJointsTorques, ctrl_manager.tau_predicted);
    ctrl_manager.tau_predicted.value = NJ*(0.0,);
    ctrl_manager.max_tau.value                              = NJ*(conf.CTRL_MANAGER_TAU_MAX,);
    ctrl_manager.max_current.value                          = NJ*(conf.CTRL_MANAGER_CURRENT_MAX,);
    ctrl_manager.percentageDriverDeadZoneCompensation.value = NJ*(conf.PERCENTAGE_DRIVER_DEAD_ZONE_COMPENSATION,);
    ctrl_manager.iMaxDeadZoneCompensation.value             = NJ*(conf.I_MAX_DEAD_ZONE_COMPENSATION,);
    ctrl_manager.in_out_gain.value                          = NJ*(conf.IN_OUT_GAIN,);
    ctrl_manager.bemfFactor.value                           = NJ*(0.0,);
    ctrl_manager.currents.value                             = NJ*(0.0,);
    #ctrl_manager.bemfFactor.value = tuple(Kpwm*0.1);
    
    # Init should be called before addCtrlMode 
    # because the size of state vector must be known.
    ctrl_manager.init(dt, conf.urdfFileName, conf.CTRL_MANAGER_CURRENT_MAX, robot_name)

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

    ctrl_manager.setRightFootForceSensorXYZ(conf.rightFootSensorXYZ);
    ctrl_manager.setRightFootSoleXYZ(conf.rightFootSoleXYZ);
    ctrl_manager.setDefaultMaxCurrent(conf.CTRL_MANAGER_CURRENT_MAX)
    return ctrl_manager;

def connect_ctrl_manager(ent):    
    # connect to device    
    plug(ent.device.robotState,             ent.ctrl_manager.base6d_encoders);
    plug(ent.device.currents,               ent.ctrl_manager.currents);
    plug(ent.estimator_kin.dx,              ent.ctrl_manager.dq);
    plug(ent.estimator_ft.jointsTorques,    ent.ctrl_manager.tau);    
    plug(ent.ctrl_manager.pwmDes,           ent.torque_ctrl.pwm);    
    ent.ctrl_manager.addCtrlMode("pos");
    ent.ctrl_manager.addCtrlMode("torque");    
    plug(ent.torque_ctrl.controlCurrent,        ent.ctrl_manager.ctrl_torque);
    plug(ent.pos_ctrl.pwmDes,                   ent.ctrl_manager.ctrl_pos);
    plug(ent.ctrl_manager.joints_ctrl_mode_torque,  ent.inv_dyn.active_joints);
    ent.ctrl_manager.setCtrlMode("all", "pos");
    plug(ent.ctrl_manager.pwmDesSafe,               ent.device.control);
    return;

def create_admittance_ctrl(robot, dt=0.001):
    admit_ctrl = AdmittanceController("adm_ctrl");
    plug(robot.device.robotState,             admit_ctrl.base6d_encoders);
    plug(robot.estimator_kin.dx,    admit_ctrl.jointsVelocities);
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

def create_topic(ros_import, signal, name, data_type='vector', sleep_time=0.1):
    ros_import.add(data_type, name+'_ros', name);
    plug(signal, ros_import.signal(name+'_ros'));
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
        create_topic(ros, robot.estimator_kin.dx,                            'jointsVelocities');
        create_topic(ros, robot.estimator_ft.contactWrenchLeftSole,          'contactWrenchLeftSole');
        create_topic(ros, robot.estimator_ft.contactWrenchRightSole,         'contactWrenchRightSole');
        create_topic(ros, robot.estimator_ft.jointsTorques,                  'jointsTorques');
#        create_topic(ros, robot.estimator.jointsTorquesFromInertiaModel,  'jointsTorquesFromInertiaModel');
#        create_topic(ros, robot.estimator.jointsTorquesFromMotorModel,    'jointsTorquesFromMotorModel');
#        create_topic(ros, robot.estimator.currentFiltered,                'currentFiltered');
    except:
        pass;

    try:
        create_topic(ros, robot.torque_ctrl.controlCurrent, 'controlCurrent');
        create_topic(ros, robot.torque_ctrl.desiredCurrent, 'desiredCurrent');
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
            f.write('Estimator F/T sensors delay: {0}\n'.format(robot.estimator_ft.getDelayFTsens()));
            f.write('Estimator use reference velocities: {0}\n'.format(robot.estimator_ft.getUseRefJointVel()));
            f.write('Estimator use reference accelerations: {0}\n'.format(robot.estimator_ft.getUseRefJointAcc()));
            f.write('Estimator accelerometer delay: {0}\n'.format(robot.estimator_ft.getDelayAcc()));
            f.write('Estimator gyroscope delay: {0}\n'.format(robot.estimator_ft.getDelayGyro()));
            f.write('Estimator use raw encoders: {0}\n'.format(robot.estimator_ft.getUseRawEncoders()));
            f.write('Estimator use f/t sensors: {0}\n'.format(robot.estimator_ft.getUseFTsensors()));
            f.write('Estimator f/t sensor offsets: {0}\n'.format(robot.estimator_ft.getFTsensorOffsets()));
        if(estimator_kin!=None):
            f.write('Estimator encoder delay: {0}\n'.format(robot.estimator_kin.getDelay()));
        if(inv_dyn!=None):
            f.write('Inv dyn Ks: {0}\n'.format(inv_dyn.Kp.value));
            f.write('Inv dyn Kd: {0}\n'.format(inv_dyn.Kd.value));
            f.write('Inv dyn Kf: {0}\n'.format(inv_dyn.Kf.value));
            f.write('Inv dyn Ki: {0}\n'.format(inv_dyn.Ki.value));
        if(torque_ctrl!=None):
            f.write('Torque ctrl KpTorque: {0}\n'.format (robot.torque_ctrl.KpTorque.value ));
            f.write('Torque ctrl KpCurrent: {0}\n'.format(robot.torque_ctrl.KpCurrent.value));
            f.write('Torque ctrl K_tau: {0}\n'.format(robot.torque_ctrl.k_tau.value));
            f.write('Torque ctrl K_v: {0}\n'.format(robot.torque_ctrl.k_v.value));
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
