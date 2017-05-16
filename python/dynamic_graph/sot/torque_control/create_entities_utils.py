# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph import plug
from dynamic_graph.sot.torque_control.force_torque_estimator import ForceTorqueEstimator
from dynamic_graph.sot.torque_control.joint_torque_controller import JointTorqueController
from dynamic_graph.sot.torque_control.joint_trajectory_generator import JointTrajectoryGenerator
from dynamic_graph.sot.torque_control.nd_trajectory_generator import NdTrajectoryGenerator
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.inverse_dynamics_controller import InverseDynamicsController
from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
from dynamic_graph.sot.torque_control.admittance_controller import AdmittanceController
from dynamic_graph.sot.torque_control.position_controller import PositionController
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import NJ
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import *
from dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains import *

def create_imu_filter(ent, dt):
    from dynamic_graph.sot.torque_control.madgwickahrs import MadgwickAHRS
    imu_filter = MadgwickAHRS('imu_filter');
    imu_filter.init(dt);
    plug(ent.device.accelerometer, imu_filter.accelerometer);
    plug(ent.device.gyrometer,     imu_filter.gyroscope);
    return imu_filter;

def create_com_traj_gen(dt=0.001):
    com_traj_gen = NdTrajectoryGenerator("com_traj_gen");
    import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_conf as conf
    com_traj_gen.initial_value.value = conf.COM_DES;
    com_traj_gen.init(dt,3);
    return com_traj_gen ;

def create_free_flyer_locator(ent, urdf):
    from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
    ff_locator = FreeFlyerLocator("ffLocator");
    plug(ent.device.robotState,           ff_locator.base6d_encoders);
    plug(ent.estimator.jointsVelocities,  ff_locator.joint_velocities);
    plug(ff_locator.base6dFromFoot_encoders, ent.dynamic.position);
    ff_locator.init(urdf);
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
    
def create_floatingBase(ent):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import FromLocalToGLobalFrame 
    floatingBase = FromLocalToGLobalFrame(ent.flex_est, "FloatingBase")
    plug(ent.ff_locator.freeflyer_aa, floatingBase.sinPos);

    from dynamic_graph.sot.core import Selec_of_vector
    base_vel_no_flex = Selec_of_vector('base_vel_no_flex');
    plug(ent.ff_locator.v, base_vel_no_flex.sin);
    base_vel_no_flex.selec(0, 6);
    plug(base_vel_no_flex.sout,   floatingBase.sinVel);
    return floatingBase
    
def create_position_controller(ent, dt=0.001):
    posCtrl = PositionController('pos_ctrl')
    posCtrl.Kp.value = tuple(kp_pos);
    posCtrl.Kd.value = tuple(kd_pos);
    posCtrl.Ki.value = tuple(ki_pos);
    posCtrl.dqRef.value = NJ*(0.0,);
    plug(ent.device.robotState,             posCtrl.base6d_encoders);  
    try:  # this works only in simulation
        plug(ent.device.jointsVelocities,    posCtrl.jointsVelocities);
    except:
        plug(ent.estimator.jointsVelocities, posCtrl.jointsVelocities);
        pass;
    plug(posCtrl.pwmDes,                ent.device.control);
    try:
        plug(ent.traj_gen.q,       posCtrl.qRef);
    except:
        pass;
    posCtrl.init(dt);
    return posCtrl;

def create_trajectory_generator(device, dt=0.001):
    jtg = JointTrajectoryGenerator("jtg");
    plug(device.robotState,             jtg.base6d_encoders);
    jtg.init(dt);
    return jtg;

def create_estimator(ent, dt, delay):
    estimator = ForceTorqueEstimator("estimator");

    plug(ent.device.robotState,     estimator.base6d_encoders);
    plug(ent.device.accelerometer,  estimator.accelerometer);
    plug(ent.device.gyrometer,      estimator.gyroscope);
    plug(ent.device.forceRLEG,      estimator.ftSensRightFoot);
    plug(ent.device.forceLLEG,      estimator.ftSensLeftFoot);
    plug(ent.device.forceRARM,      estimator.ftSensRightHand);
    plug(ent.device.forceLARM,      estimator.ftSensLeftHand);
    plug(ent.device.currents,       estimator.currentMeasure);
    try:
        plug(ent.traj_gen.dq,       estimator.dqRef);
        plug(ent.traj_gen.ddq,      estimator.ddqRef);
    except:
        pass;
    estimator.wCurrentTrust.value     = tuple(NJ*[CURRENT_TORQUE_ESTIMATION_TRUST,])
    estimator.saturationCurrent.value = tuple(NJ*[SATURATION_CURRENT,])
    estimator.motorParameterKt_p.value  = tuple(Kt_p)
    estimator.motorParameterKt_n.value  = tuple(Kt_n)
    estimator.motorParameterKf_p.value  = tuple(Kf_p)
    estimator.motorParameterKf_n.value  = tuple(Kf_n)
    estimator.motorParameterKv_p.value  = tuple(Kv_p)
    estimator.motorParameterKv_n.value  = tuple(Kv_n)
    estimator.motorParameterKa_p.value  = tuple(Ka_p)
    estimator.motorParameterKa_n.value  = tuple(Ka_n)

    estimator.init(dt,delay,delay,delay,delay,delay,True);
    
    return estimator;
        
def create_torque_controller(ent, dt=0.001):
    torque_ctrl = JointTorqueController("jtc");
    plug(ent.device.robotState,             torque_ctrl.base6d_encoders);
    plug(ent.estimator.jointsVelocities,    torque_ctrl.jointsVelocities);
    plug(ent.estimator.jointsAccelerations, torque_ctrl.jointsAccelerations);
    plug(ent.estimator.jointsTorques,       torque_ctrl.jointsTorques);
    plug(ent.estimator.currentFiltered,               torque_ctrl.measuredCurrent);
    torque_ctrl.jointsTorquesDesired.value = NJ*(0.0,);
    torque_ctrl.KpTorque.value = tuple(k_p_torque);
    torque_ctrl.KiTorque.value = NJ*(0.0,);
    torque_ctrl.KpCurrent.value = tuple(k_p_current);
    torque_ctrl.KiCurrent.value = NJ*(0.0,);
    torque_ctrl.k_tau.value = tuple(k_tau);
    torque_ctrl.k_v.value   = tuple(k_v);
    torque_ctrl.frictionCompensationPercentage.value = NJ*(FRICTION_COMPENSATION_PERCENTAGE,);

    torque_ctrl.motorParameterKt_p.value  = tuple(Kt_p)
    torque_ctrl.motorParameterKt_n.value  = tuple(Kt_n)
    torque_ctrl.motorParameterKf_p.value  = tuple(Kf_p)
    torque_ctrl.motorParameterKf_n.value  = tuple(Kf_n)
    torque_ctrl.motorParameterKv_p.value  = tuple(Kv_p)
    torque_ctrl.motorParameterKv_n.value  = tuple(Kv_n)
    torque_ctrl.motorParameterKa_p.value  = tuple(Ka_p)
    torque_ctrl.motorParameterKa_n.value  = tuple(Ka_n)
    torque_ctrl.polySignDq.value          = NJ*(3,); 
    torque_ctrl.init(dt);
    return torque_ctrl;
   
def create_balance_controller(ent, urdfFileName, dt=0.001):
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");

    try:
        from dynamic_graph.sot.core import Stack_of_vector
        ent.base6d_encoders = Stack_of_vector('base6d_encoders');
        plug(ent.floatingBase.soutPos, ent.base6d_encoders.sin1);
        ent.base6d_encoders.selec1(0,6);
        plug(ent.device.robotState,    ent.base6d_encoders.sin2);
        ent.base6d_encoders.selec2(6,6+NJ);
        plug(ent.base6d_encoders.sout,                 ctrl.q);
    
        ent.v = Stack_of_vector('v');
        plug(ent.floatingBase.soutVel, ent.v.sin1);
        ent.v.selec1(0,6);
        plug(ent.estimator.jointsVelocities,    ent.v.sin2);
        ent.v.selec2(6,NJ+6);
        plug(ent.v.sout,                        ctrl.v);
    except:
        plug(ent.ff_locator.base6dFromFoot_encoders, ctrl.q);
        plug(ent.ff_locator.v, ctrl.v);

    plug(ent.estimator.contactWrenchRightSole,  ctrl.wrench_right_foot);
    plug(ent.estimator.contactWrenchLeftSole,   ctrl.wrench_left_foot);
    plug(ctrl.tau_des,                          ent.torque_ctrl.jointsTorquesDesired);
    plug(ctrl.tau_des,                          ent.estimator.tauDes);

    plug(ctrl.right_foot_pos,       ent.rf_traj_gen.initial_value);
    plug(ent.rf_traj_gen.x,         ctrl.rf_ref_pos);
    plug(ent.rf_traj_gen.dx,        ctrl.rf_ref_vel);
    plug(ent.rf_traj_gen.ddx,       ctrl.rf_ref_acc);

    plug(ctrl.left_foot_pos,        ent.lf_traj_gen.initial_value);
    plug(ent.lf_traj_gen.x,         ctrl.lf_ref_pos);
    plug(ent.lf_traj_gen.dx,        ctrl.lf_ref_vel);
    plug(ent.lf_traj_gen.ddx,       ctrl.lf_ref_acc);
    
    plug(ent.traj_gen.q,                        ctrl.posture_ref_pos);
    plug(ent.traj_gen.dq,                       ctrl.posture_ref_vel);
    plug(ent.traj_gen.ddq,                      ctrl.posture_ref_acc);
    plug(ent.com_traj_gen.x,                    ctrl.com_ref_pos);
    plug(ent.com_traj_gen.dx,                   ctrl.com_ref_vel);
    plug(ent.com_traj_gen.ddx,                  ctrl.com_ref_acc);

    import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_conf as conf
    ctrl.rotor_inertias.value = conf.ROTOR_INERTIAS;
    ctrl.gear_ratios.value = conf.GEAR_RATIOS;
    ctrl.contact_normal.value = conf.FOOT_CONTACT_NORMAL;
    ctrl.contact_points.value = conf.RIGHT_FOOT_CONTACT_POINTS;
    ctrl.f_min.value = conf.fMin;
    ctrl.f_max.value = conf.fMax;
    ctrl.mu.value = conf.mu[0];
    ctrl.weight_contact_forces.value = (1e2, 1e2, 1e0, 1e3, 1e3, 1e3);
    ctrl.kp_com.value = 3*(conf.kp_com,);
    ctrl.kd_com.value = 3*(conf.kd_com,);
    ctrl.kp_constraints.value = 6*(conf.kp_constr,);
    ctrl.kd_constraints.value = 6*(conf.kd_constr,);
    ctrl.kp_feet.value = 6*(conf.kp_feet,);
    ctrl.kd_feet.value = 6*(conf.kd_feet,);
    ctrl.kp_posture.value = NJ*(conf.kp_posture,);
    ctrl.kd_posture.value = NJ*(conf.kd_posture,);
    ctrl.kp_pos.value = NJ*(conf.kp_pos,);
    ctrl.kd_pos.value = NJ*(conf.kd_pos,);

    ctrl.w_com.value = conf.w_com;
    ctrl.w_feet.value = conf.w_feet;
    ctrl.w_forces.value = conf.w_forces;
    ctrl.w_posture.value = conf.w_posture;
    ctrl.w_base_orientation.value = conf.w_base_orientation;
    ctrl.w_torques.value = conf.w_torques;
    
    ctrl.init(dt, urdfFileName);
    
    return ctrl;
    
def create_inverse_dynamics(ent, dt=0.001):
    inv_dyn_ctrl = InverseDynamicsController("inv_dyn");
    plug(ent.device.robotState,             inv_dyn_ctrl.base6d_encoders);
    plug(ent.estimator.jointsVelocities,    inv_dyn_ctrl.jointsVelocities);
    plug(ent.traj_gen.q,                    inv_dyn_ctrl.qRef);
    plug(ent.traj_gen.dq,                   inv_dyn_ctrl.dqRef);
    plug(ent.traj_gen.ddq,                  inv_dyn_ctrl.ddqRef);
    plug(ent.estimator.contactWrenchRightSole,   inv_dyn_ctrl.fRightFoot);
    plug(ent.estimator.contactWrenchLeftSole,    inv_dyn_ctrl.fLeftFoot);
    plug(ent.estimator.contactWrenchRightHand,   inv_dyn_ctrl.fRightHand);
    plug(ent.estimator.contactWrenchLeftHand,    inv_dyn_ctrl.fLeftHand);
    plug(ent.traj_gen.fRightFoot,           inv_dyn_ctrl.fRightFootRef);
    plug(ent.traj_gen.fLeftFoot,            inv_dyn_ctrl.fLeftFootRef);
    plug(ent.traj_gen.fRightHand,           inv_dyn_ctrl.fRightHandRef);
    plug(ent.traj_gen.fLeftHand,            inv_dyn_ctrl.fLeftHandRef);
    plug(ent.estimator.baseAngularVelocity, inv_dyn_ctrl.baseAngularVelocity);
    plug(ent.estimator.baseAcceleration,    inv_dyn_ctrl.baseAcceleration);
    plug(inv_dyn_ctrl.tauDes,           ent.torque_ctrl.jointsTorquesDesired);
    plug(inv_dyn_ctrl.tauFF,            ent.torque_ctrl.tauFF);
    plug(inv_dyn_ctrl.tauFB,            ent.torque_ctrl.tauFB);
    plug(inv_dyn_ctrl.tauDes,           ent.estimator.tauDes);
    plug(ent.estimator.dynamicsError,       inv_dyn_ctrl.dynamicsError);
    
    inv_dyn_ctrl.dynamicsErrorGain.value = (NJ+6)*(0.0,);
    inv_dyn_ctrl.Kp.value = tuple(k_s); # joint proportional gains
    inv_dyn_ctrl.Kd.value = tuple(k_d); # joint derivative gains
    inv_dyn_ctrl.Kf.value = tuple(k_f); # force proportional gains
    inv_dyn_ctrl.Ki.value = tuple(k_i); # force integral gains
    inv_dyn_ctrl.controlledJoints.value = NJ*(1.0,);
    inv_dyn_ctrl.init(dt);
    return inv_dyn_ctrl;
        
def create_ctrl_manager(ent, dt=0.001):
    ctrl_manager = ControlManager("ctrl_man");
    plug(ent.device.robotState,                  ctrl_manager.base6d_encoders);

    plug(ent.torque_ctrl.predictedJointsTorques, ctrl_manager.tau_predicted);
    plug(ent.estimator.jointsTorques,            ctrl_manager.tau);
    ctrl_manager.max_tau.value = NJ*(CTRL_MANAGER_TAU_MAX,);
    ctrl_manager.max_current.value = NJ*(CTRL_MANAGER_CURRENT_MAX,);
    ctrl_manager.percentageDriverDeadZoneCompensation.value = NJ*(PERCENTAGE_DRIVER_DEAD_ZONE_COMPENSATION,);
    ctrl_manager.signWindowsFilterSize.value = NJ*(SIGN_WINDOW_FILTER_SIZE,);
    ctrl_manager.bemfFactor.value = NJ*(0.0,);
    #ctrl_manager.bemfFactor.value = tuple(Kpwm*0.1);
    plug(ctrl_manager.pwmDesSafe,       ent.device.control);
    plug(ctrl_manager.pwmDes,           ent.torque_ctrl.pwm);
    ctrl_manager.addCtrlMode("pos");
    ctrl_manager.addCtrlMode("torque");    
    plug(ent.estimator.jointsVelocities,    ctrl_manager.dq);
    plug(ent.torque_ctrl.controlCurrent,    ctrl_manager.ctrl_torque);
    plug(ent.pos_ctrl.pwmDes,               ctrl_manager.ctrl_pos);
    plug(ctrl_manager.joints_ctrl_mode_torque,  ent.inv_dyn.active_joints);
    ctrl_manager.setCtrlMode("all", "pos");
    ctrl_manager.init(dt);
    return ctrl_manager;

def create_admittance_ctrl(ent, dt=0.001):
    admit_ctrl = AdmittanceController("adm_ctrl");
    plug(ent.device.robotState,             admit_ctrl.base6d_encoders);
    plug(ent.estimator.jointsVelocities,    admit_ctrl.jointsVelocities);
    plug(ent.estimator.contactWrenchRightSole,   admit_ctrl.fRightFoot);
    plug(ent.estimator.contactWrenchLeftSole,    admit_ctrl.fLeftFoot);
    plug(ent.estimator.contactWrenchRightHand,   admit_ctrl.fRightHand);
    plug(ent.estimator.contactWrenchLeftHand,    admit_ctrl.fLeftHand);
    plug(ent.traj_gen.fRightFoot,           admit_ctrl.fRightFootRef);
    plug(ent.traj_gen.fLeftFoot,            admit_ctrl.fLeftFootRef);
    plug(ent.traj_gen.fRightHand,           admit_ctrl.fRightHandRef);
    plug(ent.traj_gen.fLeftHand,            admit_ctrl.fLeftHandRef);
    
    admit_ctrl.damping.value = 4*(0.05,);
    admit_ctrl.Kd.value = NJ*(0,);
    kf = -0.0005;
    km = -0.008;
    admit_ctrl.Kf.value = 3*(kf,)+3*(km,)+3*(kf,)+3*(km,)+3*(kf,)+3*(km,)+3*(kf,)+3*(km,);
    
    ent.ctrl_manager.addCtrlMode("adm");
    plug(admit_ctrl.qDes,                       ent.ctrl_manager.ctrl_adm);
    plug(ent.ctrl_manager.joints_ctrl_mode_adm, admit_ctrl.controlledJoints);
    
    admit_ctrl.init(dt);
    return admit_ctrl;

def create_topic(ros_import, signal, name, data_type='vector', sleep_time=0.1):
    ros_import.add(data_type, name+'_ros', name);
    plug(signal, ros_import.signal(name+'_ros'));
    from time import sleep
    sleep(sleep_time);
    

def create_ros_topics(ent):
    from dynamic_graph.ros import RosPublish
    ros = RosPublish('rosPublish');
    try:
        create_topic(ros, ent.device.robotState,      'robotState');
        create_topic(ros, ent.device.gyrometer,       'gyrometer');
        create_topic(ros, ent.device.accelerometer,   'accelerometer');
        create_topic(ros, ent.device.forceRLEG,       'forceRLEG');
        create_topic(ros, ent.device.forceLLEG,       'forceLLEG');
        create_topic(ros, ent.device.currents,        'currents');
#        create_topic(ros, ent.device.forceRARM,       'forceRARM');
#        create_topic(ros, ent.device.forceLARM,       'forceLARM');
        ent.device.after.addDownsampledSignal('rosPublish.trigger',1);
    except:
        pass;
    
    try:
        create_topic(ros, ent.estimator.jointsVelocities,               'jointsVelocities');
        create_topic(ros, ent.estimator.contactWrenchLeftSole,          'contactWrenchLeftSole');
        create_topic(ros, ent.estimator.contactWrenchRightSole,         'contactWrenchRightSole');
        create_topic(ros, ent.estimator.jointsTorques,                  'jointsTorques');
#        create_topic(ros, ent.estimator.jointsTorquesFromInertiaModel,  'jointsTorquesFromInertiaModel');
#        create_topic(ros, ent.estimator.jointsTorquesFromMotorModel,    'jointsTorquesFromMotorModel');
#        create_topic(ros, ent.estimator.currentFiltered,                'currentFiltered');
    except:
        pass;

    try:
        create_topic(ros, ent.torque_ctrl.controlCurrent, 'controlCurrent');
        create_topic(ros, ent.torque_ctrl.desiredCurrent, 'desiredCurrent');
    except:
        pass;

    try:
        create_topic(ros, ent.traj_gen.q,   'q_ref');
#        create_topic(ros, ent.traj_gen.dq,  'dq_ref');
#        create_topic(ros, ent.traj_gen.ddq, 'ddq_ref');
    except:
        pass;

    try:
        create_topic(ros, ent.ctrl_manager.pwmDes,                  'i_des');
        create_topic(ros, ent.ctrl_manager.pwmDesSafe,              'i_des_safe');
#        create_topic(ros, ent.ctrl_manager.signOfControlFiltered,   'signOfControlFiltered');
#        create_topic(ros, ent.ctrl_manager.signOfControl,           'signOfControl');
    except:
        pass;

    try:
        create_topic(ros, ent.inv_dyn.tau_des, 'tau_des');
    except:
        pass;

    try:
        create_topic(ros, ent.ff_locator.base6dFromFoot_encoders,        'base6dFromFoot_encoders');
    except:
        pass;

    try:
        create_topic(ros, ent.floatingBase.soutPos, 'floatingBase_pos');
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


def create_tracer(device, traj_gen=None, estimator=None, inv_dyn=None, torque_ctrl=None):
    tracer = TracerRealTime('motor_id_trace');
    tracer.setBufferSize(80*(2**20));
    tracer.open('/tmp/','dg_','.dat');
    device.after.addSignal('{0}.triger'.format(tracer.name));

    addSignalsToTracer(tracer, device);
        
    with open('/tmp/dg_info.dat', 'a') as f:
        if(estimator!=None):
            f.write('Estimator encoder delay: {0}\n'.format(ent.estimator.getDelayEnc()));
            f.write('Estimator F/T sensors delay: {0}\n'.format(ent.estimator.getDelayFTsens()));
            f.write('Estimator accelerometer delay: {0}\n'.format(ent.estimator.getDelayAcc()));
            f.write('Estimator gyroscope delay: {0}\n'.format(ent.estimator.getDelayGyro()));
            f.write('Estimator use reference velocities: {0}\n'.format(ent.estimator.getUseRefJointVel()));
            f.write('Estimator use reference accelerations: {0}\n'.format(ent.estimator.getUseRefJointAcc()));
            f.write('Estimator use raw encoders: {0}\n'.format(ent.estimator.getUseRawEncoders()));
            f.write('Estimator use f/t sensors: {0}\n'.format(ent.estimator.getUseFTsensors()));
            f.write('Estimator f/t sensor offsets: {0}\n'.format(ent.estimator.getFTsensorOffsets()));
        if(inv_dyn!=None):
            f.write('Inv dyn Ks: {0}\n'.format(inv_dyn.Kp.value));
            f.write('Inv dyn Kd: {0}\n'.format(inv_dyn.Kd.value));
            f.write('Inv dyn Kf: {0}\n'.format(inv_dyn.Kf.value));
            f.write('Inv dyn Ki: {0}\n'.format(inv_dyn.Ki.value));
        if(torque_ctrl!=None):
            f.write('Torque ctrl KpTorque: {0}\n'.format (ent.torque_ctrl.KpTorque.value ));
            f.write('Torque ctrl KpCurrent: {0}\n'.format(ent.torque_ctrl.KpCurrent.value));
            f.write('Torque ctrl K_tau: {0}\n'.format(ent.torque_ctrl.k_tau.value));
            f.write('Torque ctrl K_v: {0}\n'.format(ent.torque_ctrl.k_v.value));
    f.close();
    return tracer;

def reset_tracer(device,tracer):
    tracer.stop();
    tracer.dump();
    tracer.close();
    tracer.clear();
    tracer = create_tracer(device);
    return tracer;
