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
from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.inverse_dynamics_controller import InverseDynamicsController
from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
from dynamic_graph.sot.torque_control.admittance_controller import AdmittanceController
from dynamic_graph.sot.torque_control.position_controller import PositionController
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import NJ
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import *
from dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains import *

def create_com_traj_gen(dt=0.001):
    com_traj_gen = NdTrajectoryGenerator("com_traj_gen");
    import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_conf as conf
    com_traj_gen.initial_value.value = conf.COM_DES;
    com_traj_gen.init(dt,3);
    return com_traj_gen ;

def create_free_flyer_locator(device, estimator, urdf, dynamic=None):
    from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
    ff_locator = FreeFlyerLocator("ffLocator");
    plug(device.robotState,           ff_locator.base6d_encoders);
    plug(estimator.jointsVelocities,  ff_locator.joint_velocities);
    if(dynamic!=None):
        plug(ff_locator.base6dFromFoot_encoders, dynamic.position);
    ff_locator.init(urdf);
    return ff_locator;
    
def create_flex_estimator(robot, ff_locator, dt=0.001):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce
    flex_est = HRP2ModelBaseFlexEstimatorIMUForce(robot, useMocap=False, dt=dt);
    flex_est.setOn(False);
    flex_est.interface.setExternalContactPresence(False);
    flex_est.interface.enabledContacts_lf_rf_lh_rh.value=(1,1,0,0);
    plug(ff_locator.v, flex_est.leftFootVelocity.sin2);
    plug(ff_locator.v, flex_est.rightFootVelocity.sin2);
    plug(ff_locator.v, flex_est.inputVel.sin2);
    plug(ff_locator.v, flex_est.DCom.sin2);
    return flex_est;
    
def create_floatingBase(flex_est, ff_locator):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import FromLocalToGLobalFrame 
    floatingBase = FromLocalToGLobalFrame(flex_est, "FloatingBase")
    plug(ff_locator.freeflyer_aa, floatingBase.sinPos);

    from dynamic_graph.sot.core import Selec_of_vector
    base_vel_no_flex = Selec_of_vector('base_vel_no_flex');
    plug(ff_locator.v, base_vel_no_flex.sin);
    base_vel_no_flex.selec(0, 6);
    plug(base_vel_no_flex.sout,   floatingBase.sinVel);
    return floatingBase
    
def create_position_controller(device, estimator, dt=0.001, traj_gen=None):
    posCtrl = PositionController('pos_ctrl')
    posCtrl.Kp.value = tuple(kp_pos);
    posCtrl.Kd.value = tuple(kd_pos);
    posCtrl.Ki.value = tuple(ki_pos);
    posCtrl.dqRef.value = NJ*(0.0,);
    plug(device.robotState,             posCtrl.base6d_encoders);    
    plug(estimator.jointsVelocities,    posCtrl.jointsVelocities);
    plug(posCtrl.pwmDes,                device.control);
    if(traj_gen!=None):
        plug(traj_gen.q,       posCtrl.qRef);
    posCtrl.init(dt);
    return posCtrl;

def create_trajectory_generator(device, dt=0.001):
    jtg = JointTrajectoryGenerator("jtg");
    plug(device.robotState,             jtg.base6d_encoders);
    jtg.init(dt);
    return jtg;

def create_estimator(device, dt, delay, traj_gen=None):
    estimator = ForceTorqueEstimator("estimator");

    plug(device.robotState,     estimator.base6d_encoders);
    plug(device.accelerometer,  estimator.accelerometer);
    plug(device.gyrometer,      estimator.gyroscope);
    plug(device.forceRLEG,      estimator.ftSensRightFoot);
    plug(device.forceLLEG,      estimator.ftSensLeftFoot);
    plug(device.forceRARM,      estimator.ftSensRightHand);
    plug(device.forceLARM,      estimator.ftSensLeftHand);
    plug(device.currents,       estimator.currentMeasure);
    if(traj_gen!=None):
        plug(traj_gen.dq,       estimator.dqRef);
        plug(traj_gen.ddq,      estimator.ddqRef);
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
        
def create_torque_controller(device, estimator, dt=0.001):
    torque_ctrl = JointTorqueController("jtc");
    plug(device.robotState,             torque_ctrl.base6d_encoders);
    plug(estimator.jointsVelocities,    torque_ctrl.jointsVelocities);
    plug(estimator.jointsAccelerations, torque_ctrl.jointsAccelerations);
    plug(estimator.jointsTorques,       torque_ctrl.jointsTorques);
    plug(estimator.currentFiltered,               torque_ctrl.measuredCurrent);
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
   
def create_balance_controller(device, floatingBase, estimator, torque_ctrl, traj_gen, com_traj_gen, urdfFileName, dt=0.001, ff_locator=None):
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");

    if(floatingBase!=None):
        from dynamic_graph.sot.core import Stack_of_vector
        base6d_encoders = Stack_of_vector('base6d_encoders');
        plug(floatingBase.soutPos, base6d_encoders.sin1);
        base6d_encoders.selec1(0,6);
        plug(device.robotState,    base6d_encoders.sin2);
        base6d_encoders.selec2(6,6+NJ);
        plug(base6d_encoders.sout,                 ctrl.q);
    
        v = Stack_of_vector('v');
        plug(floatingBase.soutVel, v.sin1);
        v.selec1(0,6);
        plug(estimator.jointsVelocities,    v.sin2);
        v.selec2(6,NJ+6);
        plug(v.sout,                            ctrl.v);
    else:
        plug(ff_locator.base6dFromFoot_encoders, ctrl.q);
        plug(ff_locator.v, ctrl.v);

    plug(estimator.contactWrenchRightSole,  ctrl.wrench_right_foot);
    plug(estimator.contactWrenchLeftSole,   ctrl.wrench_left_foot);
    plug(ctrl.tau_des,                      torque_ctrl.jointsTorquesDesired);
    plug(ctrl.tau_des,                      estimator.tauDes);

    plug(traj_gen.q,                        ctrl.posture_ref_pos);
    plug(traj_gen.dq,                       ctrl.posture_ref_vel);
    plug(traj_gen.ddq,                      ctrl.posture_ref_acc);
    plug(com_traj_gen.x,                    ctrl.com_ref_pos);
    plug(com_traj_gen.dx,                   ctrl.com_ref_vel);
    plug(com_traj_gen.ddx,                  ctrl.com_ref_acc);

    import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_conf as conf
    ctrl.rotor_inertias.value = conf.ROTOR_INERTIAS;
    ctrl.gear_ratios.value = conf.GEAR_RATIOS;
    ctrl.contact_normal.value = conf.FOOT_CONTACT_NORMAL;
    ctrl.contact_points.value = conf.RIGHT_FOOT_CONTACT_POINTS;
    ctrl.f_min.value = conf.fMin;
    ctrl.mu.value = conf.mu[0];
    ctrl.weight_contact_forces.value = (1e2, 1e2, 1e0, 1e3, 1e3, 1e3);
    ctrl.kp_com.value = 3*(conf.kp_com,);
    ctrl.kd_com.value = 3*(conf.kd_com,);
    ctrl.kp_constraints.value = 6*(conf.kp_constr,);
    ctrl.kd_constraints.value = 6*(conf.kd_constr,);
    ctrl.kp_posture.value = NJ*(conf.kp_posture,);
    ctrl.kd_posture.value = NJ*(conf.kd_posture,);
    ctrl.kp_pos.value = NJ*(conf.kp_pos,);
    ctrl.kd_pos.value = NJ*(conf.kd_pos,);

    ctrl.w_com.value = conf.w_com;
    ctrl.w_forces.value = conf.w_forces;
    ctrl.w_posture.value = conf.w_posture;
    ctrl.w_base_orientation.value = conf.w_base_orientation;
    ctrl.w_torques.value = conf.w_torques;
    
    ctrl.init(dt, urdfFileName);
    
    return ctrl;
    
def create_inverse_dynamics(device, estimator, torque_ctrl, traj_gen, dt=0.001):
    inv_dyn_ctrl = InverseDynamicsController("inv_dyn");
    plug(device.robotState,             inv_dyn_ctrl.base6d_encoders);
    plug(estimator.jointsVelocities,    inv_dyn_ctrl.jointsVelocities);
    plug(traj_gen.q,                    inv_dyn_ctrl.qRef);
    plug(traj_gen.dq,                   inv_dyn_ctrl.dqRef);
    plug(traj_gen.ddq,                  inv_dyn_ctrl.ddqRef);
    plug(estimator.contactWrenchRightSole,   inv_dyn_ctrl.fRightFoot);
    plug(estimator.contactWrenchLeftSole,    inv_dyn_ctrl.fLeftFoot);
    plug(estimator.contactWrenchRightHand,   inv_dyn_ctrl.fRightHand);
    plug(estimator.contactWrenchLeftHand,    inv_dyn_ctrl.fLeftHand);
    plug(traj_gen.fRightFoot,           inv_dyn_ctrl.fRightFootRef);
    plug(traj_gen.fLeftFoot,            inv_dyn_ctrl.fLeftFootRef);
    plug(traj_gen.fRightHand,           inv_dyn_ctrl.fRightHandRef);
    plug(traj_gen.fLeftHand,            inv_dyn_ctrl.fLeftHandRef);
    plug(estimator.baseAngularVelocity, inv_dyn_ctrl.baseAngularVelocity);
    plug(estimator.baseAcceleration,    inv_dyn_ctrl.baseAcceleration);
    plug(inv_dyn_ctrl.tauDes,           torque_ctrl.jointsTorquesDesired);
    plug(inv_dyn_ctrl.tauFF,            torque_ctrl.tauFF);
    plug(inv_dyn_ctrl.tauFB,            torque_ctrl.tauFB);
    plug(inv_dyn_ctrl.tauDes,           estimator.tauDes);
    plug(estimator.dynamicsError,       inv_dyn_ctrl.dynamicsError);
    
    inv_dyn_ctrl.dynamicsErrorGain.value = (NJ+6)*(0.0,);
    inv_dyn_ctrl.Kp.value = tuple(k_s); # joint proportional gains
    inv_dyn_ctrl.Kd.value = tuple(k_d); # joint derivative gains
    inv_dyn_ctrl.Kf.value = tuple(k_f); # force proportional gains
    inv_dyn_ctrl.Ki.value = tuple(k_i); # force integral gains
    inv_dyn_ctrl.controlledJoints.value = NJ*(1.0,);
    inv_dyn_ctrl.init(dt);
    return inv_dyn_ctrl;
        
def create_ctrl_manager(device, torque_ctrl, pos_ctrl, inv_dyn, estimator, dt=0.001):
    ctrl_manager = ControlManager("ctrl_man");
    plug(device.robotState,                  ctrl_manager.base6d_encoders);

    plug(torque_ctrl.predictedJointsTorques, ctrl_manager.tau_predicted);
    plug(estimator.jointsTorques,            ctrl_manager.tau);
    ctrl_manager.max_tau.value = NJ*(CTRL_MANAGER_TAU_MAX,);
    ctrl_manager.max_current.value = NJ*(CTRL_MANAGER_CURRENT_MAX,);
    ctrl_manager.percentageDriverDeadZoneCompensation.value = NJ*(PERCENTAGE_DRIVER_DEAD_ZONE_COMPENSATION,);
    ctrl_manager.signWindowsFilterSize.value = NJ*(SIGN_WINDOW_FILTER_SIZE,);
    ctrl_manager.bemfFactor.value = NJ*(0.0,);
    #ctrl_manager.bemfFactor.value = tuple(Kpwm*0.1);
    plug(ctrl_manager.pwmDesSafe,       device.control);
    plug(ctrl_manager.pwmDes,           torque_ctrl.pwm);
    ctrl_manager.addCtrlMode("pos");
    ctrl_manager.addCtrlMode("torque");    
    plug(estimator.jointsVelocities,    ctrl_manager.dq);
    plug(torque_ctrl.controlCurrent,    ctrl_manager.ctrl_torque);
    plug(pos_ctrl.pwmDes,               ctrl_manager.ctrl_pos);
    plug(ctrl_manager.joints_ctrl_mode_torque,  inv_dyn.active_joints);
    ctrl_manager.setCtrlMode("all", "pos");
    ctrl_manager.init(dt);
    return ctrl_manager;

def create_admittance_ctrl(device, estimator, ctrl_manager, traj_gen, dt=0.001):
    admit_ctrl = AdmittanceController("adm_ctrl");
    plug(device.robotState,             admit_ctrl.base6d_encoders);
    plug(estimator.jointsVelocities,    admit_ctrl.jointsVelocities);
    plug(estimator.contactWrenchRightSole,   admit_ctrl.fRightFoot);
    plug(estimator.contactWrenchLeftSole,    admit_ctrl.fLeftFoot);
    plug(estimator.contactWrenchRightHand,   admit_ctrl.fRightHand);
    plug(estimator.contactWrenchLeftHand,    admit_ctrl.fLeftHand);
    plug(traj_gen.fRightFoot,           admit_ctrl.fRightFootRef);
    plug(traj_gen.fLeftFoot,            admit_ctrl.fLeftFootRef);
    plug(traj_gen.fRightHand,           admit_ctrl.fRightHandRef);
    plug(traj_gen.fLeftHand,            admit_ctrl.fLeftHandRef);
    
    admit_ctrl.damping.value = 4*(0.05,);
    admit_ctrl.Kd.value = NJ*(0,);
    kf = -0.0005;
    km = -0.008;
    admit_ctrl.Kf.value = 3*(kf,)+3*(km,)+3*(kf,)+3*(km,)+3*(kf,)+3*(km,)+3*(kf,)+3*(km,);
    
    ctrl_manager.addCtrlMode("adm");
    plug(admit_ctrl.qDes,                   ctrl_manager.ctrl_adm);
    plug(ctrl_manager.joints_ctrl_mode_adm, admit_ctrl.controlledJoints);
    
    admit_ctrl.init(dt);
    return admit_ctrl;

def create_topic(ros_import, signal, name, data_type='vector', sleep_time=0.1):
    ros_import.add(data_type, name+'_ros', name);
    plug(signal, ros_import.signal(name+'_ros'));
    from time import sleep
    sleep(sleep_time);
    

def create_ros_topics(robot=None, estimator=None, torque_ctrl=None, traj_gen=None, ctrl_manager=None, inv_dyn=None, adm_ctrl=None, ff_locator=None, floatingBase=None):
    from dynamic_graph.ros import RosPublish
    ros = RosPublish('rosPublish');
    if(robot!=None):
        create_topic(ros, robot.device.robotState,      'robotState');
        create_topic(ros, robot.device.gyrometer,       'gyrometer');
        create_topic(ros, robot.device.accelerometer,   'accelerometer');
        create_topic(ros, robot.device.forceRLEG,       'forceRLEG');
        create_topic(ros, robot.device.forceLLEG,       'forceLLEG');
        create_topic(ros, robot.device.currents,        'currents');
#        create_topic(ros, robot.device.forceRARM,       'forceRARM');
#        create_topic(ros, robot.device.forceLARM,       'forceLARM');
        robot.device.after.addDownsampledSignal('rosPublish.trigger',1);

    if(estimator!=None):
        create_topic(ros, estimator.jointsVelocities,               'jointsVelocities');
        create_topic(ros, estimator.contactWrenchLeftSole,          'contactWrenchLeftSole');
        create_topic(ros, estimator.contactWrenchRightSole,         'contactWrenchRightSole');
        create_topic(ros, estimator.jointsTorques,                  'jointsTorques');
#        create_topic(ros, estimator.jointsTorquesFromInertiaModel,  'jointsTorquesFromInertiaModel');
#        create_topic(ros, estimator.jointsTorquesFromMotorModel,    'jointsTorquesFromMotorModel');
#        create_topic(ros, estimator.currentFiltered,                'currentFiltered');

    if(torque_ctrl!=None):
        create_topic(ros, torque_ctrl.controlCurrent, 'controlCurrent');
        create_topic(ros, torque_ctrl.desiredCurrent, 'desiredCurrent');

    if(traj_gen!=None):
        create_topic(ros, traj_gen.q,   'q_ref');
        create_topic(ros, traj_gen.dq,  'dq_ref');
        create_topic(ros, traj_gen.ddq, 'ddq_ref');

    if(ctrl_manager!=None):
        create_topic(ros, ctrl_manager.pwmDes,                  'i_des');
        create_topic(ros, ctrl_manager.pwmDesSafe,              'i_des_safe');
#        create_topic(ros, ctrl_manager.signOfControlFiltered,   'signOfControlFiltered');
#        create_topic(ros, ctrl_manager.signOfControl,           'signOfControl');

    if(inv_dyn!=None):
        create_topic(ros, inv_dyn.tau_des, 'tau_des');

    if(ff_locator!=None):
        create_topic(ros, ff_locator.base6dFromFoot_encoders,        'base6dFromFoot_encoders');

    if(floatingBase!=None):
        create_topic(ros, floatingBase.soutPos, 'floatingBase_pos');
    
    return ros;
    
    
def addTrace(tracer, entity, signalName):
    """
    Add a signal to a tracer
    """
    signal = '{0}.{1}'.format(entity.name, signalName);
    filename = '{0}-{1}'.format(entity.name, signalName);
    tracer.add(signal, filename);
    
def addSignalsToTracer(tracer, device, traj_gen, torque_ctrl):
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

    addSignalsToTracer(tracer, device, traj_gen, torque_ctrl);
        
    with open('/tmp/dg_info.dat', 'a') as f:
        if(estimator!=None):
            f.write('Estimator encoder delay: {0}\n'.format(estimator.getDelayEnc()));
            f.write('Estimator F/T sensors delay: {0}\n'.format(estimator.getDelayFTsens()));
            f.write('Estimator accelerometer delay: {0}\n'.format(estimator.getDelayAcc()));
            f.write('Estimator gyroscope delay: {0}\n'.format(estimator.getDelayGyro()));
            f.write('Estimator use reference velocities: {0}\n'.format(estimator.getUseRefJointVel()));
            f.write('Estimator use reference accelerations: {0}\n'.format(estimator.getUseRefJointAcc()));
            f.write('Estimator use raw encoders: {0}\n'.format(estimator.getUseRawEncoders()));
            f.write('Estimator use f/t sensors: {0}\n'.format(estimator.getUseFTsensors()));
            f.write('Estimator f/t sensor offsets: {0}\n'.format(estimator.getFTsensorOffsets()));
        if(inv_dyn!=None):
            f.write('Inv dyn Ks: {0}\n'.format(inv_dyn.Kp.value));
            f.write('Inv dyn Kd: {0}\n'.format(inv_dyn.Kd.value));
            f.write('Inv dyn Kf: {0}\n'.format(inv_dyn.Kf.value));
            f.write('Inv dyn Ki: {0}\n'.format(inv_dyn.Ki.value));
        if(torque_ctrl!=None):
            f.write('Torque ctrl KpTorque: {0}\n'.format (torque_ctrl.KpTorque.value ));
            f.write('Torque ctrl KpCurrent: {0}\n'.format(torque_ctrl.KpCurrent.value));
            f.write('Torque ctrl K_tau: {0}\n'.format(torque_ctrl.k_tau.value));
            f.write('Torque ctrl K_v: {0}\n'.format(torque_ctrl.k_v.value));
    f.close();
    return tracer;

def reset_tracer(device,tracer):
    tracer.stop();
    tracer.dump();
    tracer.close();
    tracer.clear();
    tracer = create_tracer(device);
    return tracer;
    


