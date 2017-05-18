# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import numpy as np
from numpy.linalg import norm
from dynamic_graph import plug
from dynamic_graph.sot.torque_control.create_entities_utils import NJ
from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
#from dynamic_graph.sot.torque_control.hrp2_device_torque_ctrl import HRP2DeviceTorqueCtrl
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.core.robot_simu import RobotSimu
from dynamic_graph.sot.torque_control.create_entities_utils import *
from dynamic_graph.sot.torque_control.utils.sot_utils import *
from time import sleep
import time

from pinocchio_inv_dyn.simulator import Simulator
from pinocchio_inv_dyn.robot_wrapper import RobotWrapper
import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_sim_conf as conf
from pinocchio.utils import zero as mat_zeros
from pinocchio import Quaternion
from pinocchio.rpy import rpyToMatrix

def to_tuple(x):
    return tuple(np.asarray(x).squeeze());

def config_sot_to_urdf(q):
    # GEPETTO VIEWER Free flyer 0-6, CHEST HEAD 7-10, LARM 11-17, RARM 18-24, LLEG 25-30, RLEG 31-36
    # ROBOT VIEWER # Free flyer0-5, RLEG 6-11, LLEG 12-17, CHEST HEAD 18-21, RARM 22-28, LARM 29-35
    qUrdf = mat_zeros(37);
    qUrdf[:3,0] = q[:3,0];
    quatMat = rpyToMatrix(q[3:6,0]);
    quatVec = Quaternion(quatMat);
    qUrdf[3:7,0]   = quatVec.coeffs();
    qUrdf[7:11,0]  = q[18:22,0]; # chest-head
    qUrdf[11:18,0] = q[29:,0]; # larm
    qUrdf[18:25,0] = q[22:29,0]; # rarm
    qUrdf[25:31,0] = q[12:18,0]; # lleg
    qUrdf[31:,0]   = q[6:12,0]; # rleg
    return qUrdf;
    
def joints_sot_to_urdf(q):
    # GEPETTO VIEWER Free flyer 0-6, CHEST HEAD 7-10, LARM 11-17, RARM 18-24, LLEG 25-30, RLEG 31-36
    # ROBOT VIEWER # Free flyer0-5, RLEG 6-11, LLEG 12-17, CHEST HEAD 18-21, RARM 22-28, LARM 29-35
    qUrdf = mat_zeros(30);
    qUrdf[:4,0]  = q[12:16,0]; # chest-head
    qUrdf[4:11,0] = q[23:,0]; # larm
    qUrdf[11:18,0] = q[16:23,0]; # rarm
    qUrdf[18:24,0] = q[6:12,0]; # lleg
    qUrdf[24:,0]   = q[:6,0]; # rleg
    return qUrdf;

def create_device(q=None):
    device = RobotSimu("device");
    device.setControlInputType('acceleration');
    if(q==None):
        q = (0,0,0.7)+(NJ+3)*(0.0,);
    ddq_des = (NJ+6)*(0.0,);
    device.resize(NJ+6);
    device.set(q);
    device.control.value = ddq_des;
    device.increment(0.001);
    return device;
    
def create_device_torque_ctrl(dt, urdfFilename, q=None):
    device = HRP2DeviceTorqueCtrl("device");
    device.kp_constraints.value = 6*(conf.kp_constr,);
    device.kd_constraints.value = 6*(conf.kd_constr,);
    device.rotor_inertias.value = conf.ROTOR_INERTIAS;
    device.gear_ratios.value = conf.GEAR_RATIOS;
    device.init(dt, urdfFilename);
    if(q==None):
        q = (0,0,0.7)+(NJ+3)*(0.0,);
    tau_des = NJ*(0.0,);
    device.set(q);
    device.setVelocity((NJ+6)*(0.0,));
    device.control.value = tau_des;
    device.increment(0.001);
    return device;
    
def main_new(dt=0.001, delay=0.01):
    COM_DES_1 = (0.012, 0.1, 0.81);
    COM_DES_2 = (0.012, -0.1, 0.81);
    dt = conf.dt;
    q0 = conf.q0_urdf;
    v0 = conf.v0;
    nv = v0.shape[0];
    
    simulator  = Simulator('hrp2_sim', q0, v0, conf.fMin, conf.mu, dt, conf.model_path, conf.urdfFileName);
    simulator.ENABLE_TORQUE_LIMITS = conf.FORCE_TORQUE_LIMITS;
    simulator.ENABLE_FORCE_LIMITS = conf.ENABLE_FORCE_LIMITS;
    simulator.ENABLE_JOINT_LIMITS = conf.FORCE_JOINT_LIMITS;
    simulator.ACCOUNT_FOR_ROTOR_INERTIAS = conf.ACCOUNT_FOR_ROTOR_INERTIAS;
    simulator.VIEWER_DT = conf.DT_VIEWER;
    simulator.CONTACT_FORCE_ARROW_SCALE = 2e-3;
    simulator.verb=0;
    robot = simulator.r;
    
    device          = create_device_torque_ctrl(dt, conf.urdfFileName, conf.q0_sot);
    traj_gen        = create_trajectory_generator(device, dt);
    (estimator_ft, estimator_kin)       = create_estimators(device, dt,);
    ff_locator      = create_free_flyer_locator(device, conf.urdfFileName);
#    flex_est        = create_flex_estimator(robot,dt);
#    floatingBase    = create_floatingBase(flex_est,ff_locator);    
    torque_ctrl     = create_torque_controller(device);
    pos_ctrl        = create_position_controller(device, dt);
    
    ctrl = create_balance_controller(device, conf.urdfFileName, dt);
    plug(device.state,                 ctrl.q);    
    ctrl.init(dt, conf.urdfFileName);
    ctrl.active_joints.value = conf.active_joints;
    
    ctrl_manager    = create_ctrl_manager(device, dt);
#    plug(device.jointsVelocities, torque_ctrl.jointsVelocities);    
    plug(device.velocity,       ctrl.v);
    plug(ctrl.tau_des,          device.control);
    t = 0.0;
    v = mat_zeros(nv);
    dv = mat_zeros(nv);
    x_rf = robot.framePosition(robot.model.getFrameId('RLEG_JOINT5')).translation;
    x_lf = robot.framePosition(robot.model.getFrameId('LLEG_JOINT5')).translation;
    device.displaySignals()
    for i in range(conf.MAX_TEST_DURATION):
#        if(norm(dv[6:24]) > 1e-8):
#            print "ERROR acceleration of blocked axes is not zero:", norm(dv[6:24]);
        device.increment(dt);
        
        if(i==1500):
            ctrl.com_ref_pos.value = COM_DES_2;
        if(i%10==0):
            q = np.matrix(device.state.value).T;
            q_urdf = config_sot_to_urdf(q);
            simulator.viewer.updateRobotConfig(q_urdf);
            ctrl.f_des_right_foot.recompute(i);
            ctrl.f_des_left_foot.recompute(i);
            f_rf = np.matrix(ctrl.f_des_right_foot.value).T
            f_lf = np.matrix(ctrl.f_des_left_foot.value).T
            simulator.updateContactForcesInViewer(['rf', 'lf'], 
                                                  [x_rf, x_lf], 
                                                  [f_rf, f_lf]);
            ctrl.com.recompute(i);
            com = np.matrix(ctrl.com.value).T
            com[2,0] = 0.0;
            simulator.updateComPositionInViewer(com);
        if(i%100==0):
            ctrl.com.recompute(i);
            com = np.matrix(ctrl.com.value).T
            v = np.matrix(device.velocity.value).T;
            dv = np.matrix(ctrl.dv_des.value).T;
            print "t=%.3f dv=%.1f v=%.1f com=" % (t, norm(dv), norm(v)), com.T,
            
        if(i==2):
            ctrl_manager = ControlManager("ctrl_man");
            ctrl_manager.resetProfiler();
        t += dt;
    
    return (simulator, ctrl);

def create_graph(dt=0.001, delay=0.01):
    dt = conf.dt;
    q0 = conf.q0_urdf;
    v0 = conf.v0;
    simulator  = Simulator('hrp2_sim', q0, v0, conf.fMin, conf.mu, dt, conf.model_path, conf.urdfFileName);
    simulator.ENABLE_TORQUE_LIMITS = conf.FORCE_TORQUE_LIMITS;
    simulator.ENABLE_FORCE_LIMITS = conf.ENABLE_FORCE_LIMITS;
    simulator.ENABLE_JOINT_LIMITS = conf.FORCE_JOINT_LIMITS;
    simulator.ACCOUNT_FOR_ROTOR_INERTIAS = conf.ACCOUNT_FOR_ROTOR_INERTIAS;
    simulator.VIEWER_DT = conf.DT_VIEWER;
    simulator.CONTACT_FORCE_ARROW_SCALE = 2e-3;
    simulator.verb=0;
    robot = simulator.r;
    
    device          = create_device(conf.q0_sot);

    from dynamic_graph.sot.core import Selec_of_vector
    joint_vel = Selec_of_vector("joint_vel");
    plug(device.velocity, joint_vel.sin);
    joint_vel.selec(6, 36);

    from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
    ff_locator = FreeFlyerLocator("ffLocator");
    plug(device.state,      ff_locator.base6d_encoders);
    plug(joint_vel.sout,    ff_locator.joint_velocities);
    ff_locator.init(conf.urdfFileName);

#    ff_locator      = create_free_flyer_locator(device, conf.urdfFileName);
#    traj_gen        = create_trajectory_generator(device, dt);
    traj_gen = JointTrajectoryGenerator("jtg");
    plug(device.state,             traj_gen.base6d_encoders);
    traj_gen.init(dt);
    
    com_traj_gen = NdTrajectoryGenerator("com_traj_gen");
    com_traj_gen.initial_value.value = to_tuple(robot.com(q0));
    com_traj_gen.init(dt,3);
        
#    estimator       = create_estimator(device, dt, delay, traj_gen);
#    torque_ctrl     = create_torque_controller(device, estimator);
#    pos_ctrl        = create_position_controller(device, estimator, dt, traj_gen);
    
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");
    plug(device.state,                 ctrl.q);

    plug(traj_gen.q,                        ctrl.posture_ref_pos);
    plug(traj_gen.dq,                       ctrl.posture_ref_vel);
    plug(traj_gen.ddq,                      ctrl.posture_ref_acc);
#    plug(estimator.contactWrenchRightSole,  ctrl.wrench_right_foot);
#    plug(estimator.contactWrenchLeftSole,   ctrl.wrench_left_foot);
#    plug(ctrl.tau_des,                      torque_ctrl.jointsTorquesDesired);
#    plug(ctrl.tau_des,                      estimator.tauDes);

    plug(com_traj_gen.x,   ctrl.com_ref_pos);
    plug(com_traj_gen.dx,  ctrl.com_ref_vel);
    plug(com_traj_gen.ddx, ctrl.com_ref_acc);
#    plug(ctrl.com, com_traj_gen.initial_value);
    
    rf_traj_gen = SE3TrajectoryGenerator("tg_rf");
    plug(ctrl.right_foot_pos, rf_traj_gen.initial_value);
    rf_traj_gen.init(dt);
    
    lf_traj_gen = SE3TrajectoryGenerator("tg_lf");
    plug(ctrl.left_foot_pos, lf_traj_gen.initial_value);
    lf_traj_gen.init(dt);
    
    plug(rf_traj_gen.x,   ctrl.rf_ref_pos);
    plug(rf_traj_gen.dx,  ctrl.rf_ref_vel);
    plug(rf_traj_gen.ddx, ctrl.rf_ref_acc);

    plug(lf_traj_gen.x,   ctrl.lf_ref_pos);
    plug(lf_traj_gen.dx,  ctrl.lf_ref_vel);
    plug(lf_traj_gen.ddx, ctrl.lf_ref_acc);

    ctrl.rotor_inertias.value = conf.ROTOR_INERTIAS;
    ctrl.gear_ratios.value = conf.GEAR_RATIOS;
    ctrl.contact_normal.value = (0.0, 0.0, 1.0);
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
    ctrl.kp_posture.value = 30*(conf.kp_posture,);
    ctrl.kd_posture.value = 30*(conf.kd_posture,);
    ctrl.kp_pos.value = 30*(0*conf.kp_pos,);
    ctrl.kd_pos.value = 30*(0*conf.kd_pos,);

    ctrl.w_com.value = conf.w_com;
    ctrl.w_feet.value = conf.w_feet;
    ctrl.w_forces.value = conf.w_forces;
    ctrl.w_posture.value = conf.w_posture;
    ctrl.w_base_orientation.value = conf.w_base_orientation;
    ctrl.w_torques.value = conf.w_torques;
    
    ctrl.init(dt, conf.urdfFileName);
    ctrl.active_joints.value = conf.active_joints;
    
#    ctrl_manager    = create_ctrl_manager(device, torque_ctrl, pos_ctrl, ctrl, estimator, dt);
#    plug(device.velocity,       torque_ctrl.jointsVelocities);    
    plug(device.velocity,       ctrl.v);
    plug(ctrl.dv_des,           device.control);
    return Bunch(simulator=simulator, device=device, ctrl=ctrl, ff_locator=ff_locator, 
                 traj_gen=traj_gen, rf_traj_gen=rf_traj_gen, lf_traj_gen=lf_traj_gen,
                 com_traj_gen=com_traj_gen);
    
def zmp_test(dt=0.001, delay=0.01):
    COM_DES_1 = (0.012, 0.1, 0.81);
    COM_DES_2 = (0.012, 0.0, 0.81);
    dt = conf.dt;
    ent = create_graph(dt, delay);
    robot = ent.simulator.r;
    nv = robot.model.nv;
    
    ent.ctrl.com_ref_pos.value = COM_DES_1;

    t = 0.0;
    v = mat_zeros(nv);
    dv = mat_zeros(nv);
    x_rf = robot.framePosition(robot.model.getFrameId('RLEG_JOINT5')).translation;
    x_lf = robot.framePosition(robot.model.getFrameId('LLEG_JOINT5')).translation;

    ent.simulator.viewer.addSphere('zmp_rf', 0.01, mat_zeros(3), mat_zeros(3), (0.8, 0.3, 1.0, 1.0), 'OFF');
    ent.simulator.viewer.addSphere('zmp_lf', 0.01, mat_zeros(3), mat_zeros(3), (0.8, 0.3, 1.0, 1.0), 'OFF');
    ent.simulator.viewer.addSphere('zmp', 0.01, mat_zeros(3), mat_zeros(3), (0.8, 0.8, 0.3, 1.0), 'OFF');

    for i in range(conf.MAX_TEST_DURATION):
#        if(norm(dv[6:24]) > 1e-8):
#            print "ERROR acceleration of blocked axes is not zero:", norm(dv[6:24]);
        
        ent.ff_locator.base6dFromFoot_encoders.recompute(i);
        ent.ff_locator.v.recompute(i);

        ent.device.increment(dt);
        
        
        if(i==1500):
            ent.ctrl.com_ref_pos.value = COM_DES_2;
        if(i%30==0):
            q = np.matrix(ent.device.state.value).T;
            q_urdf = config_sot_to_urdf(q);
            ent.simulator.viewer.updateRobotConfig(q_urdf);

            ent.ctrl.f_des_right_foot.recompute(i);
            ent.ctrl.f_des_left_foot.recompute(i);
            f_rf = np.matrix(ent.ctrl.f_des_right_foot.value).T
            f_lf = np.matrix(ent.ctrl.f_des_left_foot.value).T
            ent.simulator.updateContactForcesInViewer(['rf', 'lf'], 
                                                  [x_rf, x_lf], 
                                                  [f_rf, f_lf]);
                                                  
            ent.ctrl.zmp_des_right_foot.recompute(i);
            ent.ctrl.zmp_des_left_foot.recompute(i);
            ent.ctrl.zmp_des.recompute(i);
            zmp_rf = np.matrix(ent.ctrl.zmp_des_right_foot.value).T;
            zmp_lf = np.matrix(ent.ctrl.zmp_des_left_foot.value).T;
            zmp = np.matrix(ent.ctrl.zmp_des.value).T; 
            ent.simulator.viewer.updateObjectConfig('zmp_rf', (zmp_rf[0,0], zmp_rf[1,0], 0., 0.,0.,0.,1.));
            ent.simulator.viewer.updateObjectConfig('zmp_lf', (zmp_lf[0,0], zmp_lf[1,0], 0., 0.,0.,0.,1.));
            ent.simulator.viewer.updateObjectConfig('zmp',    (zmp[0,0],    zmp[1,0], 0., 0.,0.,0.,1.));

            ent.ctrl.com.recompute(i);
            com = np.matrix(ent.ctrl.com.value).T
            com[2,0] = 0.0;
            ent.simulator.updateComPositionInViewer(com);
        if(i%100==0):
            ent.ctrl.com.recompute(i);
            com = np.matrix(ent.ctrl.com.value).T
            v = np.matrix(ent.device.velocity.value).T;
            dv = np.matrix(ent.ctrl.dv_des.value).T;
            ent.ctrl.zmp_des_right_foot_local.recompute(i);
            ent.ctrl.zmp_des_left_foot_local.recompute(i);
            ent.ctrl.zmp_des.recompute(i);
            zmp_rf = np.matrix(ent.ctrl.zmp_des_right_foot_local.value).T;
            zmp_lf = np.matrix(ent.ctrl.zmp_des_left_foot_local.value).T;
            zmp = np.matrix(ent.ctrl.zmp_des.value).T; 
            print "t=%.3f dv=%.1f v=%.1f com=" % (t, norm(dv), norm(v)), com.T;
#            print "zmp_lf", np.array([-f_lf[4,0], f_lf[3,0]])/f_lf[2,0], zmp_lf.T;
#            print "zmp_rf", np.array([-f_rf[4,0], f_rf[3,0]])/f_rf[2,0], zmp_rf.T, '\n';
#            print "Base pos (real)", device.state.value[:3];
#            print "Base pos (esti)", ff_locator.base6dFromFoot_encoders.value[:3], '\n';
#            print "Base velocity (real)", np.array(device.velocity.value[:6]);
#            print "Base velocity (esti)", np.array(ff_locator.v.value[:6]), '\n';
            
        if(i==2):
            ent.ctrl_manager = ControlManager("ctrl_man");
            ent.ctrl_manager.resetProfiler();
        t += dt;
        sleep(dt);
    
    return ent;

def one_foot_balance_test(dt=0.001, delay=0.01):
    COM_DES_1 = (0.012, 0.09, 0.85);
    T_CONTACT_TRANSITION = 1.8;
    CONTACT_TRANSITION_DURATION = 0.5;
    T_LIFT = T_CONTACT_TRANSITION + CONTACT_TRANSITION_DURATION;
    T_MOVE_DOWN = T_LIFT+1.0;
    ent = create_graph(dt, delay);
    robot = ent.simulator.r;
    
#    ent.ctrl.com_ref_pos.value = COM_DES_1;
    ent.com_traj_gen.move(0, COM_DES_1[0], T_LIFT);
    ent.com_traj_gen.move(1, COM_DES_1[1], T_LIFT);
    ent.com_traj_gen.move(2, COM_DES_1[2], T_LIFT);

    t = 0.0;
    x_rf = robot.framePosition(robot.model.getFrameId('RLEG_JOINT5')).translation;
    x_lf = robot.framePosition(robot.model.getFrameId('LLEG_JOINT5')).translation;
    tauMax = mat_zeros(NJ);

    for i in range(conf.MAX_TEST_DURATION):
        start = time.time();
        ent.ff_locator.base6dFromFoot_encoders.recompute(i);
        ent.ff_locator.v.recompute(i);

        ent.device.increment(dt);
        
        tau = np.matrix(ent.ctrl.tau_des.value).T;
        for j in range(NJ):
            if(abs(tau[j]) > tauMax[j]):
                tauMax[j] = abs(tau[j]);
        
        if(abs(t-T_CONTACT_TRANSITION)<0.5*dt):
            print "Remove right foot contact";
            ent.ctrl.removeRightFootContact(CONTACT_TRANSITION_DURATION); 
            
        if(abs(t-T_LIFT) < 0.5*dt):
            print "Lift foot"
            ent.ctrl.right_foot_pos.recompute(i);
            M_rf = ent.ctrl.right_foot_pos.value;
            ent.rf_traj_gen.move(2, M_rf[2]+0.1, T_MOVE_DOWN-T_LIFT-dt);
            
        if(abs(t-T_MOVE_DOWN)<0.5*dt):
            print "Move foot down"
            ent.rf_traj_gen.move(2, M_rf[2], 1.0);
            
        if(abs(t%0.05)<dt):
            q = np.matrix(ent.device.state.value).T;
            q_urdf = config_sot_to_urdf(q);
            ent.simulator.viewer.updateRobotConfig(q_urdf);

            ent.ctrl.f_des_right_foot.recompute(i);
            ent.ctrl.f_des_left_foot.recompute(i);
            f_rf = np.matrix(ent.ctrl.f_des_right_foot.value).T
            f_lf = np.matrix(ent.ctrl.f_des_left_foot.value).T
            ent.simulator.updateContactForcesInViewer(['rf', 'lf'], 
                                                  [x_rf, x_lf], 
                                                  [f_rf, f_lf]);
                                                  
            ent.ctrl.com.recompute(i);
            com = np.matrix(ent.ctrl.com.value).T
        
            v = np.matrix(ent.device.velocity.value).T;
            dv = np.matrix(ent.ctrl.dv_des.value).T;
            print "t=%.3f dv=%.1f v=%.1f com=" % (t, norm(dv), norm(v)), com.T, 
            print "com_ref", np.array(ent.com_traj_gen.x.value),
            print "f_rf %.1f"%f_rf[2,0].T, "f_lf %.1f"%f_lf[2,0].T;
            
            com[2,0] = 0.0;
            ent.simulator.updateComPositionInViewer(com);
            
        if(i==1):
            ent.ctrl_manager = ControlManager("ctrl_man");
            ent.ctrl_manager.resetProfiler();
        t += dt;
        stop = time.time();
        if(start-stop<dt):
            sleep(dt-start+stop);
    
    tauMax = joints_sot_to_urdf(tauMax);
    print "max torques requested at each joint:\n", tauMax.T;
    print "tauMax - tau =\n", robot.model.effortLimit[6:,0].T - tauMax.T;
    return ent;
    
if __name__=='__main__':
    np.set_printoptions(precision=3, suppress=True);
#    ent = zmp_test();
    ent = one_foot_balance_test(conf.dt);
    print ent.ctrl;
    pass;
    
