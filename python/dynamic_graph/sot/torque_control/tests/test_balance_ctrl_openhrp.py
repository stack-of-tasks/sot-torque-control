# -*- coding: utf-8 -*-
"""
2014-2017, LAAS/CNRS
@author: Andrea Del Prete, Rohan Budhiraja
"""

import numpy as np
from numpy.linalg import norm
from dynamic_graph import plug
from dynamic_graph.sot.torque_control.create_entities_utils import NJ
from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
#from dynamic_graph.sot.torque_control.hrp2_device_torque_ctrl import HRP2DeviceTorqueCtrl
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.create_entities_utils import *
from dynamic_graph.sot.torque_control.utils.sot_utils import *
from time import sleep
import time

import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_sim_conf as conf
from dynamic_graph.sot.torque_control.hrp2.sot_utils import config_sot_to_urdf, joints_sot_to_urdf
from dynamic_graph.sot.torque_control.numerical_difference import NumericalDifference
from pinocchio.utils import zero as mat_zeros
from pinocchio import Quaternion
import pinocchio as se3
from pinocchio.rpy import rpyToMatrix

def to_tuple(x):
    return tuple(np.asarray(x).squeeze());

def create_graph(dt=0.001, delay=0.01):
    dt = conf.dt;
    q0 = conf.q0_urdf;
    v0 = conf.v0;
    device = robot.device
    device.resize(36)
    from dynamic_graph.sot.core import Selec_of_vector
    joint_state = Selec_of_vector("joint_state");
    plug(device.robotState, joint_state.sin);
    joint_state.selec(0, 36);

    joint_vel = Selec_of_vector("joint_vel");
    plug(device.robotVelocity, joint_vel.sin);
    joint_vel.selec(0, 36);

    #from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
    #ff_locator = FreeFlyerLocator("ffLocator");
    #plug(device.state,      ff_locator.base6d_encoders);
    #plug(joint_vel.sout,    ff_locator.joint_velocities);
    #ff_locator.init(conf.urdfFileName);

#    ff_locator      = create_free_flyer_locator(device, conf.urdfFileName);
#    traj_gen        = create_trajectory_generator(device, dt);
    traj_gen = JointTrajectoryGenerator("jtg");
    #plug(joint_state.sout,             traj_gen.base6d_encoders);
    traj_gen.base6d_encoders.value = conf.q0_sot;
    traj_gen.init(dt);
    
    com_traj_gen = NdTrajectoryGenerator("com_traj_gen");
  
    com_traj_gen.initial_value.value = to_tuple(se3.centerOfMass(robot.pinocchioModel, robot.pinocchioData, q0));
    com_traj_gen.init(dt,3);
        
#    estimator       = create_estimator(device, dt, delay, traj_gen);
#    torque_ctrl     = create_torque_controller(device, estimator);
#    pos_ctrl        = create_position_controller(device, estimator, dt, traj_gen);
    
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");
    plug(joint_state.sout,                 ctrl.q);

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
    ctrl.f_max_right_foot.value = conf.fMax;
    ctrl.f_max_left_foot.value = conf.fMax;
    ctrl.mu.value = conf.mu[0];
    ctrl.weight_contact_forces.value = (1e2, 1e2, 1e0, 1e3, 1e3, 1e3);
    ctrl.kp_com.value = 3*(conf.kp_com,);
    ctrl.kd_com.value = 3*(conf.kd_com,);
    ctrl.kp_constraints.value = 6*(conf.kp_constr,);
    ctrl.kd_constraints.value = 6*(conf.kd_constr,);
    ctrl.kp_feet.value = 6*(conf.kp_feet,);
    ctrl.kd_feet.value = 6*(conf.kd_feet,);
    ctrl.kp_posture.value = conf.kp_posture
    ctrl.kd_posture.value = conf.kd_posture
    ctrl.kp_pos.value = conf.kp_pos;
    ctrl.kd_pos.value = conf.kd_pos;

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

    #d_dt = NumericalDifference("state_derivative")
    #d_dt.init(dt,NJ+6,15*dt)
    #plug(joint_state.sout, d_dt.x)


    #from dynamic_graph.sot.core import Selec_of_vector
    #joint_vel_ddt = Selec_of_vector("joint_vel_ddt");
    #plug(d_dt.dx, joint_veldevice.robotState, joint_state.sin);

    #plug(d_dt.dx,       ctrl.v)




    plug(joint_vel.sout,       ctrl.v);
    #ctrl.v.value = 36*(0.,)
    plug(ctrl.tau_des,           device.control);
    return Bunch(device=device, ctrl=ctrl, traj_gen=traj_gen, rf_traj_gen=rf_traj_gen, lf_traj_gen=lf_traj_gen,
                 com_traj_gen=com_traj_gen);#, derivator = d_dt);
    
def one_foot_balance_test(dt=0.001, delay=0.01):
    COM_DES_1 = (0.012, 0.09, 0.85);
    T_CONTACT_TRANSITION = 1.8;
    CONTACT_TRANSITION_DURATION = 0.5;
    T_LIFT = T_CONTACT_TRANSITION + CONTACT_TRANSITION_DURATION + 10.0;
    T_MOVE_DOWN = T_LIFT+1.0;
    ent = create_graph(dt);
    log_file.write("Starting SoT \n")
    start_sot()
    log_file.write("SoT started \n")
    log_file.write(str(dt))

    ent.com_traj_gen.move(0, COM_DES_1[0], T_CONTACT_TRANSITION);
    ent.com_traj_gen.move(1, COM_DES_1[1], T_CONTACT_TRANSITION);
    ent.com_traj_gen.move(2, COM_DES_1[2], T_CONTACT_TRANSITION);

    start_time = ent.ctrl.tau_des.time;
    log_file.write("Control start time")
    log_file.write(str(start_time))
    log_file.write("\n")
    f_rleg = robot.pinocchioModel.frames[robot.pinocchioModel.getFrameId('RLEG_JOINT5')]
    f_lleg = robot.pinocchioModel.frames[robot.pinocchioModel.getFrameId('LLEG_JOINT5')]
    x_rf = robot.pinocchioData.oMi[f_rleg.parent].act(f_rleg.placement).translation;
    x_lf = robot.pinocchioData.oMi[f_lleg.parent].act(f_lleg.placement).translation;
    tauMax = mat_zeros(NJ);
    
    tau    = mat_zeros((NJ, conf.MAX_TEST_DURATION));
    dv     = mat_zeros((NJ+6, conf.MAX_TEST_DURATION));
    f_rf   = mat_zeros((6, conf.MAX_TEST_DURATION));
    f_lf   = mat_zeros((6, conf.MAX_TEST_DURATION));
    t = 0.0
    while(True):
        start = time.time();
        current_time = ent.ctrl.tau_des.time;
        ent.ctrl.com.recompute(current_time);
        t = (ent.ctrl.tau_des.time - start_time)*dt
        log_file.write("Current running time")
        log_file.write(str(t))
        log_file.write("\n")
        tau[:,i] = np.matrix(ent.ctrl.tau_des.value).T;

        #log_file.write("NORM ERROR BETWEEN COM_DES AND US")
        #log_file.write(str(norm(np.array(ent.ctrl.com.value)[0:2] - np.array(COM_DES_1)[0:2])))
        #log_file.write("\n")
        log_file.write("Head Torque input")
        log_file.write(str(ent.ctrl.tau_des.value[14:16]))
        log_file.write("\n")

        for j in range(NJ):
            if(abs(tau[j,i]) > tauMax[j]):
                tauMax[j] = abs(tau[j,i]);
                
        if(i>0 and norm(tau[:,i]-tau[:,i-1]) > 10.0):
          pass
          #log_file.write("t=%.3f Joint torque variation is too large: "%t, norm(tau[:,i]-tau[:,i-1]))
        ent.ctrl.f_des_right_foot.recompute(current_time);
        ent.ctrl.f_des_left_foot.recompute(current_time);
        f_rf[:,i] = np.matrix(ent.ctrl.f_des_right_foot.value).T
        f_lf[:,i] = np.matrix(ent.ctrl.f_des_left_foot.value).T
        
        #if(norm(np.array(ent.ctrl.com.value) - np.array(COM_DES_1))<0.01):
        if(norm(np.array(ent.ctrl.com.value)[0:2] - np.array(COM_DES_1)[0:2])<0.001):
            log_file.write("t=%.3f Remove right foot contact"%t)
            ent.ctrl.removeRightFootContact(CONTACT_TRANSITION_DURATION); 
            break
            
        if(False):
            # # if(abs(t-T_LIFT) < 0.5*dt):
            log_file.write("t=%.3f Lift foot"%t)
            ent.ctrl.right_foot_pos.recompute(i);
            M_rf = ent.ctrl.right_foot_pos.value;
            ent.rf_traj_gen.move(2, M_rf[2]+0.1, T_MOVE_DOWN-T_LIFT-dt);
            
        if(False):
            #if(abs(t-T_MOVE_DOWN)<0.5*dt):
            log_file.write("t=%.3f Move foot down"%t)
            ent.rf_traj_gen.move(2, M_rf[2], 1.0);

        #    com = np.matrix(ent.ctrl.com.value).T
        #    com[2,0] = 0.0;

        if(i==1):
            ent.ctrl_manager = ControlManager("ctrl_man");
            ent.ctrl_manager.resetProfiler();
            #t += dt;
        stop = time.time();
        #if(start-stop<dt):
        #    sleep(dt-start+stop);
        while(ent.ctrl.tau_des.time == current_time):
          pass
    
    tauMax_urdf = joints_sot_to_urdf(tauMax);
    print "max torques requested at each joint:\n", tauMax_urdf.T;
    print "tauMax - tau =\n", robot.pinocchioModel.effortLimit[6:,0].T - tauMax_urdf.T;
    return ent;
    
if __name__=='__main__':
    log_file = open("/tmp/message.log","w")
    log_file.write("Starting main \n")
    dt = conf.dt
    delay = 0.001
    np.set_printoptions(precision=2, suppress=True);
    ent = one_foot_balance_test(conf.dt);
    #print ent.ctrl;
