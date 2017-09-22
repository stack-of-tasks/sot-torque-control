# -*- coding: utf-8 -*-
"""
Created on Thu Aug 31 16:04:18 2017

@author: adelpret
"""

import pinocchio as se3
import numpy as np
from pinocchio import RobotWrapper
from conversion_utils import config_sot_to_urdf, joints_sot_to_urdf, velocity_sot_to_urdf
from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
from dynamic_graph.sot.torque_control.create_entities_utils import create_ctrl_manager
import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_sim_conf as balance_ctrl_conf
import dynamic_graph.sot.torque_control.hrp2.control_manager_sim_conf as control_manager_conf
from dynamic_graph.sot.torque_control.tests.robot_data_test import initRobotData

np.set_printoptions(precision=3, suppress=True, linewidth=100);
    
def create_balance_controller(dt, q, conf, robot_name='robot'):
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");
    ctrl.q.value = tuple(q);
    ctrl.v.value = (NJ+6)*(0.0,);

    ctrl.wrench_right_foot.value = 6*(0.0,);
    ctrl.wrench_left_foot.value = 6*(0.0,);

    ctrl.posture_ref_pos.value = tuple(q[6:]);
    ctrl.posture_ref_vel.value = NJ*(0.0,);
    ctrl.posture_ref_acc.value = NJ*(0.0,);
    ctrl.com_ref_pos.value = (0., 0., 0.8);
    ctrl.com_ref_vel.value = 3*(0.0,);
    ctrl.com_ref_acc.value = 3*(0.0,);

#    ctrl.rotor_inertias.value = np.array(conf.ROTOR_INERTIAS);
#    ctrl.gear_ratios.value = conf.GEAR_RATIOS;
    ctrl.rotor_inertias.value = tuple([g*g*r for (g,r) in zip(conf.GEAR_RATIOS, conf.ROTOR_INERTIAS)])
    ctrl.gear_ratios.value = NJ*(1.0,);
    ctrl.contact_normal.value = conf.FOOT_CONTACT_NORMAL;
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
    
    ctrl.active_joints.value = NJ*(1,);
    
    ctrl.init(dt, robot_name);
    return ctrl;

print "*** UNIT TEST FOR INVERSE-DYNAMICS-BALANCE-CONTROLLER (IDBC) ***"
print "This test computes the torques using the IDBC and compares them with"
print "the torques computed using the desired joint accelerations and contact"
print "wrenches computed by the IDBC. The two values should be identical."
print "Some small differences are expected due to the precision loss when"
print "Passing the parameters from python to c++."
print "However, none of the following values should be larger than 1e-3.\n"

N_TESTS = 100
dt = 0.001;    
NJ = initRobotData.nbJoints
# robot configuration
q_sot = np.array([-0.0027421149619457344, -0.0013842807952574399, 0.6421082804660067, 
                   -0.0005693871512031474, -0.0013094048521806974, 0.0028568508070167, 
                   -0.0006369040657361668, 0.002710094953239396, -0.48241992906618536, 0.9224570746372157, -0.43872624301275104, -0.0021586727954009096, 
                   -0.0023395862060549863, 0.0031045906573987617, -0.48278188636903313, 0.9218508861779927, -0.4380058166724791, -0.0025558837738616047, 
                   -0.012985322450541008, 0.04430420221275542, 0.37027327677517635, 1.4795064165303056, 
                   0.20855551221055582, -0.13188842278441873, 0.005487207370709895, -0.2586657542648506, 2.6374918629921953, -0.004223605878088189, 0.17118034021053144, 0.24171737354070008, 0.11594430024547904, -0.05264225067057105, -0.4691871937149223, 0.0031522040623960016, 0.011836097472447007, 0.18425595002313025]);

ctrl_manager = create_ctrl_manager(control_manager_conf, dt);
ctrl = create_balance_controller(dt, q_sot, balance_ctrl_conf);
robot = RobotWrapper(initRobotData.testRobotPath, [], se3.JointModelFreeFlyer())
index_rf = robot.index('RLEG_JOINT5');
index_lf = robot.index('LLEG_JOINT5');

Md = np.matrix(np.zeros((NJ+6,NJ+6)));
gr = joints_sot_to_urdf(balance_ctrl_conf.GEAR_RATIOS);
ri = joints_sot_to_urdf(balance_ctrl_conf.ROTOR_INERTIAS);
for i in range(NJ):
    Md[6+i,6+i] = ri[i] * gr[i] * gr[i];
    
for i in range(N_TESTS):
    q_sot += 0.001*np.random.random(NJ+6);
    v_sot = np.random.random(NJ+6);
    q_pin = np.matrix(config_sot_to_urdf(q_sot));
    v_pin = np.matrix(velocity_sot_to_urdf(v_sot));
    
    ctrl.q.value = tuple(q_sot);
    ctrl.v.value = tuple(v_sot);
    ctrl.tau_des.recompute(i);
    tau_ctrl = joints_sot_to_urdf(np.array(ctrl.tau_des.value));
    
    ctrl.dv_des.recompute(i);
    dv = velocity_sot_to_urdf(np.array(ctrl.dv_des.value));    
    M = Md + robot.mass(q_pin);
    h = robot.bias(q_pin, v_pin);
    
    ctrl.f_des_right_foot.recompute(i);
    ctrl.f_des_left_foot.recompute(i);
    f_rf = np.matrix(ctrl.f_des_right_foot.value).T;
    f_lf = np.matrix(ctrl.f_des_left_foot.value).T;
    J_rf = robot.jacobian(q_pin, index_rf);
    J_lf = robot.jacobian(q_pin, index_lf);
    
    tau_pin = M*np.matrix(dv).T + h - J_rf.T * f_rf - J_lf.T * f_lf;
    
#    ctrl.M.recompute(i);
#    M_ctrl = np.array(ctrl.M.value);
    
    print "norm(tau_ctrl-tau_pin) = %.4f"% np.linalg.norm(tau_ctrl - tau_pin[6:,0].T);
    print "norm(tau_pin[:6])      = %.4f"% np.linalg.norm(tau_pin[:6]);
#    print "q_pin:\n", q_pin;
#    print "tau_pin:\n", tau_pin[6:,0].T, "\n";
#    print "tau ctrl:\n", tau_ctrl.T, "\n";
#    print "dv = ", np.linalg.norm(dv);
#    print "f_rf:", f_rf.T, "\n";
#    print "f_lf:", f_lf.T, "\n";
#    print "h:", h.T, "\n";    
#    M_err = M-M_ctrl
#    print "M-M_ctrl = ", M_err.diagonal(), "\n"
#    for j in range(NJ+6):
#        print M_err[j,:];
