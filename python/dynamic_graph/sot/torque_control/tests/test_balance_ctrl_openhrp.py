# -*- coding: utf-8 -*-
"""
2014-2017, LAAS/CNRS
@author: Andrea Del Prete, Rohan Budhiraja
"""

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.torque_control.create_entities_utils import NJ
from dynamic_graph.sot.torque_control.utils.sot_utils import start_sot, Bunch
from dynamic_graph.ros import RosPublish
from dynamic_graph.sot.torque_control.create_entities_utils import create_topic
from dynamic_graph.sot.torque_control.main import main_v3

#from dynamic_graph.sot.torque_control.hrp2.sot_utils import config_sot_to_urdf, joints_sot_to_urdf
    
def get_sim_conf():
    import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_sim_conf as balance_ctrl_conf
    import dynamic_graph.sot.torque_control.hrp2.base_estimator_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.control_manager_sim_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.hrp2.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains_sim as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.hrp2.motors_parameters as motor_params
    conf = Bunch();
    conf.balance_ctrl              = balance_ctrl_conf;
    conf.base_estimator            = base_estimator_conf;
    conf.control_manager           = control_manager_conf;
    conf.force_torque_estimator    = force_torque_estimator_conf;
    conf.joint_torque_controller   = joint_torque_controller_conf;
    conf.pos_ctrl_gains            = pos_ctrl_gains;
    conf.motor_params              = motor_params;
    return conf;
    
def one_foot_balance_test(robot, use_real_vel=False, use_real_base_state=False):
    # BUILD THE STANDARD GRAPH
    conf = get_sim_conf();
    robot = main_v3(robot, startSoT=False, go_half_sitting=False, conf=conf);
    
    # BYPASS TORQUE CONTROLLER
    plug(robot.inv_dyn.tau_des,     robot.ctrl_manager.ctrl_torque);
    
    # CREATE SIGNALS WITH ROBOT STATE WITH CORRECT SIZE (36)
    from dynamic_graph.sot.core import Selec_of_vector
    robot.q = Selec_of_vector("q");
    plug(robot.device.robotState, robot.q.sin);
    robot.q.selec(0, NJ+6);
    plug(robot.q.sout,              robot.pos_ctrl.base6d_encoders);
    plug(robot.q.sout,              robot.traj_gen.base6d_encoders);
    plug(robot.q.sout,              robot.estimator_ft.base6d_encoders);
    plug(robot.q.sout,              robot.ctrl_manager.base6d_encoders);
    plug(robot.q.sout,              robot.torque_ctrl.base6d_encoders);
    
    robot.ros = RosPublish('rosPublish');
    robot.device.after.addDownsampledSignal('rosPublish.trigger',1);
    
    # BYPASS JOINT VELOCITY ESTIMATOR
    if(use_real_vel):
        robot.dq = Selec_of_vector("dq");
        plug(robot.device.robotVelocity, robot.dq.sin);
        robot.dq.selec(6, NJ+6);
        plug(robot.dq.sout,             robot.pos_ctrl.jointsVelocities);
        plug(robot.dq.sout,             robot.base_estimator.joint_velocities);

    # BYPASS BASE ESTIMATOR
    if(use_real_base_state):
        robot.v = Selec_of_vector("v");
        plug(robot.device.robotVelocity, robot.v.sin);
        robot.v.selec(0, NJ+6);
        plug(robot.q.sout,              robot.inv_dyn.q);
        plug(robot.v.sout,              robot.inv_dyn.v);
        robot.inv_dyn.v.value = (NJ+6)*(0.0,);
    
    start_sot();
    
    return robot;
    
if __name__=='__main__':
    np.set_printoptions(precision=2, suppress=True);
    ent = one_foot_balance_test(robot, dt=conf.dt);
