# -*- coding: utf-8 -*-
"""
2014-2017, LAAS/CNRS
@author: Andrea Del Prete, Rohan Budhiraja
"""

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.torque_control.create_entities_utils import NJ
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.create_entities_utils import create_trajectory_generator, create_com_traj_gen, create_encoders
from dynamic_graph.sot.torque_control.create_entities_utils import create_imu_offset_compensation, create_estimators, create_imu_filter
from dynamic_graph.sot.torque_control.create_entities_utils import create_base_estimator, create_position_controller, create_torque_controller
from dynamic_graph.sot.torque_control.create_entities_utils import create_balance_controller, create_ctrl_manager
from dynamic_graph.sot.torque_control.utils.sot_utils import start_sot

import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_sim_conf as balance_ctrl_conf
import dynamic_graph.sot.torque_control.hrp2.base_estimator_conf as base_estimator_conf
import dynamic_graph.sot.torque_control.hrp2.control_manager_sim_conf as control_manager_conf
import dynamic_graph.sot.torque_control.hrp2.force_torque_estimator_conf as force_torque_estimator_conf
import dynamic_graph.sot.torque_control.hrp2.joint_torque_controller_conf as joint_torque_controller_conf
import dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains_sim as pos_ctrl_gains
import dynamic_graph.sot.torque_control.hrp2.motors_parameters as motor_params

#from dynamic_graph.sot.torque_control.hrp2.sot_utils import config_sot_to_urdf, joints_sot_to_urdf
    
def one_foot_balance_test(robot, dt=0.001, delay=0.01, use_real_vel=False, use_real_base_state=False):
    # BUILD THE STANDARD GRAPH
    robot.traj_gen        = create_trajectory_generator(robot.device, dt);
    robot.com_traj_gen    = create_com_traj_gen(dt);
    robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf");
    robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf");
    robot.rf_traj_gen.init(dt);
    robot.lf_traj_gen.init(dt);
    
    robot.encoders                              = create_encoders(robot);
    robot.imu_offset_compensation               = create_imu_offset_compensation(robot, dt);
    (robot.estimator_ft, robot.estimator_kin)   = create_estimators(robot, force_torque_estimator_conf, motor_params, dt, delay);
    robot.imu_filter                            = create_imu_filter(robot, dt);
    robot.base_estimator                        = create_base_estimator(robot, dt, balance_ctrl_conf.urdfFileName, base_estimator_conf);

    robot.pos_ctrl        = create_position_controller(robot, pos_ctrl_gains, dt);
    robot.torque_ctrl     = create_torque_controller(robot, joint_torque_controller_conf, motor_params, dt);
    robot.inv_dyn         = create_balance_controller(robot, balance_ctrl_conf, dt);
    robot.ctrl_manager    = create_ctrl_manager(robot, control_manager_conf, dt);
    
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
