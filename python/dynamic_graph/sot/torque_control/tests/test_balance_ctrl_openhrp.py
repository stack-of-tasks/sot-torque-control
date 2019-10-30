# -*- coding: utf-8 -*-
"""
2014-2017, LAAS/CNRS
@author: Andrea Del Prete, Rohan Budhiraja
"""

from time import sleep

from dynamic_graph import plug
from dynamic_graph.ros import RosPublish
from dynamic_graph.sot.core import Selec_of_vector
from dynamic_graph.sot.torque_control.create_entities_utils import NJ
from dynamic_graph.sot.torque_control.main import main_v3
from dynamic_graph.sot.torque_control.utils.sot_utils import Bunch, start_sot

# from dynamic_graph.sot.torque_control.hrp2.sot_utils import config_sot_to_urdf, joints_sot_to_urdf


def get_sim_conf():
    import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_sim_conf as balance_ctrl_conf
    import dynamic_graph.sot.torque_control.hrp2.base_estimator_sim_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.control_manager_sim_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.hrp2.current_controller_sim_conf as current_controller_conf
    import dynamic_graph.sot.torque_control.hrp2.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains_sim as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.hrp2.motors_parameters as motor_params
    import dynamic_graph.sot.torque_control.hrp2.admittance_ctrl_conf as admittance_ctrl_conf
    conf = Bunch()
    conf.adm_ctrl = admittance_ctrl_conf
    conf.balance_ctrl = balance_ctrl_conf
    conf.base_estimator = base_estimator_conf
    conf.control_manager = control_manager_conf
    conf.current_ctrl = current_controller_conf
    conf.force_torque_estimator = force_torque_estimator_conf
    conf.joint_torque_controller = joint_torque_controller_conf
    conf.pos_ctrl_gains = pos_ctrl_gains
    conf.motor_params = motor_params
    return conf


def test_balance_ctrl_openhrp(robot, use_real_vel=True, use_real_base_state=False, startSoT=True):
    # BUILD THE STANDARD GRAPH
    conf = get_sim_conf()
    robot = main_v3(robot, startSoT=False, go_half_sitting=False, conf=conf)

    # force current measurements to zero
    robot.ctrl_manager.i_measured.value = NJ * (0.0, )
    robot.current_ctrl.i_measured.value = NJ * (0.0, )
    robot.filters.current_filter.x.value = NJ * (0.0, )

    # BYPASS TORQUE CONTROLLER
    plug(robot.inv_dyn.tau_des, robot.ctrl_manager.ctrl_torque)

    # CREATE SIGNALS WITH ROBOT STATE WITH CORRECT SIZE (36)
    robot.q = Selec_of_vector("q")
    plug(robot.device.robotState, robot.q.sin)
    robot.q.selec(0, NJ + 6)
    plug(robot.q.sout, robot.pos_ctrl.base6d_encoders)
    plug(robot.q.sout, robot.traj_gen.base6d_encoders)
    plug(robot.q.sout, robot.estimator_ft.base6d_encoders)

    robot.ros = RosPublish('rosPublish')
    robot.device.after.addDownsampledSignal('rosPublish.trigger', 1)

    # BYPASS JOINT VELOCITY ESTIMATOR
    if (use_real_vel):
        robot.dq = Selec_of_vector("dq")
        plug(robot.device.robotVelocity, robot.dq.sin)
        robot.dq.selec(6, NJ + 6)
        plug(robot.dq.sout, robot.pos_ctrl.jointsVelocities)
        plug(robot.dq.sout, robot.base_estimator.joint_velocities)
        plug(robot.device.gyrometer, robot.base_estimator.gyroscope)

    # BYPASS BASE ESTIMATOR
    robot.v = Selec_of_vector("v")
    plug(robot.device.robotVelocity, robot.v.sin)
    robot.v.selec(0, NJ + 6)
    if (use_real_base_state):
        plug(robot.q.sout, robot.inv_dyn.q)
        plug(robot.v.sout, robot.inv_dyn.v)

    if (startSoT):
        start_sot()
        # RESET FORCE/TORQUE SENSOR OFFSET
        sleep(10 * robot.timeStep)
        robot.estimator_ft.setFTsensorOffsets(24 * (0.0, ))

    return robot
