# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import random

from dynamic_graph import plug

# from dynamic_graph.sot.torque_control.force_torque_estimator import ForceTorqueEstimator
from dynamic_graph.ros import RosExport
from dynamic_graph.sot.application.velocity.precomputed_tasks import Application


def randTuple(size):
    v = ()
    for i in range(0, size - 1):
        v = v + (random.random(),)
    return v


def test_ros(robot):
    # SCRIPT PARAMETERS
    # j = 0
    # index of joint under analysis
    # N = 300
    # test duration (in number of timesteps)
    # dt = 0.001
    # time step
    # estimationDelay = 10
    # delay introduced by the estimation [number of time steps]
    createRosTopics = 1
    # 1=true, 0=false

    # CONSTANTS
    nj = 30
    # number of joints
    # rad2deg = 180 / 3.14

    app = Application(robot)
    dq_des = nj * (0.0,)

    app.robot.device.control.value = dq_des

    if createRosTopics == 1:
        ros = RosExport("rosExport")
        ros.add("vector", "robotStateRos", "state")
        plug(robot.device.state, ros.robotStateRos)

    #        robot.device.after.addSignal('rosExport.robotStateRos')

    return ros
