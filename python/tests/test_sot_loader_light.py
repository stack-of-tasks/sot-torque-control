# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import random
from dynamic_graph import plug
from dynamic_graph.sot.application.velocity.precomputed_tasks import *
from dynamic_graph.sot.core import seq_play
from dynamic_graph.ros import RosImport

def randTuple(size):
    v = ();
    for i in range(0,size-1):
        v = v + (random.random(),);
    return v;


def test_ros(robot):
    # SCRIPT PARAMETERS
    createRosTopics = 1;        # 1=true, 0=false
    nj              = 30;       # number of joints

    app = Application(robot);
    dq_des = nj*(0.0,);    
    app.robot.device.control.value = dq_des;

    if(createRosTopics==1):
        ros = RosImport('rosImport');
        ros.add('vector', 'robotStateRos', 'robotState');
        plug(robot.device.robotState, ros.robotStateRos);
        robot.device.after.addSignal('rosImport.trigger');
    
    return ros
