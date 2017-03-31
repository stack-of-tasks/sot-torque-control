# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import random
from dynamic_graph import plug
from dynamic_graph.sot.torque_control.hrp2_device_pos_ctrl import HRP2DevicePosCtrl
#from dynamic_graph.sot.core.robot_simu import RobotSimu


# create an instance of the device
device = HRP2DevicePosCtrl("hrp2");
#device = RobotSimu("hrp2");
nj = 25;
q     = (nj+6) * (random.random(),);
q_des = nj * (random.random(),);
device.resize(nj+6);
device.set(q);
print "\nInitial state = "+repr(q);
print "\nInitial vel   = "+repr(device.velocity.value);
device.kp.value = nj * (1,)
device.kd.value = nj * (2*pow(device.kp.value[0],0.5),);

device.control.value = q_des;
print "\nControl = "+repr(device.control.value)
#(-1.5, -1.6, 1.4, 0.3, -0.5, -1.3,    3, -1.8, 1.5, 0.3, -0.5, -1.3)
#(-0.2, -0.5, 1.7, 0.3, -0.5, -1.3,    2.8, -1.2, 1.7, 0.3, 0.5, -1.7)

for i in range(1,1000):
    device.increment(0.01);
    
print "\nFinal state:"
print device.state.value
