# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete

Test new version of SoTHRP2Device, which implements a new method
setControlInputType taking a string that has to be either "position"
or "velocity" or "acceleration".
"""

import random
from dynamic_graph import plug
from dynamic_graph.sot.hrp2.sot_hrp2_device import *
#dir()

# create an instance of the device
device = Device('HRP2LAAS')
nj = 25;
q     = (nj+6) * (random.random(),);
q_des = nj * (random.random(),);
device.resize(nj+6);
device.set(q);
print "\nInitial state = "+repr(q);
print "\nInitial vel   = "+repr(device.velocity.value);

device.control.value = q_des;
print "\nControl = "+repr(device.control.value)
#(-1.5, -1.6, 1.4, 0.3, -0.5, -1.3,    3, -1.8, 1.5, 0.3, -0.5, -1.3)
#(-0.2, -0.5, 1.7, 0.3, -0.5, -1.3,    2.8, -1.2, 1.7, 0.3, 0.5, -1.7)
#
#device.setControlInputType('position');
#device.increment(0.01);
#print "\nFinal state: "+repr(device.state.value);
#
#device.setControlInputType('velocity');
#device.set(q);
for i in range(1,1000):
    device.increment(0.01);
print "\nFinal state: "+repr(device.state.value);
#    
#device.setControlInputType('acceleration');
#device.set(q);
#for i in range(1,1000):
#    device.increment(0.01);
#print "\nFinal state: "+repr(device.state.value);
#
