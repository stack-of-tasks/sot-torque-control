from dynamic_graph.sot.torque_control.control_manager import *

from dynamic_graph.sot.torque_control.free_flyer_locator import *

ffl = FreeFlyerLocator("ffl_test")

ffl.setJointsUrdfToSot((12,13,14,15,23,24,25,26,27,28,29,16,17,18,19,20,21,22,6,7,8,9,10,11,0,1,2,3,4,5))

v=ffl.getJointsUrdfToSot()
v


