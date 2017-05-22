from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.control_manager import *
from dynamic_graph.sot.torque_control.free_flyer_locator import *
from numpy import matrix, identity, zeros, eye, array, pi, ndarray

# Instanciate the free flyer
ffl = FreeFlyerLocator("ffl_test")

# Mapping with urdf to sot
ffl.setJointsUrdfToSot((12,13,14,15,23,24,25,26,27,28,29,16,17,18,19,20,21,22,6,7,8,9,10,11,0,1,2,3,4,5))

# Check if the map from URDF to SOT is done properly.
v=ffl.getJointsUrdfToSot()
v

# Initialize FFL
ffl.init("/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14_reduced.urdf","RLEG_JOINT5","LLEG_JOINT5")

# TODO : Set the value of the encoders.

q=zeros(36)
dq=zeros(30)
ffl.base6d_encoders.value = q
ffl.joint_velocities.value = dq
ffl.freeflyer_aa.recompute(100)


