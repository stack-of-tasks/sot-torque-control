from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.position_controller import *
from numpy import matrix, identity, zeros, eye, array, pi, ndarray

# Instanciate the free flyer
pc = PositionController("pc_test")

q=zeros(36)

pc.base6d_encoders.value=[36](0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Mapping with urdf to sot
pc.init(0.005,36)

