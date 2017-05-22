from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.position_controller import *
from numpy import matrix, identity, zeros, eye, array, pi, ndarray

# Instanciate the free flyer
pc = PositionController("pc_test")

q=zeros(36)
dq=zeros(30)

# Setting the robot configuration
pc.base6d_encoders.value=q
# Setting the robot velocities
pc.jointsVelocities.value=dq

# Initializing the entity.
pc.init(0.005,36)

