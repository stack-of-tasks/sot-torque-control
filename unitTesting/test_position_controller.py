from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.position_controller import *
from numpy import matrix, identity, zeros, eye, array, pi, ndarray, ones

# Instanciate the free flyer
pc = PositionController("pc_test")

q=zeros(36)
dq=zeros(30)
qRef=zeros(36)
dqRef=zeros(30)

# Setting the robot configuration
pc.base6d_encoders.value=q
pc.qRef.value=qRef

# Setting the robot velocities
pc.jointsVelocities.value=dq
pc.dqRef.value=dqRef

Kp=ones(30)
pc.Kp.value=Kp
Ki=ones(30)
pc.Ki.value=Ki
Kd=ones(30)
pc.Kd.value=Kd

# Initializing the entity.
pc.init(0.005,36)

pc.pwmDes.recompute(10)
pc.qError.recompute(10)

