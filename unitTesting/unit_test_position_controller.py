from dynamic_graph.sot.torque_control.tests.test_control_manager import cm
from dynamic_graph.sot.torque_control.tests.robot_data_test import initRobotData
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.position_controller import *
from numpy import matrix, identity, zeros, eye, array, pi, ndarray, ones

# Instanciate the free flyer
pc = PositionController("pc_test")

q=zeros(initRobotData.nbJoints+6)
dq=zeros(initRobotData.nbJoints)
qRef=zeros(initRobotData.nbJoints)
dqRef=zeros(initRobotData.nbJoints)

# Setting the robot configuration
pc.base6d_encoders.value=q
pc.qRef.value=qRef

# Setting the robot velocities
pc.jointsVelocities.value=dq
pc.dqRef.value=dqRef

Kp=ones(initRobotData.nbJoints)
pc.Kp.value=Kp
Ki=ones(initRobotData.nbJoints)
pc.Ki.value=Ki
Kd=ones(initRobotData.nbJoints)
pc.Kd.value=Kd

# Initializing the entity.
pc.init(cm.controlDT, "control-manager-robot"); #initRobotData.robotRef)

pc.pwmDes.recompute(10)
pc.qError.recompute(10)

