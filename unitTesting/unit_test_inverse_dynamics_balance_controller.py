from dynamic_graph.sot.torque_control.tests.test_control_manager import cm
from dynamic_graph.sot.torque_control.tests.robot_data_test import initRobotData
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import *
from numpy import matrix, identity, zeros, eye, array, pi, ndarray, ones

# Instanciate the free flyer
idbc = InverseDynamicsBalanceController("idbc_test")

q=zeros(initRobotData.nbJoints+6)
dq=zeros(initRobotData.nbJoints)
ddq=zeros(initRobotData.nbJoints)
qRef=zeros(initRobotData.nbJoints+6)
dqRef=zeros(initRobotData.nbJoints)
ddqRef=zeros(initRobotData.nbJoints)

# Setting the robot configuration
idbc.posture_ref_pos.value=qRef
idbc.posture_ref_vel.value=dqRef
idbc.posture_ref_acc.value=ddqRef

# Com ref
comRefPos=zeros(3)
comRefVel=zeros(3)
comRefAcc=zeros(3)
idbc.com_ref_pos.value=comRefPos
idbc.com_ref_vel.value=comRefVel
idbc.com_ref_acc.value=comRefAcc

# Right foot
rfRefPos=zeros(12)
rfRefPos[0]=1.0
rfRefPos[4]=1.0
rfRefPos[8]=1.0

rfRefVel=zeros(6)
rfRefAcc=zeros(6)
idbc.rf_ref_pos.value=rfRefPos
idbc.rf_ref_vel.value=rfRefVel
idbc.rf_ref_acc.value=rfRefAcc

# Left foot
lfRefPos=zeros(12)
lfRefPos[0]=1.0
lfRefPos[4]=1.0
lfRefPos[8]=1.0
lfRefVel=zeros(6)
lfRefAcc=zeros(6)
idbc.lf_ref_pos.value=lfRefPos
idbc.lf_ref_vel.value=lfRefVel
idbc.lf_ref_acc.value=lfRefAcc

kpConstraints=zeros(6)
kdConstraints=zeros(6)
idbc.kp_constraints.value= kpConstraints

kpCom=zeros(3)
kdCom=zeros(3)
idbc.kp_com.value=kpCom


