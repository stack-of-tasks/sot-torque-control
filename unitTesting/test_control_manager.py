from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.control_manager import *
from dynamic_graph.sot.torque_control.tests.robot_data_test import testRobotPath,controlDT,maxCurrent,mapJointNameToID
from numpy import matrix, identity, zeros, eye, array, pi, ndarray, ones

# Instanciate the free flyer
cm = ControlManager("cm_test")

q=zeros(36)
dq=zeros(30)
bemfFactor=ones(30)
max_current=30.0*ones(30)
max_tau = 100.0*ones(30)
percentageDriverDeadZoneCompensation = 20.0*ones(30)
signWindowsFilterSize = ones(30)
tau = 100.0 * ones(30)
tau_predicted = 110.0 * ones(30)
pwmDes = 100.0 *ones(30)
currentDes = 100.0 *ones(30)

# Setting the robot configuration
cm.base6d_encoders.value=q
cm.dq.value=dq
cm.bemfFactor.value = bemfFactor
cm.max_current.value = max_current
cm.max_tau.value= max_tau
cm.percentageDriverDeadZoneCompensation.value = percentageDriverDeadZoneCompensation
cm.signWindowsFilterSize.value = signWindowsFilterSize
cm.tau.value = tau
cm.tau_predicted.value = tau_predicted

for key in mapJointNameToID:
    cm.setNameToId(key,mapJointNameToID[key])

cm.setDefaultMaxCurrent(-10.0)
cm.setDefaultMaxCurrent(maxCurrent)
cm.getDefaultMaxCurrent()
# Init should be called before addCtrlMode 
# because the size of state vector must be known.
cm.init(controlDT,testRobotPath,maxCurrent)

## Specify control mode ##
# Add position mode
cm.addCtrlMode("pos")
# Add torque mode
cm.addCtrlMode("torque")

cm.ctrl_torque.value = currentDes
cm.ctrl_pos.value = pwmDes
cm.setCtrlMode("all","pos")

cm.pwmDes.recompute(10)
cm.pwmDes.value

