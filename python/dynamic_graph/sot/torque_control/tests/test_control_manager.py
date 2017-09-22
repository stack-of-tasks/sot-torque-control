from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.control_manager import *
from dynamic_graph.sot.torque_control.tests.robot_data_test import initRobotData
from numpy import matrix, identity, zeros, eye, array, pi, ndarray, ones

# Instanciate the free flyer
cm = ControlManager("cm_test")

q=zeros(initRobotData.nbJoints+6)
dq=zeros(initRobotData.nbJoints)
bemfFactor=ones(initRobotData.nbJoints)
max_current=30.0*ones(initRobotData.nbJoints)
max_tau = 100.0*ones(initRobotData.nbJoints)
percentageDriverDeadZoneCompensation = 20.0*ones(initRobotData.nbJoints)
signWindowsFilterSize = ones(initRobotData.nbJoints)
tau = 100.0 * ones(initRobotData.nbJoints)
tau_predicted = 110.0 * ones(initRobotData.nbJoints)
pwmDes = 100.0 *ones(initRobotData.nbJoints)
currentDes = 100.0 *ones(initRobotData.nbJoints)

cm.controlDT=0.005

# Initializing the input ports
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

cmInitRobotData = initRobotData()

cmInitRobotData.init_and_set_controller_manager(cm)

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


