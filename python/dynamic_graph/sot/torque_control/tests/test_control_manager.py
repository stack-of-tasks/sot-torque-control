from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.tests.robot_data_test import initRobotData
from numpy import array, ones, zeros

# Instanciate the free flyer
cm = ControlManager("cm_test")

q = zeros(initRobotData.nbJoints + 6)
dq = zeros(initRobotData.nbJoints)
bemfFactor = ones(initRobotData.nbJoints)
max_current = 30.0 * ones(initRobotData.nbJoints)
max_tau = 100.0 * ones(initRobotData.nbJoints)
percentageDriverDeadZoneCompensation = 20.0 * ones(initRobotData.nbJoints)
signWindowsFilterSize = ones(initRobotData.nbJoints)
tau = 100.0 * ones(initRobotData.nbJoints)
tau_predicted = 110.0 * ones(initRobotData.nbJoints)
pwmDes = 100.0 * ones(initRobotData.nbJoints)
currentDes = 100.0 * ones(initRobotData.nbJoints)

cm.controlDT = 0.005

# Initializing the input ports
# Setting the robot configuration
cm.i_max.value = max_current
cm.u_max.value = max_tau
cm.tau.value = tau
cm.tau_predicted.value = tau_predicted

cmInitRobotData = initRobotData()

cmInitRobotData.init_and_set_controller_manager(cm)

# Specify control mode ##
# Add position mode
cm.addCtrlMode("pos")
# Add torque mode
cm.addCtrlMode("torque")

# TODO ctrl_* non working yet
# cm.ctrl_torque.value = array(currentDes)
# cm.ctrl_pos.value = pwmDes
# cm.setCtrlMode("all", "pos")
