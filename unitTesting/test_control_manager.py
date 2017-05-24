from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.control_manager import *
from dynamic_graph.sot.torque_control.tests.robot_data_test import testRobotPath,controlDT,maxCurrent,mapJointNameToID, \
    mapJointLimits, urdftosot,mapForceIdToForceLimits, mapNameToForceId, \
    FootFrameNames, RightFootSensorXYZ
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


# Init should be called before addCtrlMode 
# because the size of state vector must be known.
cm.init(controlDT,testRobotPath,maxCurrent)

# Set the map from joint name to joint ID
for key in mapJointNameToID:
    cm.setNameToId(key,mapJointNameToID[key])

# Set the map joint limits for each id
for key in mapJointLimits:
    cm.setJointLimitsFromId(key,mapJointLimits[key][0],mapJointLimits[key][1])

# Set the force limits for each id
for key in mapForceIdToForceLimits:
    cm.setForceLimitsFromId(key,tuple(mapForceIdToForceLimits[key][0]),tuple(mapForceIdToForceLimits[key][1]))

# Set the force sensor id for each sensor name
for key in mapNameToForceId:
    cm.setForceNameToForceId(key,mapNameToForceId[key])

# Set the map from the urdf joint list to the sot joint list
cm.setJointsUrdfToSot(urdftosot)

# Set the foot frame name
for key in FootFrameNames:
    cm.setFootFrameName(key,FootFrameNames[key])

cm.setRightFootSoleXYZ(RightFootSensorXYZ)

cm.setDefaultMaxCurrent(-10.0)
cm.setDefaultMaxCurrent(maxCurrent)
cm.getDefaultMaxCurrent()

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

cm.displayRobotUtil()

