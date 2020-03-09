from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
from dynamic_graph.sot.torque_control.tests.robot_data_test import initRobotData
from dynamic_graph.sot.torque_control.tests.test_control_manager import cm
from numpy import zeros

cm.displayRobotUtil()

# Instanciate the free flyer
ffl = FreeFlyerLocator("ffl_test")

ffl.init(initRobotData.robotRef)

# TODO : Set the value of the encoders.

q = zeros(initRobotData.nbJoints + 6)
dq = zeros(initRobotData.nbJoints)
ffl.base6d_encoders.value = q
ffl.joint_velocities.value = dq
ffl.freeflyer_aa.recompute(100)

ffl.displayRobotUtil()
