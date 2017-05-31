from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.torque_control.se3_trajectory_generator import *
from numpy import matrix, identity, zeros, eye, array, pi, ndarray, ones

# Instanciate a pose trajectory generator
initial_value=zeros(6)
se3tg=SE3TrajectoryGenerator("se3tg_test")
se3tg.initial_value.value = initial_value
se3tg.recompute(10)





