# plug encoder velocities (with different base vel) to balance controller
from dynamic_graph.sot.core import Stack_of_vector, Selec_of_vector
robot.v = Stack_of_vector('v')
plug(robot.base_estimator.v_flex, robot.v.sin1)
plug(robot.filters.estimator_kin.dx, robot.v.sin2)
robot.v.selec1(0, 6)
robot.v.selec2(0, 30)
plug(robot.v.sout, robot.inv_dyn.v)

# Compute finite differences of base position
from dynamic_graph.sot.torque_control.utils.filter_utils import create_chebi2_lp_filter_Wn_03_N_4
#robot.q_fd = create_chebi2_lp_filter_Wn_03_N_4('q_filter', robot.timeStep, 36)
from dynamic_graph.sot.torque_control.filter_differentiator import FilterDifferentiator
robot.q_fd = FilterDifferentiator('q_filter')
robot.q_fd.init(robot.timeStep, 36, (1., 0.), (1., 0.))
plug(robot.base_estimator.q, robot.q_fd.x)
create_topic(robot.ros, robot.q_fd.dx, 'q_fd')

# Replace force sensor filters
from dynamic_graph.sot.torque_control.utils.filter_utils import create_butter_lp_filter_Wn_05_N_3, create_chebi2_lp_filter_Wn_03_N_4
robot.force_LF_filter = create_chebi2_lp_filter_Wn_03_N_4('force_LF_filter', robot.timeStep, 6)
robot.force_RF_filter = create_chebi2_lp_filter_Wn_03_N_4('force_RF_filter', robot.timeStep, 6)
plug(robot.device.forceLLEG, robot.force_LF_filter.x)
plug(robot.force_LF_filter.x_filtered, robot.base_estimator.forceLLEG)
plug(robot.force_LF_filter.dx, robot.base_estimator.dforceLLEG)
plug(robot.device.forceRLEG, robot.force_RF_filter.x)
plug(robot.force_RF_filter.x_filtered, robot.base_estimator.forceRLEG)
plug(robot.force_RF_filter.dx, robot.base_estimator.dforceRLEG)

# reconnect sav-gol filters
plug(robot.filters.ft_LF_filter.x_filtered, robot.base_estimator.forceLLEG)
plug(robot.filters.ft_RF_filter.x_filtered, robot.base_estimator.forceRLEG)
plug(robot.filters.ft_LF_filter.dx, robot.base_estimator.dforceLLEG)
plug(robot.filters.ft_RF_filter.dx, robot.base_estimator.dforceRLEG)

# change delay of low-pass filters
dt = robot.timeStep
delay = 0.06
robot.filters.ft_RF_filter.init(dt, 6, delay, 1)
robot.filters.ft_LF_filter.init(dt, 6, delay, 1)
robot.filters.ft_RH_filter.init(dt, 6, delay, 1)
robot.filters.ft_LH_filter.init(dt, 6, delay, 1)
robot.filters.gyro_filter.init(dt, 3, delay, 1)
robot.filters.acc_filter.init(dt, 3, delay, 1)
robot.filters.estimator_kin.init(dt, 30, delay, 2)
