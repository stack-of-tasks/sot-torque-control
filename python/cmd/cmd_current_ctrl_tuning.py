create_topic(robot.ros, robot.device.currents,                    'i');
create_topic(robot.ros, robot.ctrl_manager.currents_real,         'i_real');
create_topic(robot.ros, robot.ctrl_manager.pwmDes,                'i_des')
create_topic(robot.ros, robot.device.robotState,                  'robotState')
create_topic(robot.ros, robot.ctrl_manager.pwmDesSafe,            'ctrl')
create_topic(robot.ros, robot.ctrl_manager.current_errors,        'i_err');
create_topic(robot.ros, robot.estimator_ft.jointsTorques,       'tau');
create_topic(robot.ros, robot.torque_ctrl.jointsTorquesDesired, 'tau_des');

# create trajectory generator for torque
from dynamic_graph.sot.torque_control.nd_trajectory_generator import NdTrajectoryGenerator
torque_traj_gen = NdTrajectoryGenerator('torque_traj_gen')
torque_traj_gen.initial_value.value = 30*(0.0,)
torque_traj_gen.init(robot.timeStep, 30)
plug(torque_traj_gen.x, robot.torque_ctrl.jointsTorquesDesired)

# create trajectory generator for current
from dynamic_graph.sot.torque_control.nd_trajectory_generator import NdTrajectoryGenerator
cur_traj_gen = NdTrajectoryGenerator('cur_traj_gen')
cur_traj_gen.initial_value.value = 30*(0.0,)
cur_traj_gen.init(robot.timeStep, 30)
plug(cur_traj_gen.x, robot.ctrl_manager.ctrl_torque)

robot.ctrl_manager.current_sensor_offsets_real_out.recompute(1)
offset = list(robot.ctrl_manager.current_sensor_offsets_real_out.value)
robot.ctrl_manager.current_sensor_offsets_real_in.value = tuple(offset)

dz = list(robot.ctrl_manager.percentageDriverDeadZoneCompensation.value)
robot.ctrl_manager.percentageDriverDeadZoneCompensation.value = tuple(dz)

i_offsets = list(robot.ctrl_manager.current_sensor_offsets_low_level.value)
robot.ctrl_manager.current_sensor_offsets_low_level.value = tuple(i_offsets)

iMaxDzComp = list(robot.ctrl_manager.iMaxDeadZoneCompensation.value)
robot.ctrl_manager.iMaxDeadZoneCompensation.value = tuple(iMaxDzComp)

bemf_comp = list(robot.ctrl_manager.percentage_bemf_compensation.value)
robot.ctrl_manager.percentage_bemf_compensation.value = tuple(bemf_comp)

ki_current = list(robot.ctrl_manager.ki_current.value)
robot.ctrl_manager.ki_current.value = tuple(ki_current)

robot.ctrl_manager.setCtrlMode('rk', 'torque')
