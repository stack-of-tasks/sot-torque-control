# modify file /opt/ros/indigo/lib/python2.7/dist-packages/rqt_py_console/spyder_console_widget.py (moduleCompletion -> module_completion)

from dynamic_graph.sot.torque_control.main import *
from dynamic_graph.sot.torque_control.utils.sot_utils import smoothly_set_signal,  stop_sot
from dynamic_graph import plug
robot.timeStep=0.0015
robot = main_v3(robot, startSoT=True, go_half_sitting=False)

robot.base_estimator.w_lf_in.value = 1.0
robot.base_estimator.w_rf_in.value = 1.0
robot.inv_dyn.v.value = 36*(0.,)
robot.inv_dyn.kd_com.value = (0.0, 0.0, 0.0)
robot.inv_dyn.kd_feet.value = 6*(0.,)
robot.inv_dyn.kd_constraints.value = 6*(0.,)
robot.inv_dyn.kp_com.value = (30.0, 30.0, 50.0)

# enable integral feedback in torque control
import dynamic_graph.sot.torque_control.hrp2.motors_parameters as motor_params
robot.torque_ctrl.torque_integral_saturation.value = tuple(0.9*motor_params.Kf_n / motor_params.Kt_n)
robot.torque_ctrl.KiTorque.value = 30*(5.0,)

# create ros topics
create_topic(robot.ros, robot.device.robotState,                'robotState')
create_topic(robot.ros, robot.estimator_ft.jointsTorques,       'tau');
create_topic(robot.ros, robot.torque_ctrl.jointsTorquesDesired, 'tau_des');
create_topic(robot.ros, robot.inv_dyn.com,                      'com')
create_topic(robot.ros, robot.inv_dyn.com_vel,                  'com_vel')
create_topic(robot.ros, robot.inv_dyn.com_ref_pos,              'com_ref_pos')
create_topic(robot.ros, robot.inv_dyn.com_ref_vel,              'com_ref_vel')
create_topic(robot.ros, robot.inv_dyn.com_acc_des,              'com_acc_des')
create_topic(robot.ros, robot.base_estimator.lf_xyzquat,        'lf_est')
create_topic(robot.ros, robot.base_estimator.rf_xyzquat,        'rf_est')
create_topic(robot.ros, robot.current_ctrl.i_real,              'i_real');
create_topic(robot.ros, robot.ctrl_manager.u,                   'i_des')
create_topic(robot.ros, robot.base_estimator.q,               'q');
create_topic(robot.ros, robot.base_estimator.v,               'v');

# wait until the motion has finished
go_to_position(robot.traj_gen, 30*(0.0,), 5.0)
# put the robot down
robot.imu_offset_compensation.update_offset(5.0)
# wait 5 seconds
robot.imu_filter.setBeta(1.0)
sleep(1.0)
import dynamic_graph.sot.torque_control.hrp2.base_estimator_conf as base_est_conf
robot.imu_filter.setBeta(base_est_conf.beta)
# lift the robot
go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0);
# put the robot down
robot.base_estimator.reset_foot_positions();
# start torque control on leg joints
robot.ctrl_manager.setCtrlMode('rhp-rhy-rhr-rk-rar-rap-lhp-lhr-lhy-lk-lar-lap','torque')
robot.base_estimator.K_fb_feet_poses.value = 1e-3

# plug encoder velocities (with different base vel) to balance controller
from dynamic_graph.sot.core import Stack_of_vector, Selec_of_vector
robot.v = Stack_of_vector('v');
plug(robot.base_estimator.v_flex, robot.v.sin1);
plug(robot.filters.estimator_kin.dx, robot.v.sin2)
robot.v.selec1(0,6)
robot.v.selec2(0,30)
plug(robot.v.sout, robot.inv_dyn.v)

# Compute finite differences of base position
from dynamic_graph.sot.torque_control.utils.filter_utils import create_butter_lp_filter_Wn_05_N_3
robot.q_fd = create_butter_lp_filter_Wn_05_N_3('q_filter', robot.timeStep, 36)
plug(robot.base_estimator.q, robot.q_fd.x)
create_topic(robot.ros, robot.q_fd.dx,         'q_fd')

# Replace force sensor filters
from dynamic_graph.sot.torque_control.utils.filter_utils import create_butter_lp_filter_Wn_05_N_3
robot.force_LF_filter = create_butter_lp_filter_Wn_05_N_3('force_LF_filter', robot.timeStep, 6)
robot.force_RF_filter = create_butter_lp_filter_Wn_05_N_3('force_RF_filter', robot.timeStep, 6)
plug(robot.device.forceLLEG, robot.force_LF_filter.x)
plug(robot.force_LF_filter.x_filtered, robot.base_estimator.forceLLEG)
plug(robot.force_LF_filter.dx, robot.base_estimator.dforceLLEG)
plug(robot.device.forceRLEG, robot.force_RF_filter.x)
plug(robot.force_RF_filter.x_filtered, robot.base_estimator.forceRLEG)
plug(robot.force_RF_filter.dx, robot.base_estimator.dforceRLEG)

# set dz comp and bemf comp to zero
robot.ctrl_manager.percentageDriverDeadZoneCompensation.value = 30*(0.,)
robot.ctrl_manager.percentage_bemf_compensation.value = 30*(0.,)
robot.ctrl_manager.ki_current.value = 30*(0.,)
# disable friction compensatio
robot.torque_ctrl.motorParameterKv_p.value = 30*(0.,)
robot.torque_ctrl.motorParameterKv_n.value = 30*(0.,)
# disable flexibility estimation and IMU
robot.base_estimator.forceLLEG.value = robot.filters.ft_LF_filter.x_filtered.value
robot.base_estimator.forceRLEG.value = robot.filters.ft_RF_filter.x_filtered.value
robot.base_estimator.set_imu_weight(0.0)
# change delay of low-pass filters
dt = robot.timeStep
delay = 0.06
robot.filters.ft_RF_filter.init( dt, 6, delay, 1)
robot.filters.ft_LF_filter.init( dt, 6, delay, 1)
robot.filters.ft_RH_filter.init( dt, 6, delay, 1)
robot.filters.ft_LH_filter.init( dt, 6, delay, 1)
robot.filters.gyro_filter.init(  dt, 3, delay, 1)
robot.filters.acc_filter.init(   dt, 3, delay, 1)
robot.filters.estimator_kin.init(dt,30, delay, 2);

# connect old velocity estimator
plug(robot.filters.estimator_kin.dx,         robot.ctrl_manager.dq);
plug(robot.filters.estimator_kin.dx,         robot.torque_ctrl.jointsVelocities);
plug(robot.filters.estimator_kin.dx,         robot.base_estimator.joint_velocities);

# switch to position control
go_to_position(robot.traj_gen, robot.device.robotState.value[6:], 3.0)
robot.ctrl_manager.setCtrlMode('all', 'pos')


create_topic(robot.ros, robot.inv_dyn.f_des_right_foot,         'inv_dyn_f_des_R')
create_topic(robot.ros, robot.inv_dyn.f_des_left_foot,          'inv_dyn_f_des_L')
create_topic(robot.ros, robot.inv_dyn.tau_des,                  'inv_dyn_tau_des');
create_topic(robot.ros, robot.estimator_ft.contactWrenchLeftSole,  'f_LeftSole');
create_topic(robot.ros, robot.estimator_ft.contactWrenchRightSole, 'f_RightSole');
create_topic(robot.ros, robot.ctrl_manager.pwmDes,              'i_des');	
create_topic(robot.ros, robot.ctrl_manager.ctrl_torque,              'i_des');

create_topic(robot.ros, robot.inv_dyn.zmp_des_right_foot_local, 'inv_dyn_cop_R')
create_topic(robot.ros, robot.inv_dyn.zmp_des_left_foot_local,  'inv_dyn_cop_L')
create_topic(robot.ros, robot.inv_dyn.zmp_des_right_foot,       'inv_dyn_cop_R_world')
create_topic(robot.ros, robot.inv_dyn.zmp_des_left_foot,        'inv_dyn_cop_L_world')
create_topic(robot.ros, robot.inv_dyn.zmp_des,                  'inv_dyn_cop')
create_topic(robot.ros, robot.inv_dyn.zmp_right_foot,           'inv_dyn_cop_R_mes')
create_topic(robot.ros, robot.inv_dyn.zmp_left_foot,            'inv_dyn_cop_L_mes')
create_topic(robot.ros, robot.inv_dyn.zmp,                      'inv_dyn_cop_mes')

create_topic(robot.ros, robot.estimator_kin.dx,                'dq_sav');
create_topic(robot.ros, robot.encoder_filter.dx,               'dq');
create_topic(robot.ros, robot.device.gyrometer,               'gyrometer');
create_topic(robot.ros, robot.device.accelerometer,           'accelerometer');



--------------------ROBOT ON THE GROUND ---------------------------------

robot.com_traj_gen.move(2, 0.85, 2.0)
smoothly_set_signal(robot.torque_ctrl.KpTorque,30*(1.,))
smoothly_set_signal(robot.torque_ctrl.KpTorque,30*(0.,))
smoothly_set_signal(robot.inv_dyn.kp_posture,30*(5.,))
smoothly_set_signal(robot.inv_dyn.kp_posture,30*(1.,))

robot.com_traj_gen.stop(-1)
robot.com_traj_gen.move(1,0.03,3.0)
robot.com_traj_gen.startSinusoid(1,-0.03,1.5)


