# modify file /opt/ros/indigo/lib/python2.7/dist-packages/rqt_py_console/spyder_console_widget.py (moduleCompletion -> module_completion)

# file to modify joint bounds: 
# nano ~/devel/hrp2-n14-system/src/direct-access/hrp2-io-boards-init.cpp

from dynamic_graph.sot.torque_control.main import *
from dynamic_graph.sot.torque_control.utils.sot_utils import smoothly_set_signal,  stop_sot
from dynamic_graph import plug
robot.timeStep=0.0015
robot = main_v3(robot, startSoT=True, go_half_sitting=True)
#go_to_position(robot.traj_gen, 30*(0.0,), 5.0)

#robot.base_estimator.set_imu_weight(0.0)
plug(robot.filters.estimator_kin.dx,         robot.base_estimator.joint_velocities);
#plug(robot.filters.estimator_kin.dx,         robot.current_ctrl.dq);
#plug(robot.filters.estimator_kin.dx,         robot.torque_ctrl.jointsVelocities);
robot.inv_dyn.kp_com.value = (30.0, 30.0, 50.0)
robot.inv_dyn.kd_com.value = (8.0, 8.0, 0.0)
robot.torque_ctrl.KpTorque.value = 30*(0.0,)
#plug(robot.filters_sg.ft_LF_filter.x_filtered, robot.base_estimator.forceLLEG)
#plug(robot.filters_sg.ft_RF_filter.x_filtered, robot.base_estimator.forceRLEG)

# create ros topics
#create_topic(robot.ros, robot.device.robotState,                'robotState')
#create_topic(robot.ros, robot.estimator_ft.jointsTorques,       'tau');
#create_topic(robot.ros, robot.torque_ctrl.jointsTorquesDesired, 'tau_des');
create_topic(robot.ros, robot.inv_dyn.com,                      'com')
create_topic(robot.ros, robot.inv_dyn.com_vel,                  'com_vel')
create_topic(robot.ros, robot.inv_dyn.com_ref_pos,              'com_ref_pos')
create_topic(robot.ros, robot.inv_dyn.com_ref_vel,              'com_ref_vel')
create_topic(robot.ros, robot.inv_dyn.com_acc_des,              'com_acc_des')
create_topic(robot.ros, robot.base_estimator.lf_xyzquat,        'lf_est')
create_topic(robot.ros, robot.base_estimator.rf_xyzquat,        'rf_est')
#create_topic(robot.ros, robot.current_ctrl.i_real,              'i_real');
#create_topic(robot.ros, robot.ctrl_manager.u,                   'i_des')
create_topic(robot.ros, robot.base_estimator.q,               'q');
create_topic(robot.ros, robot.base_estimator.v,               'v');
create_topic(robot.ros, robot.base_estimator.v_flex,          'v_flex');
create_topic(robot.ros, robot.base_estimator.v_kin,           'v_kin');
create_topic(robot.ros, robot.base_estimator.v_gyr,           'v_gyr');
create_topic(robot.ros, robot.inv_dyn.zmp_des,                  'cop_des')
create_topic(robot.ros, robot.inv_dyn.zmp,                      'cop')

create_topic(robot.ros, robot.filters.ft_LF_filter.x_filtered, 'f_LF_filt')
create_topic(robot.ros, robot.filters_sg.ft_LF_filter.x_filtered, 'f_LF_filt_sg', robot, robot.filters_sg.ft_LF_filter)


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
robot.inv_dyn.updateComOffset()
# start torque control on leg joints
robot.ctrl_manager.setCtrlMode('rhp-rhy-rhr-rk-rar-rap-lhp-lhr-lhy-lk-lar-lap','torque')

# enable integral feedback in torque control
import dynamic_graph.sot.torque_control.hrp2.motors_parameters as motor_params
robot.torque_ctrl.torque_integral_saturation.value = tuple(0.5*motor_params.Kf_n / motor_params.Kt_n)
robot.torque_ctrl.KiTorque.value = 30*(5.0,)

# enable foot pose update in base estimator
robot.base_estimator.K_fb_feet_poses.value = 1e-3

# compute derivatives of joint torques
from dynamic_graph.sot.torque_control.numerical_difference import NumericalDifference
torque_der = NumericalDifference("torque_der");
plug(robot.torque_ctrl.jointsTorques, torque_der.x)
torque_der.init(robot.timeStep, 30, 0.015, 2);
create_topic(robot.ros, torque_der.dx, 'dtau')
plug(torque_der.dx, robot.torque_ctrl.jointsTorquesDerivative)

# switch to position control
go_to_position(robot.traj_gen, robot.device.robotState.value[6:], 3.0)
robot.ctrl_manager.setCtrlMode('all', 'pos')

create_topic(robot.ros, robot.inv_dyn.f_des_right_foot,         'f_des_R')
create_topic(robot.ros, robot.inv_dyn.f_des_left_foot,          'f_des_L')
create_topic(robot.ros, robot.inv_dyn.tau_des,                  'tau_des');
create_topic(robot.ros, robot.estimator_ft.contactWrenchLeftSole,  'f_LeftSole');
create_topic(robot.ros, robot.estimator_ft.contactWrenchRightSole, 'f_RightSole');
create_topic(robot.ros, robot.ctrl_manager.pwmDes,              'i_des');	
create_topic(robot.ros, robot.ctrl_manager.ctrl_torque,              'i_des');

create_topic(robot.ros, robot.inv_dyn.zmp_des_right_foot_local, 'cop_des_R_local')
create_topic(robot.ros, robot.inv_dyn.zmp_des_left_foot_local,  'cop_des_L_local')
create_topic(robot.ros, robot.inv_dyn.zmp_des_right_foot,       'cop_des_R')
create_topic(robot.ros, robot.inv_dyn.zmp_des_left_foot,        'cop_des_L')
create_topic(robot.ros, robot.inv_dyn.zmp_right_foot,           'cop_R')
create_topic(robot.ros, robot.inv_dyn.zmp_left_foot,            'cop_L')
create_topic(robot.ros, robot.inv_dyn.zmp_des,                  'cop_des')
create_topic(robot.ros, robot.inv_dyn.zmp,                      'cop')

create_topic(robot.ros, robot.estimator_kin.dx,                'dq_sav');
create_topic(robot.ros, robot.encoder_filter.dx,               'dq');
create_topic(robot.ros, robot.device.gyrometer,               'gyrometer');
create_topic(robot.ros, robot.device.accelerometer,           'accelerometer');



--------------------ROBOT ON THE GROUND ---------------------------------

robot.com_traj_gen.move(2, 0.85, 2.0)
smoothly_set_signal(robot.torque_ctrl.KpTorque,30*(1.,))
smoothly_set_signal(robot.inv_dyn.kp_posture,30*(5.,))

robot.com_traj_gen.stop(-1)
robot.com_traj_gen.move(1, 0.05, 1.5)
robot.com_traj_gen.startSinusoid(1,-0.05,1.5)


