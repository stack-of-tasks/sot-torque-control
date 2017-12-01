from dynamic_graph.sot.torque_control.main import *
from dynamic_graph.sot.torque_control.utils.sot_utils import smoothly_set_signal,  stop_sot
from dynamic_graph import plug
robot.timeStep=0.0015
robot = main_v3(robot, startSoT=True, go_half_sitting=1)

plug(robot.filters.estimator_kin.dx,         robot.base_estimator.joint_velocities);
robot.inv_dyn.kp_com.value = (20.0, 20.0, 20.0)
robot.inv_dyn.kd_com.value = (0.0, 0.0, 0.0)
robot.torque_ctrl.KpTorque.value = 30*(0.0,)

create_topic(robot.ros, robot.inv_dyn.com,                      'com')
create_topic(robot.ros, robot.inv_dyn.com_vel,                  'com_vel')
create_topic(robot.ros, robot.inv_dyn.com_ref_pos,              'com_ref_pos')
create_topic(robot.ros, robot.inv_dyn.com_ref_vel,              'com_ref_vel')
create_topic(robot.ros, robot.inv_dyn.com_acc_des,              'com_acc_des')
create_topic(robot.ros, robot.base_estimator.q,               'q');
create_topic(robot.ros, robot.base_estimator.v,               'v');
create_topic(robot.ros, robot.base_estimator.v_flex,          'v_flex');
create_topic(robot.ros, robot.base_estimator.v_kin,           'v_kin');
create_topic(robot.ros, robot.base_estimator.v_gyr,           'v_gyr');
create_topic(robot.ros, robot.inv_dyn.zmp_des,                  'cop_des')
create_topic(robot.ros, robot.inv_dyn.zmp,                      'cop')
create_topic(robot.ros, robot.inv_dyn.zmp_des_right_foot,       'cop_des_R')
create_topic(robot.ros, robot.inv_dyn.zmp_des_left_foot,        'cop_des_L')
create_topic(robot.ros, robot.inv_dyn.zmp_right_foot,           'cop_R')
create_topic(robot.ros, robot.inv_dyn.zmp_left_foot,            'cop_L')
create_topic(robot.ros, robot.inv_dyn.dq_admittance, 'dq_adm')
create_topic(robot.ros, robot.ctrl_manager.u,                        'i_des');	
create_topic(robot.ros, robot.filters.ft_LF_filter.x_filtered, 'f_LF_filt')
create_topic(robot.ros, robot.filters_sg.ft_LF_filter.x_filtered, 'f_LF_filt_sg', robot, robot.filters_sg.ft_LF_filter)

b = (0.00554272, 0.01108543, 0.00554272)
a = (1., -1.77863178, 0.80080265)
robot.filters.ft_RF_filter.switch_filter(b, a)
robot.filters.ft_LF_filter.switch_filter(b, a)
robot.filters.ft_RH_filter.switch_filter(b, a)
robot.filters.ft_LH_filter.switch_filter(b, a)
robot.filters.acc_filter.switch_filter(b, a)

b = (2.16439898e-05, 4.43473520e-05, -1.74065002e-05, -8.02197247e-05,  -1.74065002e-05,   4.43473520e-05, 2.16439898e-05)
a = (1.,-5.32595322,11.89749109,-14.26803139, 9.68705647,  -3.52968633,   0.53914042)
robot.filters.gyro_filter.switch_filter(b, a)
robot.filters.estimator_kin.switch_filter(b, a)

K = (1e10, 1e10, 1e10, 707, 502, 1e10);
robot.base_estimator.set_stiffness_left_foot(K)
robot.base_estimator.set_stiffness_right_foot(K)

robot.base_estimator.set_imu_weight(0.0)
robot.inv_dyn.w_forces.value = 1e-4

robot.base_estimator.reset_foot_positions();
sleep(1)
robot.inv_dyn.updateComOffset()

robot.ctrl_manager.setCtrlMode('rhp-rhy-rhr-rk-rar-rap-lhp-lhr-lhy-lk-lar-lap','torque')

robot.com_traj_gen.move(1, 0.03, 1.5)
robot.com_traj_gen.startSinusoid(1,-0.03,2.0)
robot.torque_ctrl.KdVel.value = 30*(0.1,)

ki_vel = 30*[0.0,]
ki_vel[5] = 0.1
ki_vel[11] = 0.1
robot.torque_ctrl.KiVel.value = tuple(ki_vel)

def print_cop_stats(robot):
  cop_L = robot.inv_dyn.zmp_left_foot.value
  cop_R = robot.inv_dyn.zmp_right_foot.value
  cop = robot.inv_dyn.zmp.value
  f_L = robot.inv_dyn.wrench_left_foot.value
  f_R = robot.inv_dyn.wrench_right_foot.value
  print "invdyn cop", cop[1]
  print "f_L", f_L[2]
  print "f_R", f_R[2]
  print "Average cop: ", 0.5*(cop_L[1]+cop_R[1])
  print "Weighted average cop: ", (f_L[2]*cop_L[1]+f_R[2]*cop_R[1])/(f_L[2]+f_R[2])
