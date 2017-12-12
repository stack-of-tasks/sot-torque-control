from dynamic_graph.sot.torque_control.main import *
from dynamic_graph.sot.torque_control.utils.sot_utils import smoothly_set_signal,  stop_sot
from dynamic_graph import plug
import numpy as np
robot.timeStep=0.0015
robot = main_v3(robot, startSoT=1, go_half_sitting=1)

robot.torque_ctrl.KpTorque.value = 30*(0.0,)
robot.inv_dyn.kp_com.value = (20.0, 20.0, 20.0)
robot.inv_dyn.kd_com.value = (5.0, 5.0, 0.0)
robot.inv_dyn.kp_posture.value = 30*(30.,)
robot.base_estimator.reset_foot_positions();
robot.inv_dyn.updateComOffset()

robot.ctrl_manager.setCtrlMode('rhp-rhy-rhr-rk-rar-rap-lhp-lhr-lhy-lar-lap','torque')

robot.traj_gen.moveJoint('lk', 0.8, 3.0)
robot.com_traj_gen.move(2, 0.85, 3.0)

create_topic(robot.ros, robot.base_estimator.q,               'q');
create_topic(robot.ros, robot.adm_ctrl.fRightFootRef,         'f_des_R')
create_topic(robot.ros, robot.adm_ctrl.fLeftFootRef,          'f_des_L')
create_topic(robot.ros, robot.adm_ctrl.u,                     'u_adm')
create_topic(robot.ros, robot.adm_ctrl.dqDes,                 'dq_des')
create_topic(robot.ros, robot.estimator_ft.contactWrenchLeftSole,  'f_L');
create_topic(robot.ros, robot.estimator_ft.contactWrenchRightSole, 'f_R');
create_topic(robot.ros, robot.inv_dyn.com,                      'com')
create_topic(robot.ros, robot.inv_dyn.com_vel,                  'com_vel')
create_topic(robot.ros, robot.inv_dyn.com_ref_pos,              'com_ref_pos')
create_topic(robot.ros, robot.inv_dyn.com_ref_vel,              'com_ref_vel')
create_topic(robot.ros, robot.inv_dyn.zmp_des,                  'cop_des')
create_topic(robot.ros, robot.inv_dyn.zmp,                      'cop')
create_topic(robot.ros, robot.inv_dyn.zmp_right_foot,           'cop_R')
create_topic(robot.ros, robot.inv_dyn.zmp_left_foot,            'cop_L')
create_topic(robot.ros, robot.inv_dyn.zmp_des_right_foot,       'cop_des_R')
create_topic(robot.ros, robot.inv_dyn.zmp_des_left_foot,        'cop_des_L')
create_topic(robot.ros, robot.ctrl_manager.u,                   'i_des');
create_topic(robot.ros, robot.current_ctrl.i_real,              'i_real');


robot.adm_ctrl.force_integral_saturation.value = (0,0,160.0,20.0,20.0,0)
kp_vel = 12*[0.5,]+18*[0.0,]
smoothly_set_signal(robot.adm_ctrl.kp_vel, tuple(kp_vel))
#smoothly_set_signal(robot.adm_ctrl.kp_vel, 30*(0.0,))
smoothly_set_signal(robot.adm_ctrl.ki_force, (0, 0, 20.0, 2, 0, 0))

b = (0.00554272, 0.01108543, 0.00554272)
a = (1., -1.77863178, 0.80080265)
b = (1.0, 0.0)
a = (1.0, 0.0)
robot.filters.ft_RF_filter.switch_filter(b, a)
robot.filters.ft_LF_filter.switch_filter(b, a)
robot.filters.ft_RH_filter.switch_filter(b, a)
robot.filters.ft_LH_filter.switch_filter(b, a)
robot.filters.acc_filter.switch_filter(b, a)

robot.com_traj_gen.move(1, 0.03, 1.5)
robot.com_traj_gen.startSinusoid(1,-0.03,2.0)

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
