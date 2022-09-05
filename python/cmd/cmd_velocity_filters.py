print(robot)
from dynamic_graph.sot.torque_control.tests.test_velocity_filters import *

robot.timeStep = 0.00125
dt = robot.timeStep
conf_traj = Bunch()
conf_list = conf_filter_list()
conf = get_default_conf()

# conf_traj.j_name="rk"
# conf_traj.x_f=1.5
# conf_traj.w_min=0.1
# conf_traj.w_max=0.4
# conf_traj.deltaT=30.0

conf_traj.j_name = "rhp"
conf_traj.x_f = -1.5
conf_traj.w_min = 0.05
conf_traj.w_max = 0.2
conf_traj.deltaT = 20.0

conf.joint_torque_controller.FRICTION_COMPENSATION_PERCENTAGE = 0.0


b_list = [tuple(b) for b in conf_list.b_list]
a_list = [tuple(a) for a in conf_list.a_list]

filter_index = 15
# setup_velocity_filter(robot, conf, (1.,0.), (1.,0.)
setup_velocity_filter(robot, conf, b_list[filter_index], a_list[filter_index])

robot.inv_dyn_ctrl.dynamicsError.value = (0.0,) * 36
robot.inv_dyn_ctrl.Kf.value = 30 * (0.0,)
robot.inv_dyn_ctrl.Kd.value = 30 * (7.0,)
robot.inv_dyn_ctrl.Kp.value = 30 * (200.0,)

start_sot()

create_topic(robot.ros, robot.estimator_kin.dx, "dq_savgol", robot, robot.estimator_kin)
create_topic(robot.ros, robot.estimator_fd.dx, "dq_fd", robot, robot.estimator_fd)
create_topic(robot.ros, robot.inv_dyn_ctrl.dqRef, "dqRef", robot, robot.inv_dyn_ctrl)
create_topic(robot.ros, robot.inv_dyn_ctrl.qRef, "qRef", robot, robot.inv_dyn_ctrl)
create_topic(robot.ros, robot.device.robotState, "robotState", robot, robot.device)
create_topic(robot.ros, robot.inv_dyn_ctrl.qError, "qError", robot, robot.inv_dyn_ctrl)
create_topic(robot.ros, robot.estimator_fd.x_filtered, "q_fd", robot, robot.estimator_fd)
create_topic(robot.ros, robot.device.currents, "i")
create_topic(robot.ros, robot.ctrl_manager.currents_real, "i_real")
create_topic(robot.ros, robot.ctrl_manager.pwmDes, "i_des")
create_topic(robot.ros, robot.estimator_ft.jointsTorques, "tau")
create_topic(robot.ros, robot.torque_ctrl.jointsTorquesDesired, "tau_des")
sleep(1.0)
go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)

# start torque control on right leg joints
robot.ctrl_manager.setCtrlMode("rhp-rhy-rhr-rk-rar-rap", "torque")

i_max = np.array(robot.ctrl_manager.max_current.value)
i_max[3] = 10.0
i_max[9] = 10.0
robot.ctrl_manager.max_current.value = tuple(i_max)

ctrl_max = np.array(robot.ctrl_manager.max_ctrl.value)
ctrl_max[:12] = 20.0
robot.ctrl_manager.max_ctrl.value = tuple(ctrl_max)

kp = np.array(robot.pos_ctrl.Kp.value)
kp[:12] = 100.0
robot.pos_ctrl.Kp.value = tuple(kp)

robot.traj_gen.playTrajectoryFile("/home/adelpret/devel/src/sot-torque-control/share/climbing32_1.25ms.pos")

#################################################################
robot.ctrl_manager.setCtrlMode(conf_traj.j_name, "torque")
sleep(0.5)
#################################################################3
# filter_index -=1
# b = b_list[filter_index]
# a = a_list[filter_index]
# robot.estimator_fd.init(dt, NJ, b, a)


filter_index = 0
robot.estimator_fd.switch_filter(b_list[filter_index], a_list[filter_index])
robot.estimator_fd.switch_filter((1.0, 0.0), (1.0, 0.0))
sleep(2.0)
# dir_name='/tmp/'+'climbing32_1.25ms.pos'+'/filter_'+str(filter_index)+'/'
# robot.tracer = create_tracer(robot, dir_name)


# robot.tracer.start()
# robot.traj_gen.playTrajectoryFile('/home/adelpret/devel/src/sot-torque-control/share/climbing32_1.25ms.pos')
sleep(5.0)
robot.traj_gen.playTrajectoryFile("/home/adelpret/devel/src/sot-torque-control/share/climbing32_1.25ms.pos")
sleep(12.0)
dump_stop_tracer(robot.tracer)
robot.ctrl_manager.setCtrlMode(conf_traj.j_name, "pos")
go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)

robot.pos_ctrl.jointsVelocities.value = 30 * (0.0,)
robot.estimator_ft.dq_filtered.value = 30 * (0.0,)
robot.torque_ctrl.jointsVelocities.value = 30 * (0.0,)
robot.inv_dyn_ctrl.jointsVelocities.value = 30 * (0.0,)
robot.ctrl_manager.dq.value = 30 * (0.0,)
sleep(0.5)
filter_index = 15
# robot.estimator_fd.switch_filter((1., 0.), (1., 0.))
sleep(5.0)
plug(robot.estimator_fd.dx, robot.pos_ctrl.jointsVelocities)
plug(robot.estimator_fd.dx, robot.estimator_ft.dq_filtered)
plug(robot.estimator_fd.dx, robot.torque_ctrl.jointsVelocities)
plug(robot.estimator_fd.dx, robot.inv_dyn_ctrl.jointsVelocities)
plug(robot.estimator_fd.dx, robot.ctrl_manager.dq)

stop_sot()
