import os
import sys
from time import sleep

import numpy as np

from dynamic_graph import plug
from dynamic_graph.sot.core.operator import Selec_of_vector
# from dynamic_graph.sot.torque_control.create_entities_utils import create_estimators
# from dynamic_graph.sot.torque_control.create_entities_utils import create_position_controller
# from dynamic_graph.sot.torque_control.create_entities_utils import create_trajectory_generator
from dynamic_graph.sot.torque_control.create_entities_utils import NJ, addTrace, create_inverse_dynamics
from dynamic_graph.sot.torque_control.filter_differentiator import FilterDifferentiator
from dynamic_graph.sot.torque_control.main import main_v3
from dynamic_graph.sot.torque_control.utils.sot_utils import Bunch, go_to_position, start_sot
from dynamic_graph.tracer_real_time import TracerRealTime

SIM_MODE = False
robot = None  # TODO
conf_traj = None  # TODO


def get_sim_conf():
    import dynamic_graph.sot.torque_control.hrp2.inverse_dynamics_controller_gains as inv_dyn_gains
    import dynamic_graph.sot.torque_control.hrp2.base_estimator_sim_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.control_manager_sim_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.hrp2.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains_sim as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.hrp2.motors_parameters as motor_params
    conf = Bunch()
    conf.inv_dyn_gains = inv_dyn_gains
    conf.base_estimator = base_estimator_conf
    conf.control_manager = control_manager_conf
    conf.force_torque_estimator = force_torque_estimator_conf
    conf.joint_torque_controller = joint_torque_controller_conf
    conf.pos_ctrl_gains = pos_ctrl_gains
    conf.motor_params = motor_params
    return conf


def get_default_conf():
    import dynamic_graph.sot.torque_control.hrp2.inverse_dynamics_controller_gains as inv_dyn_gains
    import dynamic_graph.sot.torque_control.hrp2.base_estimator_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.control_manager_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.hrp2.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.hrp2.motors_parameters as motor_params
    conf = Bunch()
    conf.inv_dyn_gains = inv_dyn_gains
    conf.base_estimator = base_estimator_conf
    conf.control_manager = control_manager_conf
    conf.force_torque_estimator = force_torque_estimator_conf
    conf.joint_torque_controller = joint_torque_controller_conf
    conf.pos_ctrl_gains = pos_ctrl_gains
    conf.motor_params = motor_params
    return conf


def create_base_encoders(robot):
    base_encoders = Selec_of_vector('base_encoders')
    plug(robot.device.robotState, base_encoders.sin)
    base_encoders.selec(0, NJ + 6)
    return base_encoders


def replug_estimator_kin(robot, estimator_fd):
    plug(robot.encoders.sout, estimator_fd.x)
    plug(estimator_fd.dx, robot.base_estimator.joint_velocities)
    # plug(estimator_fd.dx,             robot.ff_locator.joint_velocities);
    plug(estimator_fd.dx, robot.pos_ctrl.jointsVelocities)
    plug(estimator_fd.x_filtered, robot.estimator_ft.q_filtered)
    plug(estimator_fd.dx, robot.estimator_ft.dq_filtered)
    robot.estimator_ft.ddq_filtered.value = (0., ) * 30
    plug(estimator_fd.dx, robot.torque_ctrl.jointsVelocities)
    robot.torque_ctrl.jointsAccelerations.value = (0., ) * 30
    plug(estimator_fd.dx, robot.inv_dyn_ctrl.jointsVelocities)
    plug(estimator_fd.dx, robot.ctrl_manager.dq)
    return


def replug_inv_dyn(robot, inv_dyn_ctrl):
    plug(robot.ctrl_manager.joints_ctrl_mode_torque, robot.inv_dyn_ctrl.controlledJoints)
    return


def setup_velocity_filter(robot, conf, filter_b, filter_a):
    main_v3(robot, startSoT=False, go_half_sitting=False, conf=None)
    robot.estimator_fd = FilterDifferentiator("fd_filter")
    dt = robot.timeStep
    robot.estimator_fd.init(dt, NJ, filter_b, filter_a)
    robot.inv_dyn_ctrl = create_inverse_dynamics(robot, conf.inv_dyn_gains, conf.motor_params, dt=dt)
    replug_inv_dyn(robot, robot.inv_dyn_ctrl)
    replug_estimator_kin(robot, robot.estimator_fd)


def wait_for_motion():
    while not robot.traj_gen.isTrajectoryEnded():
        sleep(1.0)
    return


def dump_stop_tracer(tracer):
    print("Dumping Tracer")
    tracer.stop()
    sleep(0.2)
    tracer.dump()
    sleep(0.2)
    tracer.close()
    sleep(0.2)
    tracer.clear()
    sleep(0.2)
    return


def conf_velocity_filter():
    conf = Bunch()
    conf.j_name = "re"
    conf.x_f = -1.0
    conf.w_min = 0.01
    conf.w_max = 0.7
    conf.deltaT = 1.0
    return conf


def conf_filter_list():
    conf = Bunch()

    conf.b_list = [
        np.array([0.00902094, 0.01800633, 0.00902094]),
        np.array([0.00542136, 0.01084273, 0.00542136]),
        np.array([0.00355661, 0.00711322, 0.00355661]),
        np.array([0.00208057, 0.00416113, 0.00208057]),
        np.array([0.03046875, 0.03046875]),
        np.array([0.0020518, 0.00410359, 0.0020518]),
        np.array([0.00094469, 0.00188938, 0.00094469]),
        np.array([6.70858643e-05, 2.01257593e-04, 2.01257593e-04, 6.70858643e-05]),
        np.array([0.00010101, 0.00016741, 0.00016741, 0.00010101]),
        np.array([9.36150553e-05, 2.80845166e-04, 2.80845166e-04, 9.36150553e-05]),
        np.array([0.00093575, 0.0018715, 0.00093575]),
        np.array([
            1.67370188e-07, 1.00422113e-06, 2.51055282e-06, 3.34740376e-06, 2.51055282e-06, 1.00422113e-06,
            1.67370188e-07
        ]),
        np.array([4.27323622e-06, 1.70929449e-05, 2.56394173e-05, 1.70929449e-05, 4.27323622e-06]),
        np.array([1.38055495e-05, -2.98864128e-05, 1.80947937e-05, 1.80947937e-05, -2.98864128e-05, 1.38055495e-05]),
        np.array([1.95099715e-07, 9.75498574e-07, 1.95099715e-06, 1.95099715e-06, 9.75498574e-07, 1.95099715e-07]),
        np.array([
            1.70134115e-10, 1.36107292e-09, 4.76375523e-09, 9.52751046e-09, 1.19093881e-08, 9.52751046e-09,
            4.76375523e-09, 1.36107292e-09, 1.70134115e-10
        ])
    ]

    conf.a_list = [
        np.array([1., -1.72610371, 0.76236003]),
        np.array([1., -1.73969005, 0.7613755]),
        np.array([1., -1.78994555, 0.80417199]),
        np.array([1., -1.86689228, 0.87521455]),
        np.array([1., -0.93906251]),
        np.array([1., -1.84107589, 0.84928308]),
        np.array([1., -1.91119707, 0.91497583]),
        np.array([1., -2.85434518, 2.721757, -0.86687514]),
        np.array([1., -2.8543607, 2.72178648, -0.86688893]),
        np.array([1., -2.77503623, 2.57077857, -0.79499342]),
        np.array([1., -1.89310959, 0.8968526]),
        np.array([1., -5.30603285, 11.7558334, -13.91966085, 9.2894345, -3.31270578, 0.49314228]),
        np.array([1., -3.70990618, 5.16705461, -3.20193343, 0.74485337]),
        np.array([1., -4.80072628, 9.23289417, -8.89170853, 4.28778797, -0.82824331]),
        np.array([1., -4.64518256, 8.63880318, -8.0399445, 3.74448588, -0.69815575]),
        np.array([
            1., -7.27133134, 23.1548678, -42.17546448, 48.05912505, -35.08178214, 16.02023822, -4.18419295, 0.47853988
        ])
    ]
    # conf.b_list = np.flipud(conf.b_list)
    # conf.a_list = np.flipud(conf.a_list)

    return conf


def test_velocity_filters(robot):
    conf_traj = conf_velocity_filter()
    conf_list = conf_filter_list()
    filter_b = tuple(conf_list.b_list[0])
    filter_a = tuple(conf_list.a_list[0])
    b_list = [tuple(b) for b in conf_list.b_list[1:]]
    a_list = [tuple(a) for a in conf_list.a_list[1:]]
    if SIM_MODE:
        conf = get_sim_conf()
    else:
        conf = get_default_conf()
    setup_velocity_filter(robot, conf, filter_b, filter_a)

    print("Gonna start SoT")
    sleep(1.0)
    start_sot()

    print("Gonna go to half sitting")
    sleep(1.0)
    wait_for_motion()
    go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)

    wait_for_motion()
    index = 0
    dir_name = '/tmp/' + conf_traj.j_name + '/filter_0/'
    robot.tracer = create_tracer(robot, dir_name)
    robot.tracer.start()
    robot.traj_gen.startLinChirp(conf_traj.j_name, conf_traj.x_f, conf_traj.w_min, conf_traj.w_max, conf_traj.deltaT)
    wait_for_motion()
    dump_stop_tracer(robot.tracer)
    print("Gonna go to half sitting")
    sys.__stdout__.flush()
    sleep(1.0)
    go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)
    for b, a in zip(b_list, a_list):
        index += 1
        dir_name = '/tmp/' + conf_traj.j_name + '/filter_' + str(index) + '/'
        robot.estimator_fd.switch_filter(b, a)
        robot.tracer = create_tracer(robot, dir_name)
        print("Gonna start chirp")
        sys.__stdout__.flush()
        robot.tracer.start()
        robot.traj_gen.startLinChirp(conf_traj.j_name, conf_traj.x_f, conf_traj.w_min, conf_traj.w_max,
                                     conf_traj.deltaT)
        wait_for_motion()
        dump_stop_tracer(robot.tracer)
        print("Gonna go to half sitting")
        sys.__stdout__.flush()
        sleep(1.0)
        go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)
        wait_for_motion()


def test_velocity_filters2(robot):
    print("Gonna go to half sitting")
    sleep(1.0)
    go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)

    wait_for_motion()
    index = 0
    dir_name = '/tmp/' + conf_traj.j_name + '/filter_0/'
    robot.tracer = create_tracer(robot, dir_name)
    robot.tracer.start()
    robot.traj_gen.startLinChirp(conf_traj.j_name, conf_traj.x_f, conf_traj.w_min, conf_traj.w_max, conf_traj.deltaT)
    wait_for_motion()
    dump_stop_tracer(robot.tracer)
    print("Gonna go to half sitting")
    sleep(1.0)
    go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)
    for b, a in zip(b_list, a_list):  # noqa TODO
        index += 1
        dir_name = '/tmp/' + conf_traj.j_name + '/filter_' + str(index) + '/'
        robot.estimator_fd.switch_filter(b, a)
        robot.tracer = create_tracer(robot, dir_name)
        print("Gonna start chirp")
        robot.tracer.start()
        robot.traj_gen.startLinChirp(conf_traj.j_name, conf_traj.x_f, conf_traj.w_min, conf_traj.w_max,
                                     conf_traj.deltaT)
        wait_for_motion()
        dump_stop_tracer(robot.tracer)
        print("Gonna go to half sitting")
        sleep(1.0)
        go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)
        wait_for_motion()


def addSignalsToTracer(tracer, robot):
    addTrace(tracer, robot.estimator_fd, 'x')
    addTrace(tracer, robot.device, 'robotState')
    addTrace(tracer, robot.device, 'control')
    addTrace(tracer, robot.inv_dyn_ctrl, 'qRef')
    addTrace(tracer, robot.inv_dyn_ctrl, 'dqRef')
    addTrace(tracer, robot.estimator_fd, 'x_filtered')
    addTrace(tracer, robot.estimator_fd, 'dx')
    return


def create_tracer(robot, dir_name):
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)
    tracer = TracerRealTime('motor_id_trace')
    tracer.setBufferSize(80 * (2**20))
    tracer.open(dir_name, 'dg_', '.dat')
    robot.device.after.addSignal('{0}.triger'.format(tracer.name))
    addSignalsToTracer(tracer, robot)
    return tracer
