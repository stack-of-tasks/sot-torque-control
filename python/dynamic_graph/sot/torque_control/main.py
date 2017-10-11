# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph import plug
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.create_entities_utils import create_trajectory_generator, create_com_traj_gen, create_encoders
from dynamic_graph.sot.torque_control.create_entities_utils import create_imu_offset_compensation, create_estimators, create_imu_filter
from dynamic_graph.sot.torque_control.create_entities_utils import create_base_estimator, create_position_controller, create_torque_controller
from dynamic_graph.sot.torque_control.create_entities_utils import create_balance_controller, create_ctrl_manager, create_ros_topics
from dynamic_graph.sot.torque_control.create_entities_utils import create_free_flyer_locator, create_flex_estimator, create_floatingBase
from dynamic_graph.sot.torque_control.create_entities_utils import create_current_controller, connect_ctrl_manager
from dynamic_graph.sot.torque_control.create_entities_utils import create_tracer, create_topic
from dynamic_graph.ros import RosPublish
from dynamic_graph.sot.torque_control.utils.sot_utils import start_sot, stop_sot, go_to_position, Bunch
from dynamic_graph.sot.torque_control.utils.filter_utils import create_chebi2_lp_filter_Wn_03_N_4

from time import sleep

def get_default_conf():
    import dynamic_graph.sot.torque_control.hrp2.balance_ctrl_conf as balance_ctrl_conf
    import dynamic_graph.sot.torque_control.hrp2.base_estimator_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.control_manager_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.hrp2.current_controller_conf as current_controller_conf
    import dynamic_graph.sot.torque_control.hrp2.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.hrp2.joint_pos_ctrl_gains as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.hrp2.motors_parameters as motor_params
    conf = Bunch();
    conf.balance_ctrl              = balance_ctrl_conf;
    conf.base_estimator            = base_estimator_conf;
    conf.control_manager           = control_manager_conf;
    conf.current_ctrl              = current_controller_conf;
    conf.force_torque_estimator    = force_torque_estimator_conf;
    conf.joint_torque_controller   = joint_torque_controller_conf;
    conf.pos_ctrl_gains            = pos_ctrl_gains;
    conf.motor_params              = motor_params;
    return conf;

''' Main function to call before starting the graph. '''
def main_v3(robot, startSoT=True, go_half_sitting=True, conf=None):
    if(conf is None):
        conf = get_default_conf();
    dt = robot.timeStep;
    
    # TMP: overwrite halfSitting configuration to use SoT joint order
    robot.halfSitting = (
         # Free flyer
         0., 0., 0.648702, 0., 0. , 0.,
         # Legs
         0., 0., -0.453786, 0.872665, -0.418879, 0.,
         0., 0., -0.453786, 0.872665, -0.418879, 0.,
         # Chest and head
         0., 0., 0., 0.,
         # Arms
         0.261799, -0.17453, 0., -0.523599, 0., 0., 0.1,
         0.261799,  0.17453, 0., -0.523599, 0., 0., 0.1);
    
    robot.device.setControlInputType('noInteg');
    robot.ctrl_manager    = create_ctrl_manager(conf.control_manager, conf.motor_params, dt);
    
    robot.traj_gen        = create_trajectory_generator(robot.device, dt);
    robot.com_traj_gen    = create_com_traj_gen(conf.balance_ctrl, dt);
    robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf");
    robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf");
    robot.rf_traj_gen.init(dt);
    robot.lf_traj_gen.init(dt);
    
    robot.encoders                              = create_encoders(robot);
    robot.imu_offset_compensation               = create_imu_offset_compensation(robot, dt);
    (robot.estimator_ft, robot.filters)         = create_estimators(robot, conf.force_torque_estimator, conf.motor_params, dt);
    robot.imu_filter                            = create_imu_filter(robot, dt);
    robot.base_estimator                        = create_base_estimator(robot, dt, conf.base_estimator);

    robot.pos_ctrl        = create_position_controller(robot, conf.pos_ctrl_gains, dt);
    robot.torque_ctrl     = create_torque_controller(robot, conf.joint_torque_controller, conf.motor_params, dt);
    robot.inv_dyn         = create_balance_controller(robot, conf.balance_ctrl,conf.motor_params, dt);
    robot.current_ctrl    = create_current_controller(robot, conf.current_ctrl, conf.motor_params, dt);
    connect_ctrl_manager(robot);

    # create low-pass filter for computing joint velocities
    robot.encoder_filter = create_chebi2_lp_filter_Wn_03_N_4('encoder_filter', dt, conf.motor_params.NJ)
    plug(robot.encoders.sout,             robot.encoder_filter.x)
    plug(robot.encoder_filter.dx,         robot.current_ctrl.dq);
    plug(robot.encoder_filter.dx,         robot.torque_ctrl.jointsVelocities);
    plug(robot.encoder_filter.x_filtered, robot.base_estimator.joint_positions);
    plug(robot.encoder_filter.dx,         robot.base_estimator.joint_velocities);

    robot.ros = RosPublish('rosPublish');
    robot.device.after.addDownsampledSignal('rosPublish.trigger',1);

    robot.estimator_ft.dgyro.value = (0.0, 0.0, 0.0);
    robot.estimator_ft.gyro.value = (0.0, 0.0, 0.0);
#    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    if(startSoT):
        print "Gonna start SoT";
        sleep(1.0);
        start_sot();

        if(go_half_sitting):
            print "Gonna go to half sitting in 1 sec";
            sleep(1.0);
            go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0);

    return robot;

    
''' Main function to call after having started the graph. '''
def main_post_start(robot):
    ros = create_ros_topics(robot);
    return ros;

''' Main function to call before starting the graph. '''
def main_v2(robot, delay=0.01, startSoT=True, go_half_sitting=True, urdfFileName='/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14.urdf'):
    dt = robot.timeStep;
    robot.device.setControlInputType('position');
    
    robot.traj_gen        = create_trajectory_generator(robot.device, dt);
    robot.com_traj_gen    = create_com_traj_gen(dt);
    robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf");
    robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf");
    robot.rf_traj_gen.init(dt);
    robot.lf_traj_gen.init(dt);
    (robot.estimator_ft, robot.filters)       = create_estimators(robot, dt, delay);

    robot.ff_locator      = create_free_flyer_locator(robot, urdfFileName);
    robot.flex_est        = create_flex_estimator(robot, dt);
    robot.floatingBase    = create_floatingBase(robot);

    robot.pos_ctrl        = create_position_controller(robot, dt);
    robot.torque_ctrl     = create_torque_controller(robot);
#    inv_dyn         = create_inverse_dynamics(robot, dt);    
    robot.inv_dyn         = create_balance_controller(robot, urdfFileName, dt);
    robot.ctrl_manager    = create_ctrl_manager(robot, dt);
    
    robot.estimator_ft.gyro.value = (0.0, 0.0, 0.0);
#    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    if(startSoT):
        print "Gonna start SoT";
        sleep(1.0);
        start_sot();

        if(go_half_sitting):
            print "Gonna go to half sitting";
            sleep(1.0);
            go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0);

    return robot;
