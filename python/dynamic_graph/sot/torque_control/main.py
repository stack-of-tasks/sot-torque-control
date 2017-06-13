# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph.sot.torque_control.create_entities_utils import *
from dynamic_graph.sot.torque_control.utils.sot_utils import *
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import *
from time import sleep

''' Main function to call before starting the graph. '''
def main_pre_start(robot, delay=0.01, startSoT=True, go_half_sitting=True, urdfFileName='/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14.urdf'):
    dt = robot.timeStep;
    robot.device.setControlInputType('position');
    
    robot.traj_gen        = create_trajectory_generator(robot.device, dt);
    robot.com_traj_gen    = create_com_traj_gen(dt);
    robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf");
    robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf");
    robot.rf_traj_gen.init(dt);
    robot.lf_traj_gen.init(dt);
    (robot.estimator_ft, robot.estimator_kin)       = create_estimators(robot, dt, delay);

    robot.ff_locator      = create_free_flyer_locator(robot, urdfFileName);
    robot.flex_est        = create_flex_estimator(robot, dt);
    robot.floatingBase    = create_floatingBase(robot);

    robot.pos_ctrl        = create_position_controller(robot, dt);
    robot.torque_ctrl     = create_torque_controller(robot);
#    inv_dyn         = create_inverse_dynamics(robot, dt);    
    robot.inv_dyn         = create_balance_controller(robot, urdfFileName, dt);
    robot.ctrl_manager    = create_ctrl_manager(robot, dt);
    
    robot.estimator_ft.gyroscope.value = (0.0, 0.0, 0.0);
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
    
''' Main function to call before starting the graph. '''
def main_pre_start_v3(robot, delay=0.01, startSoT=True, go_half_sitting=True, urdfFileName='/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14.urdf'):
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
         
    robot.device.setControlInputType('position');
    
    robot.traj_gen        = create_trajectory_generator(robot.device, dt);
    robot.com_traj_gen    = create_com_traj_gen(dt);
    robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf");
    robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf");
    robot.rf_traj_gen.init(dt);
    robot.lf_traj_gen.init(dt);
    robot.estimator       = create_estimator(robot, dt, delay);

    robot.ff_locator      = create_free_flyer_locator(robot, urdfFileName);

    robot.pos_ctrl        = create_position_controller(robot, dt);
    robot.torque_ctrl     = create_torque_controller(robot);
    robot.inv_dyn         = create_balance_controller(robot, urdfFileName, dt);
    robot.ctrl_manager    = create_ctrl_manager(robot, dt);
    
    robot.estimator.gyroscope.value = (0.0, 0.0, 0.0);
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

    
''' Main function to call after having started the graph. '''
def main_post_start(robot):
    ros = create_ros_topics(robot);
    return ros;

