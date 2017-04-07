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
    traj_gen        = create_trajectory_generator(robot.device, dt);
    com_traj_gen    = create_com_traj_gen(dt);
    estimator       = create_estimator(robot.device, dt, delay, traj_gen);

    ff_locator      = create_free_flyer_locator(robot.device, estimator, urdfFileName, robot.dynamic);
    flex_est        = create_flex_estimator(robot, ff_locator, dt);
    floatingBase    = create_floatingBase(flex_est, ff_locator);

    pos_ctrl        = create_position_controller(robot.device, estimator, dt, traj_gen);
    torque_ctrl     = create_torque_controller(robot.device, estimator);    
#    inv_dyn         = create_inverse_dynamics(robot.device, estimator, torque_ctrl, traj_gen, dt);    
    inv_dyn         = create_balance_controller(robot.device, floatingBase, estimator, torque_ctrl, traj_gen, com_traj_gen, urdfFileName, dt);    
    ctrl_manager    = create_ctrl_manager(robot.device, torque_ctrl, pos_ctrl, inv_dyn, estimator, dt);
    
    estimator.gyroscope.value = (0.0, 0.0, 0.0);
#    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    if(startSoT):
        print "Gonna start SoT";
        sleep(1.0);
        start_sot();

        if(go_half_sitting):
            print "Gonna go to half sitting";
            sleep(1.0);
            go_to_position(traj_gen, robot.halfSitting[6:], 10.0);

    return Bunch(estimator=estimator, torque_ctrl=torque_ctrl, traj_gen=traj_gen, com_traj_gen=com_traj_gen, ctrl_manager=ctrl_manager, inv_dyn=inv_dyn, pos_ctrl=pos_ctrl, ff_locator=ff_locator, flex_est=flex_est, floatingBase=floatingBase);

    
''' Main function to call after having started the graph. '''
def main_post_start(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl, ff_locator, floatingBase):
    ros = create_ros_topics(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl, ff_locator, floatingBase);
    return ros;

