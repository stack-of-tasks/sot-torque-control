# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from create_entities_utils import *
from hrp2_motors_parameters import *

''' Main function to call before starting the graph. '''
def main_pre_start_pwm(robot,dt=0.001,delay=0.01, urdfFileName='/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14.urdf'):
    robot.device.setControlInputType('position');
    traj_gen        = create_trajectory_generator(robot.device, dt);
    estimator       = create_estimator(robot.device, dt, delay, traj_gen);

    ff_locator      = create_free_flyer_locator(robot.device, estimator, urdfFileName, robot.dynamic);
    flex_est        = create_flex_estimator(robot, ff_locator, dt);
    floatingBase    = create_floatingBase(flex_est, ff_locator);

    pos_ctrl        = create_position_controller(robot.device, estimator, dt, traj_gen);
    torque_ctrl     = create_torque_controller(robot.device, estimator);    
#    inv_dyn         = create_inverse_dynamics(robot.device, estimator, torque_ctrl, traj_gen, dt);    
    inv_dyn         = create_balance_controller(robot.device, floatingBase, estimator, torque_ctrl, traj_gen, urdfFileName, dt);    
    ctrl_manager    = create_ctrl_manager(robot.device, torque_ctrl, pos_ctrl, inv_dyn, estimator, dt);
    

    estimator.gyroscope.value = (0.0, 0.0, 0.0);
#    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    return (estimator,torque_ctrl,traj_gen,ctrl_manager,inv_dyn,pos_ctrl,ff_locator,flex_est,floatingBase);

''' Main function to call before starting the graph. '''
def main_pre_start_pwm_noTorqueControl(robot,dt=0.001,delay=0.01):
    robot.device.setControlInputType('position');
    traj_gen        = create_trajectory_generator(robot.device, dt);
    estimator       = create_estimator(robot.device, dt, delay, traj_gen);
    pos_ctrl        = create_position_controller(robot.device, estimator, dt, traj_gen);
    torque_ctrl     = create_torque_controller(robot.device, estimator);    
    #inv_dyn         = create_inverse_dynamics(robot.device, estimator, torque_ctrl, traj_gen, dt);    
    ctrl_manager    = create_ctrl_manager_noTorqueControl (robot.device, torque_ctrl, pos_ctrl, estimator, dt);
        
    estimator.gyroscope.value = (0.0, 0.0, 0.0);
    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    return (estimator,torque_ctrl,traj_gen,ctrl_manager,pos_ctrl);




''' Main function to call before starting the graph. '''
def main_pre_start(task,robot,dt=0.001,delay=0.01):
    robot.device.setControlInputType('position');
    traj_gen        = create_trajectory_generator(robot.device, dt);
    estimator       = create_estimator(robot.device, dt, delay, traj_gen);
    torque_ctrl     = create_torque_controller(robot.device, estimator);    
    inv_dyn         = create_inverse_dynamics(robot.device, estimator, torque_ctrl, traj_gen, dt);    
    ctrl_manager    = create_ctrl_manager(robot.device, torque_ctrl, traj_gen, inv_dyn, estimator, dt);
    adm_ctrl        = create_admittance_ctrl(robot.device, estimator, ctrl_manager, traj_gen, dt);
    
    if(task=='identification'):
        estimator.setUseRawEncoders(False);
        estimator.setUseRefJointVel(False);
        estimator.setUseRefJointAcc(False);
#        estimator.setUseFTsensors(False);
        robot.device.after.addSignal('estimator.jointsVelocities');
    elif(task=='torque_ctrl'):
        ctrl_manager.setCtrlMode('rhy','torque');
        ctrl_manager.setCtrlMode('rhr','torque');
        ctrl_manager.setCtrlMode('rhp','torque');
        ctrl_manager.setCtrlMode('rk','torque');
        ctrl_manager.setCtrlMode('rap','torque');
        ctrl_manager.setCtrlMode('rar','torque');
    
    estimator.gyroscope.value = (0.0, 0.0, 0.0);
    #estimator.accelerometer.value = (0.0, 0.0, 9.81);
        
    return (estimator,torque_ctrl,traj_gen,ctrl_manager,inv_dyn,adm_ctrl);

    
''' Main function to call after having started the graph. '''
def main_post_start(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl, ff_locator, floatingBase):
    ros = create_ros_topics(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl, ff_locator, floatingBase);
    return ros;
