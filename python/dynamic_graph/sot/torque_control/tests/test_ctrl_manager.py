# -*- coding: utf-8 -*-
"""
2017, LAAS/CNRS
@author: Florent Forget
"""

import numpy as np
from dynamic_graph import plug
from time import sleep
import matplotlib.pyplot as plt
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import NJ
from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.hrp2.motors_parameters import *
from IPython import embed



def main(task='', dt=0.001, delay=0.01):
    ctrl_manager = ControlManager("ctrl_man");

    ctrl_manager.max_tau.value = NJ*(CTRL_MANAGER_TAU_MAX,);
    ctrl_manager.max_current.value = NJ*(CTRL_MANAGER_CURRENT_MAX,);
    ctrl_manager.percentageDriverDeadZoneCompensation.value = NJ*(PERCENTAGE_DRIVER_DEAD_ZONE_COMPENSATION,);
    ctrl_manager.signWindowsFilterSize.value = NJ*(SIGN_WINDOW_FILTER_SIZE,);
    ctrl_manager.bemfFactor.value = NJ*(0.0,);

    ctrl_manager.addCtrlMode("pos");
    ctrl_manager.addCtrlMode("torque");
    ctrl_manager.addEmergencyStopSIN("0");
    ctrl_manager.addEmergencyStopSIN("1");
    ctrl_manager.addEmergencyStopSIN("2");

    ctrl_manager.base6d_encoders.value = np.zeros(30)
    ctrl_manager.tau.value = np.zeros(30)
    ctrl_manager.tau_predicted.value = np.zeros(30)
    ctrl_manager.dq.value = np.zeros(30)
    ctrl_manager.ctrl_torque.value = 1.2*np.ones(30)
    ctrl_manager.ctrl_pos.value = 3.0*np.ones(30)

    ctrl_manager.emergencyStop_0.value = 0;
    ctrl_manager.emergencyStop_1.value = 0;
    ctrl_manager.emergencyStop_2.value = 0;

    ctrl_manager.setCtrlMode("all", "pos");
    ctrl_manager.setCtrlMode('rhp-rhy-rhr-rk-rar-rap-lhp-lhr-lhy-lk-lar-lap','torque');
    ctrl_manager.init(dt);
    sleep(1.0);
    
    ''' standard use '''
    for i in range(0,10000,2):
        ctrl_manager.ctrl_torque.value = 1.2*np.ones(30)
        ctrl_manager.ctrl_pos.value = 3.0*np.ones(30)
        ctrl_manager.pwmDesSafe.recompute(ctrl_manager.pwmDesSafe.time+1)
        ctrl_manager.pwmDes.recompute(ctrl_manager.pwmDes.time+1)
    if ctrl_manager.pwmDes.value == (   1.2, 1.2, 1.2,
                                        1.2, 1.2, 1.2,
                                        1.2, 1.2, 1.2,
                                        1.2, 1.2, 1.2,
                                        3.0, 3.0, 3.0,
                                        3.0, 3.0, 3.0,
                                        3.0, 3.0, 3.0,
                                        3.0, 3.0, 3.0,
                                        3.0, 3.0, 3.0,
                                        3.0, 3.0, 3.0):
        print "ctrl-manager working properly"
    else:
        print "ctrl-manager default"


    ''' too high current '''
    ctrl_manager.init(dt);
    for i in range(0,10000,2):
        ctrl_manager.ctrl_torque.value = 1.2*np.ones(30)
        ctrl_manager.ctrl_pos.value = 3.0*np.ones(30)
        if(i>5000):
            ctrl_manager.ctrl_torque.value = 20.0*np.ones(30)
        ctrl_manager.pwmDesSafe.recompute(ctrl_manager.pwmDesSafe.time+1)
        ctrl_manager.pwmDes.recompute(ctrl_manager.pwmDes.time+1)

    if ctrl_manager.pwmDesSafe.value == (   0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0):
        print "current limitation working properly"
    else:
        print "current limitation issue"


    ''' emergency signal Up '''
    ctrl_manager.init(dt);
    for i in range(0,10000,2):
        if(i==5000):
            ctrl_manager.emergencyStop_2.value = 1;
        ctrl_manager.ctrl_torque.value = 1.2*np.ones(30)
        ctrl_manager.ctrl_pos.value = 3.0*np.ones(30)
        ctrl_manager.pwmDesSafe.recompute(ctrl_manager.pwmDesSafe.time+1)
        ctrl_manager.pwmDes.recompute(ctrl_manager.pwmDes.time+1)
        if(i==5000):
            break


    if ctrl_manager.pwmDesSafe.value == (   0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0):
        print "emergency signal working properly"
    else:
        print "emergency signal limitation issue"
    return (ctrl_manager);

if __name__=='__main__':
    (ctrl_manager) = main('none',0.001,0.01);
    pass;
    