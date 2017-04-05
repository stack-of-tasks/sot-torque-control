# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from create_entities_utils import create_tracer
import numpy as np
from time import sleep
    
def start_tracer(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl):
    tracer = create_tracer(robot.device, traj_gen, estimator, inv_dyn, torque_ctrl);
    tracer.start();
    return tracer;

def start_admittance_ctrl_RL(ctrl_manager):
    ctrl_manager.setCtrlMode('rhy','adm');
    ctrl_manager.setCtrlMode('rhr','adm');
    ctrl_manager.setCtrlMode('rhp','adm');
    ctrl_manager.setCtrlMode('rk','adm');
    ctrl_manager.setCtrlMode('rap','adm');
    ctrl_manager.setCtrlMode('rar','adm');

def move_to_initial_configuration(traj_gen):
    traj_gen.moveJoint('rhy', 0, 4.0);
    traj_gen.moveJoint('rhr', 0, 4.0);
    traj_gen.moveJoint('rhp',-0.6, 4);
    traj_gen.moveJoint('rk', 1.1, 4);
    traj_gen.moveJoint('rap',-0.6, 4);    
    traj_gen.moveJoint('rar', 0, 4.0);
    traj_gen.moveJoint('lhy', 0, 4.0);
    traj_gen.moveJoint('lhp', 0, 4.0);
    traj_gen.moveJoint('lhr', 0.5, 4.0);
    traj_gen.moveJoint('lk', 1.7, 4.0);
    traj_gen.moveJoint('lap', 0, 4.0);
    traj_gen.moveJoint('lar', 0, 4.0);
    
def smoothly_set_signal_to_zero(sig):
    v = np.array(sig.value);
    for i in range(40):
        v = 0.95*v;
        sig.value = tuple(v);
        sleep(1);
    print 'Setting signal to zero';
    v[:] = 0.0;
    sig.value = tuple(v);
    
def smoothly_set_signal(sig, final_value, duration=5.0, steps=500, prints = 10):
    v = np.array(sig.value);
    for i in range(steps+1):
        alpha = 1.0*i/steps
        sig.value = tuple(v*alpha);
        sleep(1.0*duration/steps);
        if (i%(steps/prints)==0):
            print 'smoothly setting signal... %(number)02d%%' % {"number": 100.*alpha} 
    print 'Signal set';
    sig.value = tuple(final_value);
    
def monitor_tracking_error(sig, sigRef, dt, time):
    N = int(time/dt);
    err = np.zeros((N,6));
    for i in range(N):
        err[i,:] = np.array(sig.value) - np.array(sigRef.value);
        sleep(dt);
    for i in range(6):
        print 'Max tracking error for axis %d:         %.2f' % (i, np.max(np.abs(err[:,i])));
        print 'Mean square tracking error for axis %d: %.2f' % (i, np.linalg.norm(err[:,i])/N);
    
def dump_signal_to_file(sig_list, index, filename, T, dt):
    N = int(T/dt);
    m = len(sig_list);
    f= open('/tmp/'+filename, 'a', 1);
    for t in range(N):
        for s in sig_list:
            f.write('{0}\t'.format(s.value[index]))
        f.write('\n');
        sleep(dt);
    f.close();
    
