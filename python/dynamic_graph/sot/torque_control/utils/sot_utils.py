# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph.sot.torque_control.create_entities_utils import create_tracer
import numpy as np
from time import sleep
import os
    
class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds);

    def __str__(self, prefix=""):
        res = "";
        for (key,value) in self.__dict__.iteritems():
            if (isinstance(value, np.ndarray) and len(value.shape)==2 and value.shape[0]>value.shape[1]):
                res += prefix+" - " + key + ": " + str(value.T) + "\n";
            elif (isinstance(value, Bunch)):
                res += prefix+" - " + key + ":\n" + value.__str__(prefix+"    ") + "\n";
            else:
                res += prefix+" - " + key + ": " + str(value) + "\n";
        return res[:-1];

def start_sot():
    os.system('rosservice call /start_dynamic_graph');

def stop_sot():
    os.system('rosservice call /stop_dynamic_graph');

def start_tracer(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl):
    tracer = create_tracer(robot.device, traj_gen, estimator, inv_dyn, torque_ctrl);
    tracer.start();
    return tracer;

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

def go_to_position(traj_gen,q,T=10.0):
    #put the robot in position q
    # RLEG TO 0 **********************
    traj_gen.moveJoint('rhy',q[0],T) #0
    traj_gen.moveJoint('rhr',q[1],T) #1
    traj_gen.moveJoint('rhp',q[2],T) #2
    traj_gen.moveJoint('rk' ,q[3],T) #3
    traj_gen.moveJoint('rap',q[4],T) #4
    traj_gen.moveJoint('rar',q[5],T) #5

    # LLEG TO 0 **********************
    traj_gen.moveJoint('lhy',q[6],T) #6
    traj_gen.moveJoint('lhr',q[7],T) #7
    traj_gen.moveJoint('lhp',q[8],T) #8
    traj_gen.moveJoint('lk' ,q[9],T) #9
    traj_gen.moveJoint('lap',q[10],T) #10
    traj_gen.moveJoint('lar',q[11],T) #11

    # TORSO TO 0
    traj_gen.moveJoint('ty' ,q[12],T) #12
    traj_gen.moveJoint('tp' ,q[13],T) #13

    # HEAD TO 0
    traj_gen.moveJoint('hy' ,q[14],T) #14
    traj_gen.moveJoint('hp' ,q[15],T) #15

    # RARM TO 0 **********************
    traj_gen.moveJoint('rsp',q[16],T) #16
    traj_gen.moveJoint('rsr',q[17],T) #17
    traj_gen.moveJoint('rsy',q[18],T) #18
    traj_gen.moveJoint('re' ,q[19],T) #19
    traj_gen.moveJoint('rwy',q[20],T) #20
    traj_gen.moveJoint('rwp',q[21],T) #21
    traj_gen.moveJoint('rh' ,q[22],T) #22

    # LARM TO 0 **********************
    traj_gen.moveJoint('lsp',q[23],T) #23
    traj_gen.moveJoint('lsr',q[24],T) #24
    traj_gen.moveJoint('lsy',q[25],T) #25
    traj_gen.moveJoint('le' ,q[26],T) #26
    traj_gen.moveJoint('lwy',q[27],T) #27
    traj_gen.moveJoint('lwp',q[28],T) #28
    traj_gen.moveJoint('lh' ,q[29],T) #29
    
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
    vf = np.array(final_value)
    for i in range(steps+1):
        alpha = 1.0*i/steps
        sig.value = tuple(vf*alpha+(1-alpha)*v);
        sleep(1.0*duration/steps);
        if (i%(steps/prints)==0):
            print 'smoothly setting signal... %(number)02d%%' % {"number": 100.*alpha} 
    print 'Signal set';
    sig.value = tuple(final_value);

def smoothly_set_signal_scalar(sig, final_value, duration=5.0, steps=500, prints = 10):
    v = sig.value;
    vf = final_value;
    for i in range(steps+1):
        alpha = 1.0*i/steps
        sig.value = vf*alpha+(1-alpha)*v;
        sleep(1.0*duration/steps);
        if (i%(steps/prints)==0):
            print 'smoothly setting signal... %(number)02d%%' % {"number": 100.*alpha}
    print 'Signal set';
    sig.value = final_value;
    
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
    
