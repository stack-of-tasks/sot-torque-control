# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import random
import numpy as np
from dynamic_graph import plug
from create_entities_utils import *
from hrp2_motors_parameters_sim import *
from dynamic_graph.sot.torque_control.hrp2_device_pos_ctrl import HRP2DevicePosCtrl
import robotviewer  # start robotviewer from bash with 'robotviewer -sXML-RPC'.
from time import sleep
from plot_utils import create_empty_figure
import plot_utils
import matplotlib.pyplot as plt

USE_ROBOT_VIEWER = False;

def randTuple(size):
    v = ();
    for i in range(0,size):
        v = v + (random.random(),);
    return v;
    
def simulate(device, duration, dt=0.001):
    N = int(duration/dt);
    if(USE_ROBOT_VIEWER):
        viewer=robotviewer.client('XML-RPC');
    for i in range(1,N):
        device.increment (dt);
        sleep(dt);
        if(USE_ROBOT_VIEWER):            
            viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);

def create_device(kp=30*[1,]):
    # create an instance of the device
    device = HRP2DevicePosCtrl("device");
    # This line isn't necessary but I need to test it
    device.setControlInputType('position');
    q     = (0,0,0.7)+(NJ+3)*(0.0,);
    q_des = NJ*(0.0,); #randTuple(nj);
    device.resize(NJ+6);
    device.set(q);
    device.kp.value = tuple(kp);
    device.kd.value = tuple(2*np.sqrt(np.array(kp)));
    device.control.value = q_des;
    device.increment(0.001);
    return device;

def test_vel_acc_estimator(device, estimator_ft, estimator_kin, dt=0.001, estimationDelay = 5):
    j               = 0;        # index of joint under analysis
    N               = 1000;     # test duration (in number of timesteps)
    
    q        = device.state.value;
    rad2deg  = 180/3.14;
    q_real   = rad2deg*q[j+6];
    dq_real  = 0;
    ddq_real = 0;
    err_q    = 0;
    err_dq   = 0;
    err_ddq  = 0;
    count    = 0;
    t        = estimator_ft.jointsTorques.time;
    if(USE_ROBOT_VIEWER):
        viewer=robotviewer.client('XML-RPC');
    print "Start simulation..."
    for i in range(1,N):
        device.increment (dt);
        if(USE_ROBOT_VIEWER):            
            viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);
        estimator_ft.jointsTorques.recompute (t+i);
        if(i%estimationDelay == 0):
            q_real   = rad2deg*device.robotState.value[j+6];
            dq_real  = rad2deg*device.jointsVelocities.value[j];
            ddq_real = rad2deg*device.jointsAccelerations.value[j];
            if(i>estimationDelay):
                print "\nTime %.3f) q est VS real = (%.2f, %.2f), dq est VS real (%.2f, %.2f) ddq est VS real (%.2f, %.2f)" % (i*dt, \
                    rad2deg*estimator_kin.x_filtered.value[j], q_real, \
                    rad2deg*estimator_kin.dx.value[j], dq_real, \
                    rad2deg*estimator_kin.ddx.value[j], ddq_real);
                print "Torque %f" % (estimator_ft.jointsTorques.value[j]);
                
                if(abs(ddq_real)<1e-2):
                    ddq_real = 1e-2;
                if(q_real!=0.0):
                    err_q   += abs((rad2deg*estimator_kin.x_filtered.value[j]     - q_real)/q_real);
                if(dq_real!=0.0):
                    err_dq  += abs((rad2deg*estimator_kin.dx.value[j]    - dq_real)/dq_real);
                if(ddq_real!=0.0):
                    err_ddq += abs((rad2deg*estimator_kin.ddx.value[j] - ddq_real)/ddq_real);
                count   += 1;
    
    print "*************************************************************************************************\n";
    print "Testing joint %d" % j;
    print "Initial position of joint %d: %.2f" % (j, rad2deg*q[6+j]);
    print "Desired position of joint %d: %.2f" % (j, rad2deg*device.control.value[j]);
    print "Test duration: %.3f s" % (N*dt);
    print "Time step: %.3f s" % dt;
    print "Estimation delay: %.3f" % (estimationDelay*dt);
    if(count>0):
        print "Average percentual errors:\n* position: %.2f %%\n* velocity: %.2f %%\n* acceleration: %.2f %%" % \
            (100*err_q/count, 100*err_dq/count, 100*err_ddq/count);

def test_force_estimator(device, estimator_ft, estimator_kin, dt=0.001, estimationDelay = 5):
    N               = 1000;     # test duration (in number of timesteps)
    freq = 0.5; # frequency
    A = 100;    # amplitude
    
    f        = np.zeros((4,6));
    f_est    = np.zeros((4,6));
    err_f    = np.zeros((4,6));
    count    = 0;
    t = 0;
    if(USE_ROBOT_VIEWER):
        viewer=robotviewer.client('XML-RPC');
    print "Start simulation..."
    for i in range(1,N):
        t += dt;
        axis = int(random.random()*6);
        for j in range(4):
            f[j,axis] += A*(np.sin(2*3.14*t)-np.sin(2*3.14*(t-dt)));
        device.inputForceLARM.value = tuple(f[0,:]);
        device.inputForceRARM.value = tuple(f[1,:]);
        device.inputForceLLEG.value = tuple(f[2,:]);
        device.inputForceRLEG.value = tuple(f[3,:]);
        device.increment (dt);
        if(USE_ROBOT_VIEWER):            
            viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);
        if(i%estimationDelay == 0):
            f_est[0,:]   = np.array(estimator_ft.contactWrenchLeftHand.value);
            f_est[1,:]   = np.array(estimator_ft.contactWrenchRightHand.value);
            f_est[2,:]   = np.array(estimator_ft.contactWrenchLeftFoot.value);
            f_est[3,:]   = np.array(estimator_ft.contactWrenchRightFoot.value);
            if(i>estimationDelay):
                print ("Time",i,"f est", f_est[0,:], "f real", f[0,:]);
                err_f += abs(f_est - f);
                count += 1;
    
    print "*************************************************************************************************\n";
    print "Amplitude of force variation: %.2f" % (A);
    print "Frequency of force variation: %.2f" % (freq);
    print "Test duration: %.3f s" % (N*dt);
    print "Time step: %.3f s" % dt;
    print "Estimation delay: %.3f" % (estimationDelay*dt);
    if(count>0):
        print "Average percentual errors:";
        print (f_est/count);

def test_chirp(device, traj_gen, estimator_ft):
    device.after.addDownsampledSignal('estimator_ft.ftSensRightFootPrediction',1);
    print "Start linear chirp from %f to -1.0" % device.robotState.value[6+2];
    tt = 4;
    jid = 1;
    traj_gen.startLinChirp('rhr', -0.6, 0.2, 2.0, tt)
    N = int(tt/dt);
    qChirp = np.zeros(N+2);
    dqChirp = np.zeros(N+2);
    ddqChirp = np.zeros(N+2);
    q = np.zeros(N+2);
    dq = np.zeros(N+2);
    if(USE_ROBOT_VIEWER):
        viewer=robotviewer.client('XML-RPC');
        viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);
        
    for i in range(N+2):
        device.increment (dt);
        qChirp[i] = traj_gen.q.value[jid];
        dqChirp[i] = traj_gen.dq.value[jid];
        ddqChirp[i] = traj_gen.ddq.value[jid];
        q[i] = device.robotState.value[6+jid];
        dq[i] = device.jointsVelocities.value[jid];
#            print "q(%.3f)  = %.3f,  \tqDes   = %.3f" % (device.robotState.time*dt,device.robotState.value[6+jid],traj_gen.q.value[jid]);
#            print "dq(%.3f) = %.3f,  \tdqDes  = %.3f" % (device.robotState.time*dt,device.jointsVelocities.value[6+jid],traj_gen.dq.value[jid]);
#            print "tauDes(%.3f)= %.3f" % (device.robotState.time*dt,inv_dyn.tauDes.value[jid]);
#            sleep(0.1);
        if(USE_ROBOT_VIEWER and i%30==0):
            viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);
        if(i%100==0):
            print ("FT sensor right foot prediction: ", estimator_ft.ftSensRightFootPrediction.value);
#                print "q(%.3f) = %.3f, \tqDes = %.3f" % (device.robotState.time*dt,device.robotState.value[6+jid],traj_gen.q.value[jid]);
    
    dq_fd = np.diff(qChirp)/dt;
    ddq_fd = np.diff(dqChirp)/dt;
    (fig,ax) = create_empty_figure(3,1);
    ax[0].plot(qChirp,'r');  ax[0].plot(q,'b');
    ax[1].plot(dqChirp,'r'); ax[1].plot(dq,'b'); ax[1].plot(dq_fd,'g--');
    ax[2].plot(ddqChirp,'r'); ax[2].plot(ddq_fd,'g--');
    plt.show();
    

def test_min_jerk(device, traj_gen):
    print "\nGonna move joint to -1.5...";
    tt = 1.5;
    jid = 2;
    traj_gen.moveJoint('rhp', -1.5, tt);
    N = int(tt/dt);
    qMinJerk = np.zeros(N+2);
    dqMinJerk = np.zeros(N+2);
    ddqMinJerk = np.zeros(N+2);
    q = np.zeros(N+2);
    dq = np.zeros(N+2);
    if(USE_ROBOT_VIEWER):
        viewer=robotviewer.client('XML-RPC');
    for i in range(N+2):
        device.increment (dt);
        qMinJerk[i] = traj_gen.q.value[jid];
        dqMinJerk[i] = traj_gen.dq.value[jid];
        ddqMinJerk[i] = traj_gen.ddq.value[jid];
        q[i] = device.robotState.value[6+jid];
        dq[i] = device.jointsVelocities.value[jid];
        if(USE_ROBOT_VIEWER and i%30==0):
            viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);
        if(i%100==0):
            print "q(%.3f) = %.3f, \tqDes = %.3f" % (device.robotState.time*dt,device.robotState.value[6+jid],traj_gen.q.value[jid]);
    dq_fd = np.diff(qMinJerk)/dt;
    ddq_fd = np.diff(dqMinJerk)/dt;
    (fig,ax) = create_empty_figure(3,1);
    ax[0].plot(qMinJerk,'r');  ax[0].plot(q,'b');
    ax[1].plot(dqMinJerk,'r'); ax[1].plot(dq,'b'); ax[1].plot(dq_fd,'g--');
    ax[2].plot(ddqMinJerk,'r');  ax[2].plot(ddq_fd,'g--');
    plt.show();
    
def test_sinusoid(device, traj_gen):
    print "\nGonna start sinusoid to 0.2...";
    traj_gen.startSinusoid('rhp', 0.2, tt);
    qSin = np.zeros(4*N);
    dqSin = np.zeros(4*N);
    ddqSin = np.zeros(4*N);
    q = np.zeros(4*N);
    dq = np.zeros(4*N);
    for i in range(4*N):
        device.increment (dt);
        qSin[i] = traj_gen.q.value[jid];
        dqSin[i] = traj_gen.dq.value[jid];
        ddqSin[i] = traj_gen.ddq.value[jid];
        q[i] = device.robotState.value[6+jid];
        dq[i] = device.jointsVelocities.value[jid];
        if(USE_ROBOT_VIEWER and i%30==0):
            viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);
        if(i%100==0):
            print "q(%.3f) = %.3f, \tqDes = %.3f" % (device.robotState.time*dt,device.robotState.value[6+jid],device.control.value[jid]);
    traj_gen.stop('rhp');
    print "q(%.3f) = %f\n" % (device.robotState.time*dt,device.robotState.value[6+2]);
    dq_fd = np.diff(qSin)/dt;
    ddq_fd = np.diff(dqSin)/dt;
    (fig,ax) = create_empty_figure(3,1);
    ax[0].plot(qSin,'r');  ax[0].plot(q,'b');
    ax[1].plot(dqSin,'r'); ax[1].plot(dq,'b'); ax[1].plot(dq_fd,'g--');
    ax[2].plot(ddqSin,'r'); ax[2].plot(ddq_fd,'g--');
    plt.show();
    
    
def test_read_traj_file(device, traj_gen):
    FILE_NAME = 'climbing32_1ms.pos'
    FILE_PATH = '../share/joint_traj/';
    traj_gen.playTrajectoryFile(FILE_PATH+FILE_NAME);
    simulate(device, 4.1);
    print 'First part finished'
    traj_gen.playTrajectoryFile(FILE_PATH+FILE_NAME);
    simulate(device, 8.1);
    
def test_admittance_ctrl(device, ctrl_manager, traj_gen, estimator_ft, adm_ctrl, dt=0.001):
    if(USE_ROBOT_VIEWER):
        viewer=robotviewer.client('XML-RPC');
    N = 10*1000;
    axis = 1;
    fDes = np.zeros((N,6));
    f = np.zeros((N,6));
    ctrl_manager.setCtrlMode('all', 'adm');
    traj_gen.startForceSinusoid('lf',axis, 100, 1.5); #name, axis, final force, time
    adm_ctrl.fLeftFoot.value = 6*(0,);
    adm_ctrl.fRightFoot.value = 6*(0,);
    for i in range(N):
        if(i==1500):
            traj_gen.stopForce('lf');
            traj_gen.startForceSinusoid('lf',axis, -100, 1.5); #name, axis, final force, time
        device.increment (dt);
        sleep(dt);
        fDes[i,:] = traj_gen.fLeftFoot.value;
        f[i,:] = estimator_ft.contactWrenchLeftFoot.value;        
        if(USE_ROBOT_VIEWER and i%30==0):
            viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);
        if(i%100==0):
#            print "f(%.3f) = %.3f, \tfDes = %.3f" % (device.robotState.time*dt,f[i,axis],fDes[i,axis]);
            print ("fLeftFootError", adm_ctrl.fLeftFootError.value);
            print ("fRightFootError", adm_ctrl.fRightFootError.value);
            print ("dqDes", adm_ctrl.dqDes.value);
    (fig,ax) = create_empty_figure(3,1);
    ax[0].plot(fDes[:,0],'r'); ax[0].plot(f[:,0],'b');
    ax[1].plot(fDes[:,1],'r'); ax[1].plot(f[:,1],'b');
    ax[2].plot(fDes[:,2],'r'); ax[2].plot(f[:,2],'b');
    plt.show();
    
def test_force_jacobians(device,estimator_ft,torque_ctrl,traj_gen,ctrl_manager,inv_dyn, dt):
    if(USE_ROBOT_VIEWER):
        viewer=robotviewer.client('XML-RPC');
    ctrl_manager.setCtrlMode("rhr", "torque");
    ctrl_manager.setCtrlMode("rhp", "torque");
    ctrl_manager.setCtrlMode("rhy", "torque");
    ctrl_manager.setCtrlMode("rk", "torque");
    ctrl_manager.setCtrlMode("rap", "torque");
    ctrl_manager.setCtrlMode("rar", "torque");
    inv_dyn.Kp.value = NJ*(0.0,);
    inv_dyn.Kd.value = NJ*(0.0,);
    inv_dyn.Kf.value = (6*4)*(1.0,);
    N = 1; #10*1000;
    axis = 2;
    FORCE_MAX = 0.1;
    tauFB1 = np.zeros((N,30));
    tauFB2 = np.zeros((N,30));
    traj_gen.startForceSinusoid('rf',axis, FORCE_MAX, 1.5); #name, axis, final force, time
    for i in range(N):
        if(i==1500):
            traj_gen.stopForce('rf');
            traj_gen.startForceSinusoid('rf',axis, -FORCE_MAX, 1.5); #name, axis, final force, time
        device.increment (dt);
        sleep(dt);
        if(USE_ROBOT_VIEWER and i%100==0):
            viewer.updateElementConfig('hrp_device', list(device.state.value)+[0.0,]*10);
        inv_dyn.tauFB2.recompute(i);
        tauFB1[i,:] = inv_dyn.tauFB.value;
        tauFB2[i,:] = inv_dyn.tauFB2.value;
        
    (fig,ax) = create_empty_figure(3,1);
    ax[0].plot(tauFB1[:,0],'r'); ax[0].plot(tauFB2[:,0],'b--');
    ax[1].plot(tauFB1[:,1],'r'); ax[1].plot(tauFB2[:,1],'b--');
    ax[2].plot(tauFB1[:,2],'r'); ax[2].plot(tauFB2[:,2],'b--');
    (fig,ax) = create_empty_figure(3,1);
    ax[0].plot(tauFB1[:,3],'r'); ax[0].plot(tauFB2[:,3],'b--');
    ax[1].plot(tauFB1[:,4],'r'); ax[1].plot(tauFB2[:,4],'b--');
    ax[2].plot(tauFB1[:,5],'r'); ax[2].plot(tauFB2[:,5],'b--');
    plt.show();

def main(task='', dt=0.001, delay=0.01):
    np.set_printoptions(precision=2, suppress=True);
    device          = create_device(list(1.0/k_tau));
    traj_gen        = create_trajectory_generator(device, dt);
    (estimator_ft, estimator_kin)       = create_estimators(device, dt, delay, traj_gen);
    torque_ctrl     = create_torque_controller(device);
    pos_ctrl        = create_position_controller(device, dt);
    inv_dyn         = create_inverse_dynamics(device, dt);
    ctrl_manager    = create_ctrl_manager(device, dt);
    adm_ctrl        = create_admittance_ctrl(device, dt);
    tracer          = create_tracer(device, traj_gen, estimator_ft, estimator_kin, inv_dyn, torque_ctrl);
#    tracer.start();

    torque_ctrl.KpTorque.value = tuple(k_p);
    torque_ctrl.k_tau.value = tuple(k_tau);
    torque_ctrl.k_v.value   = tuple(k_v);
    inv_dyn.Kp.value = NJ*(1.0,); #10
    inv_dyn.Kd.value = NJ*(2.0,);  #5
    plug(device.jointsVelocities,       torque_ctrl.jointsVelocities);    
    plug(device.jointsVelocities,       inv_dyn.jointsVelocities);
    ctrl_manager.max_pwm.value = NJ*(500,);
    
    if(task=='test_admittance_control'):
        test_admittance_ctrl(device, ctrl_manager, traj_gen, estimator_ft, adm_ctrl, dt);
    elif(task=='test_read_traj_file'):
        device.increment (dt);
        test_read_traj_file(device, traj_gen);
    elif(task=='test_force_jacobians'):
        test_force_jacobians(device,estimator_ft,torque_ctrl,traj_gen,ctrl_manager,inv_dyn, dt);
        
        
#        q_des = randTuple(30);
#        traj_gen.moveJoint('rhr', q_des[0], 1.5);
#        test_vel_acc_estimator(device,estimator,dt, int(delay/dt));
        
#        test_chirp(device, traj_gen, estimator);
#        test_min_jerk(device, traj_gen);
#        test_sinusoid(device, traj_gen);
        
#    tracer.stop();
#    tracer.dump();
    return (device,estimator_ft, estimator_kin,torque_ctrl,traj_gen,ctrl_manager,inv_dyn,pos_ctrl,adm_ctrl,tracer);
    
if __name__=='__main__':
    (device,estimator_ft,estimator_kin,torque_ctrl,traj_gen,ctrl_manager,inv_dyn,pos_ctrl,adm_ctrl,tracer) = main('none',0.001,0.01);
    pass;
    
