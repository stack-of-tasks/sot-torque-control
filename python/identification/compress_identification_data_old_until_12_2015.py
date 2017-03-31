# -*- coding: utf-8 -*-
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""

import numpy as np
import matplotlib.pyplot as plt
from plot_utils import *
from compute_estimates_from_sensors import compute_estimates_from_sensors

FOLDER_ID = 10;
EST_DELAY = 0.02;       ''' delay introduced by the estimation in seconds '''
NJ = 30;                ''' number of joints '''
DT = 0.001;             ''' sampling period '''
PLOT_DATA = True;
FORCE_ESTIMATE_RECOMPUTATION = True;
NEGLECT_GYROSCOPE = True;
NEGLECT_ACCELEROMETER = True;
SET_NORMAL_FORCE_RIGHT_FOOT_TO_ZERO = False;
JOINT_ID = np.array(range(12)); ''' IDs of the joints to save '''

if(FOLDER_ID==1):
    data_folder = '../results/20150422_182432_rhp_torque_id/';    
elif(FOLDER_ID==2):
    data_folder = '../results/20150423_102257_rhp_torque_id/';
elif(FOLDER_ID==3):
    data_folder = '../results/20150423_173259_rhp_torque_id_contact/';
    JOINT_ID = np.array([2,3,4]);
elif(FOLDER_ID==4):
    data_folder = '../results/20150424_100759_rhp_torque_id_2ways/';
    JOINT_ID = np.array([2,3,4]);
elif(FOLDER_ID==5):
    data_folder = '../results/20150424_112301_rhp_torque_id_multi_conf/';
    JOINT_ID = np.array([2,3,4]);
elif(FOLDER_ID==6):
    data_folder = '../results/20150424_135254_rhp_torque_id_multi_norm_force/';
    JOINT_ID = np.array([2,3,4]);
elif(FOLDER_ID==7):
    data_folder = '../results/20150430_095211_rhp_id_zero_normal_force/';
    JOINT_ID = np.array([2,3,4]);
elif(FOLDER_ID==8):
    data_folder = '../results/20150430_103057_rhp_id_100_normal_force/';
    JOINT_ID = np.array([2,3,4]);
elif(FOLDER_ID==9):
    data_folder = '../results/20150505_113323_rhp_id_clamp/';
    JOINT_ID = np.array([2,3,4]);
elif(FOLDER_ID==10):
    data_folder = '../results/20160203_172245_id_rhp_pwm/';
    JOINT_ID = np.array([2]);
    
FILE_READ_SUCCEEDED = False;    
DATA_FILE_NAME = 'data';
TEXT_DATA_FILE_NAME = 'data.txt';
N_DELAY = int(EST_DELAY/DT);

file_name_qDes    = 'dg_HRP2LAAS-control.dat';
file_name_enc     = 'dg_HRP2LAAS-robotState.dat';
file_name_acc     = 'dg_HRP2LAAS-accelerometer.dat';
file_name_gyro    = 'dg_HRP2LAAS-gyrometer.dat';
file_name_forceLA = 'dg_HRP2LAAS-forceLARM.dat';
file_name_forceRA = 'dg_HRP2LAAS-forceRARM.dat';
file_name_forceLL = 'dg_HRP2LAAS-forceLLEG.dat';
file_name_forceRL = 'dg_HRP2LAAS-forceRLEG.dat';
file_name_ptorque = 'dg_HRP2LAAS-ptorque.dat';
file_name_current = 'dg_HRP2LAAS-currents.dat';
file_name_p_gain  = 'dg_HRP2LAAS-p_gains.dat';
    
''' Load data from file '''
try:
    data = np.load(data_folder+DATA_FILE_NAME+'.npz');
    time = data['time'];
    enc = data['enc'];
    acc = data['acc'];
    gyro = data['gyro'];
    forceLA = data['forceLA'];
    forceRA = data['forceRA'];
    forceLL = data['forceLL'];
    forceRL = data['forceRL'];

    N = acc.shape[0];    
    qDes = np.empty((N,len(JOINT_ID)));
    dq = np.empty((N,len(JOINT_ID)));
    ddq = np.empty((N,len(JOINT_ID)));
    tau = np.empty((N,len(JOINT_ID)));
    ptorques = np.empty((N,len(JOINT_ID)));
    p_gains = np.empty((N,len(JOINT_ID)));
    currents = np.empty((N,len(JOINT_ID)));
    
    FILE_READ_SUCCEEDED = True; 
    
    for i in range(len(JOINT_ID)):
        data = np.load(data_folder+DATA_FILE_NAME+'_j'+str(JOINT_ID[i])+'.npz');    
        qDes[:,i] = data['qDes'];
        ptorques[:,i] = data['ptorque'];
        p_gains[:,i] = data['p_gain'];
        currents[:,i] = data['current'];
        if(FORCE_ESTIMATE_RECOMPUTATION==False):
            dq[:,i] = data['dq'];
            ddq[:,i] = data['ddq'];
            tau[:,i] = data['tau'];

except (IOError, KeyError):
    print 'Gonna read text files...'
    
    qDes    = np.loadtxt(data_folder+file_name_qDes);
    enc     = np.loadtxt(data_folder+file_name_enc);
    acc     = np.loadtxt(data_folder+file_name_acc);
    gyro    = np.loadtxt(data_folder+file_name_gyro);
    forceLA = np.loadtxt(data_folder+file_name_forceLA);
    forceRA = np.loadtxt(data_folder+file_name_forceRA);
    forceLL = np.loadtxt(data_folder+file_name_forceLL);
    forceRL = np.loadtxt(data_folder+file_name_forceRL);
    ptorques = np.loadtxt(data_folder+file_name_ptorque);
    currents = np.loadtxt(data_folder+file_name_current);
    p_gains = np.loadtxt(data_folder+file_name_p_gain);
    
    # check that largest signal has same length of smallest signal
    n_enc  = len(enc[:,0]);
    n_acc  = len(acc[:,0]);
    if(n_acc!=n_enc):
        print "Reducing size of signals from %d to %d" % (n_acc, n_enc);
    N = np.min([n_enc,n_acc]);
    time = enc[:N,0];
    qDes = qDes[:N,1:];
    qDes = qDes[:,JOINT_ID].reshape(N,len(JOINT_ID));
    enc  = enc[:N,7:];
    acc  = acc[:N,1:];
    gyro = gyro[:N,1:];
    forceLA = forceLA[:N,1:];
    forceRA = forceRA[:N,1:];
    forceLL = forceLL[:N,1:];
    forceRL = forceRL[:N,1:];
    ptorques = ptorques[:N,1:];
    currents = currents[:N,1:];
    p_gains = p_gains[:N,1:];
    
    # save sensor data
    np.savez(data_folder+DATA_FILE_NAME+'.npz', 
             time=time, 
             enc=enc.reshape(N,NJ), 
             acc=acc.reshape(N,3), 
             gyro=gyro.reshape(N,3), 
             forceLA=forceLA.reshape(N,6), 
             forceRA=forceRA.reshape(N,6), 
             forceLL=forceLL.reshape(N,6), 
             forceRL=forceRL.reshape(N,6));


N = len(enc[:,0]);
if(FORCE_ESTIMATE_RECOMPUTATION or FILE_READ_SUCCEEDED==False):
    print 'Gonna estimate dq, ddq, tau';
    dt='f4';
    a = np.zeros(N, dtype=[ ('enc',dt,NJ)
                           ,('forceLA',dt,6)
                           ,('forceRA',dt,6)
                           ,('forceLL',dt,6)
                           ,('forceRL',dt,6)
                           ,('acc',dt,3)
                           ,('gyro',dt,3)
                           ,('time',dt,1)]);
    a['enc']      = enc;
    a['forceLA']  = forceLA;
    a['forceRA']  = forceRA;
    a['forceLL']  = forceLL;
    a['forceRL']  = forceRL;
    if(SET_NORMAL_FORCE_RIGHT_FOOT_TO_ZERO):
        a['forceRL'][:,2] = 0.0;
    if(NEGLECT_ACCELEROMETER):
        a['acc']      = np.mean(acc,0);
    else:
        a['acc']      = acc;
    if(NEGLECT_GYROSCOPE==False):
        a['gyro']     = gyro;
    a['time']     = np.squeeze(time*DT);
    (tau, dq, ddq) = compute_estimates_from_sensors(a, EST_DELAY);
    
    # shift estimate backward in time to compensate for estimation delay
    dq[:-N_DELAY,:]  = dq[N_DELAY::,:];
    ddq[:-N_DELAY,:] = ddq[N_DELAY::,:];
    tau[:-N_DELAY,:] = tau[N_DELAY::,:];
    # set last N_DELAY sample to constant value
    dq[-N_DELAY:,:]  = dq[-N_DELAY,:];
    ddq[-N_DELAY:,:] = ddq[-N_DELAY,:];
    tau[-N_DELAY:,:] = tau[-N_DELAY,:];
    # eliminate data of joints not to save
    dq   = dq[:,JOINT_ID].reshape(N,len(JOINT_ID));
    ddq  = ddq[:,JOINT_ID].reshape(N,len(JOINT_ID));
    tau  = tau[:,JOINT_ID].reshape(N,len(JOINT_ID));    
    
    for i in range(len(JOINT_ID)):
        np.savez(data_folder+DATA_FILE_NAME+'_j'+str(JOINT_ID[i])+'.npz', qDes=qDes[:,i], 
                 enc=enc[:,JOINT_ID[i]], tau=tau[:,i], dq=dq[:,i], ddq=ddq[:,i], current=currents[:,i],
                 ptorque=ptorques[:,i], p_gain=p_gains[:,i]);


if(PLOT_DATA):
    ''' Plot data '''
    plt.figure(); plt.plot(acc); plt.title('Acc');
    plt.figure(); plt.plot(gyro); plt.title('Gyro');
    plt.figure(); plt.plot(forceLA); plt.title('Force Left Arm');
    plt.figure(); plt.plot(forceRA); plt.title('Force Right Arm');
    plt.figure(); plt.plot(forceLL); plt.title('Force Left Leg');
    plt.figure(); plt.plot(forceRL); plt.title('Force Right Leg');
    
    for i in range(len(JOINT_ID)):
#        plt.figure(); plt.plot(enc[:,JOINT_ID[i]]-qDes[:,i]); plt.title('Delta_q '+str(JOINT_ID[i]));
#        plt.figure(); plt.plot(dq[:,i]); plt.title('Joint velocity '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(tau[:,i]); plt.title('Joint torque '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(tau[:,i], qDes[:,i]-enc[:,JOINT_ID[i]]); plt.title('Torque VS delta_q '+str(JOINT_ID[i]));
#        plt.figure(); plt.plot(tau[:,i], currents[:,i]); plt.title('Torque VS current '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(tau[:,i], ptorques[:,i]); plt.title('Torque VS pseudo-torque '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(ptorques[:,i], qDes[:,i]-enc[:,JOINT_ID[i]]); plt.title('Pseudo-torque VS delta_q '+str(JOINT_ID[i]));
#        plt.figure(); plt.plot(p_gains[:,i]); plt.title('Proportional gain '+str(JOINT_ID[i]));
        plt.show(); 

