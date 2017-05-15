# -*- coding: utf-8 -*-
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""
from IPython import embed
import numpy as np
import matplotlib.pyplot as plt
from dynamic_graph.sot.torque_control.utils.plot_utils import *
from compute_estimates_from_sensors import compute_estimates_from_sensors


DATA_SET = 1;
FOLDER_ID = 1;
EST_DELAY = 0.1;       ''' delay introduced by the estimation in seconds '''
NJ = 30;                ''' number of joints '''
DT = 0.001;             ''' sampling period '''
PLOT_DATA = False;
FORCE_ESTIMATE_RECOMPUTATION = True;
NEGLECT_GYROSCOPE = True;
NEGLECT_ACCELEROMETER = True;
SET_NORMAL_FORCE_RIGHT_FOOT_TO_ZERO = False;
USE_FT_SENSORS = True
JOINT_ID = np.array(range(12)); ''' IDs of the joints to save '''
if(DATA_SET==1):
    if(FOLDER_ID==1):
        data_folder = '../../../../../../../results/20160923_165916_Joint2_id_Kt/';
        JOINT_ID = np.array([2]);
    elif(FOLDER_ID==2):
        data_folder = '../../../../../../../results/20160923_170659_Joint2_id_Kv/';
        JOINT_ID = np.array([2]);
    elif(FOLDER_ID==3):
        data_folder = '../../../../../../../results/20160923_173001_Joint2_id_Ka/';
        JOINT_ID = np.array([2]);
if(DATA_SET==2):
    if(FOLDER_ID==1):
        #~ data_folder = '../../results/20160720_132041_rsp_torque_id/';
        #~ data_folder = '../../results/20160720_132429_rsp_torque_id/';
        data_folder = '../../../../../../../results/results/20160720_143905_rsp_torque_id_noGravity/';
        JOINT_ID = np.array([16]);
    elif(FOLDER_ID==2):
        data_folder = '../../../../../../../results/results/20160720_134957_rsp_friction_id_ext/';
        #~ data_folder = '../../../../../../../results/results/20160722_144631_rsp_const_vel/';
        JOINT_ID = np.array([16]);
    elif(FOLDER_ID==3):
        data_folder = '../../rien/';
        JOINT_ID = np.array([16]);


FILE_READ_SUCCEEDED = False;    
DATA_FILE_NAME = 'data';
TEXT_DATA_FILE_NAME = 'data.txt';
N_DELAY = int(EST_DELAY/DT);

file_name_ctrl    = 'dg_HRP2LAAS-control.dat';
file_name_enc     = 'dg_HRP2LAAS-robotState.dat';
file_name_acc     = 'dg_HRP2LAAS-accelerometer.dat';
file_name_gyro    = 'dg_HRP2LAAS-gyrometer.dat';
file_name_forceLA = 'dg_HRP2LAAS-forceLARM.dat';
file_name_forceRA = 'dg_HRP2LAAS-forceRARM.dat';
file_name_forceLL = 'dg_HRP2LAAS-forceLLEG.dat';
file_name_forceRL = 'dg_HRP2LAAS-forceRLEG.dat';
file_name_current = 'dg_HRP2LAAS-currents.dat';
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
    ctrl = np.empty((N,len(JOINT_ID)));
    dq = np.empty((N,len(JOINT_ID)));
    ddq = np.empty((N,len(JOINT_ID)));
    tau = np.empty((N,len(JOINT_ID)));

#    ptorques = np.empty((N,len(JOINT_ID)));
#    p_gains = np.empty((N,len(JOINT_ID)));
    current = np.empty((N,len(JOINT_ID)));
    
    FILE_READ_SUCCEEDED = True; 
    
    for i in range(len(JOINT_ID)):
        data = np.load(data_folder+DATA_FILE_NAME+'_j'+str(JOINT_ID[i])+'.npz');    
        ctrl[:,i] = data['ctrl'];
#        ptorques[:,i] = data['ptorque'];
#        p_gains[:,i] = data['p_gain'];
        current[:,i] = data['current']; 
        if(FORCE_ESTIMATE_RECOMPUTATION==False):
            dq [:,i] = data['dq'];
            ddq[:,i] = data['ddq'];
            tau[:,i] = data['tau'];

except (IOError, KeyError):
    print 'Gonna read text files...'
    
    ctrl    = np.loadtxt(data_folder+file_name_ctrl);
    enc     = np.loadtxt(data_folder+file_name_enc);
    acc     = np.loadtxt(data_folder+file_name_acc);
    gyro    = np.loadtxt(data_folder+file_name_gyro);
    forceLA = np.loadtxt(data_folder+file_name_forceLA);
    forceRA = np.loadtxt(data_folder+file_name_forceRA);
    forceLL = np.loadtxt(data_folder+file_name_forceLL);
    forceRL = np.loadtxt(data_folder+file_name_forceRL);
#    ptorques = np.loadtxt(data_folder+file_name_ptorque);
    current = np.loadtxt(data_folder+file_name_current);
#    p_gains = np.loadtxt(data_folder+file_name_p_gain);
    
    # check that largest signal has same length of smallest signal
    n_enc  = len(enc[:,0]);
    n_acc  = len(acc[:,0]);
    if(n_acc!=n_enc):
        print "Reducing size of signals from %d to %d" % (n_acc, n_enc);
    N = np.min([n_enc,n_acc]);

    time = enc[:N,0];
    ctrl = ctrl[:N,1:];
    ctrl = ctrl[:,JOINT_ID].reshape(N,len(JOINT_ID));
    current = current[:N,1:];
    current = current[:,JOINT_ID].reshape(N,len(JOINT_ID));
    enc  = enc[:N,7:];
    acc  = acc[:N,1:];
    gyro = gyro[:N,1:];
    forceLA = forceLA[:N,1:];
    forceRA = forceRA[:N,1:];
    forceLL = forceLL[:N,1:];
    forceRL = forceRL[:N,1:];
#    ptorques = ptorques[:N,1:];
#    p_gains = p_gains[:N,1:];
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
        a['acc']      = np.array([0, 0, 9.81]); #np.mean(acc,0);
    else:
        a['acc']      = acc;
    if(NEGLECT_GYROSCOPE==False):
        a['gyro']     = gyro;
    a['time']     = np.squeeze(time*DT);
    (tau, dq, ddq) = compute_estimates_from_sensors(a, EST_DELAY, USE_FT_SENSORS=USE_FT_SENSORS);
    
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
        np.savez(data_folder+DATA_FILE_NAME+'_j'+str(JOINT_ID[i])+'.npz', ctrl=ctrl[:,i], 
                 enc=enc[:,JOINT_ID[i]], tau=tau[:,i], dq=dq[:,i], ddq=ddq[:,i], current=current[:,i]);
                 #, ptorque=ptorques[:,i], p_gain=p_gains[:,i]);


#embed()
if(PLOT_DATA):
    ''' Plot data '''
    if(NEGLECT_ACCELEROMETER==False):
        plt.figure(); plt.plot(acc); plt.title('Acc');
    if(NEGLECT_GYROSCOPE==False):
        plt.figure(); plt.plot(gyro); plt.title('Gyro');
    #~ plt.figure(); plt.plot(forceLA); plt.title('Force Left Arm');
    plt.figure(); plt.plot(forceRA); plt.title('Force Right Arm'); plt.legend(['fx','fy','fz','mx','my','mz']);
    plt.plot(enc[:,JOINT_ID[i]], '--');
    #~ plt.figure(); plt.plot(forceLL); plt.title('Force Left Leg');
    #~ plt.figure(); plt.plot(forceRL); plt.title('Force Right Leg');
    
    for i in range(len(JOINT_ID)):
#        plt.figure(); plt.plot(enc[:,JOINT_ID[i]]-ctrl[:,i]); plt.title('Delta_q '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(dq[:,i]); plt.title('Joint velocity '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(ddq[:,i]); plt.title('Joint acceleration '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(tau[:,i]); plt.title('Joint torque '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(tau[:,i], ctrl[:,i], 'b. '); plt.title('Torque VS pwm '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(tau[:,i], current[:,i],'.'); plt.title('Torque VS current '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(tau[:,i], current[:,i],'.'); plt.title('Torque VS current '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(dq[:,i], current[:,i],'.'); plt.title('Velocity VS current '+str(JOINT_ID[i]));
        plt.figure(); plt.plot(ddq[:,i], current[:,i],'.'); plt.title('Velocity VS current '+str(JOINT_ID[i]));
#        plt.figure(); plt.plot(p_gains[:,i]); plt.title('Proportional gain '+str(JOINT_ID[i]));
        plt.show(); 

