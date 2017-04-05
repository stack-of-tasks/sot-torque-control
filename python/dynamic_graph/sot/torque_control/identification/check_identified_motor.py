# -*- coding: utf-8 -*-
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""

import numpy as np
from plot_utils import *
from hrp2_motors_parameters import *
import sys

FOLDER_ID = 4;

if(FOLDER_ID==1):
    data_folder = '../results/20150223_id_rhp/';
    JOINT_ID = np.array([2]);
    DELTA_Q_MAX = 0.015;
elif(FOLDER_ID==2):
    data_folder = '../results/20150223_191245_rhr/';
    JOINT_ID = np.array([1]);
    DELTA_Q_MAX = 0.015;
elif(FOLDER_ID==3):
    data_folder = '../results/20150223_193218_rhy/';
    JOINT_ID = np.array([0]);
    DELTA_Q_MAX = 0.025;
elif(FOLDER_ID==4):
    data_folder = '../results/20150223_194057_rk/';
    JOINT_ID = np.array([3]);
    DELTA_Q_MAX = 0.015;
elif(FOLDER_ID==5):
    data_folder = '../results/20150223_194647_rap/';
    JOINT_ID = np.array([4]);
    DELTA_Q_MAX = 0.025;
elif(FOLDER_ID==6):
    data_folder = '../results/20150223_195514_rar/';
    JOINT_ID = np.array([5]);
    DELTA_Q_MAX = 0.015;
elif(FOLDER_ID==7):
    data_folder = '../results/20150226_182400_lhp/';
    JOINT_ID = np.array([8]);
    DELTA_Q_MAX = 0.02;
elif(FOLDER_ID==8):
    data_folder = '../results/20150226_182400_rleg_multi_joint/';
    JOINT_ID = np.array([1,2,3]); # hip roll, pitch and knee
    DELTA_Q_MAX = 0.02;
elif(FOLDER_ID==9):
    data_folder = '../results/20150317_175631_rhp_chirp/';
    JOINT_ID = np.array([2]); # hip pitch chirp
    DELTA_Q_MAX = 0.02;

DATA_FILE_NAME = 'data.npz';

''' Load data from file '''
try:
    data = np.load(data_folder+DATA_FILE_NAME);
    N = data['q'].shape[0];
    NJ = len(JOINT_ID);
    q = data['q'].reshape((N,NJ));
    enc = data['enc'].reshape((N,NJ));
    dq = data['dq'].reshape((N,NJ));
    tau = data['tau'].reshape((N,NJ));
    qDes = data['qDes'].reshape((N,NJ));
except IOError:
    print "Impossible to read data file %f" % (data_folder+DATA_FILE_NAME);
    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");

for i in range(len(JOINT_ID)):
    delta_q = qDes[:,i]-enc[:,i]; #qDes-q;
    m = len(delta_q);
    
    ''' Display residuals '''
    x       = np.array([k_v[JOINT_ID[i]], k_tau[JOINT_ID[i]]]);
    A       = np.zeros((m,2));
    A[:,0]  = dq[:,i];
    A[:,1]  = tau[:,i];
    f, ax = plt.subplots(1,1,sharex=True);
    res = np.abs(np.dot(A,x)) - np.abs(delta_q);
    binwidth = (max(res) - min(res))/20.0;
    ax.hist(res, bins=np.arange(min(res), max(res) + binwidth, binwidth));
    ax.set_title('Residuals with asymmetric penalty');
    
    ''' Plot 3d surface '''
    x_grid = np.linspace(min(dq[:,i]), max(dq[:,i]), 15);
    y_grid = np.linspace(min(tau[:,i]), max(tau[:,i]), 15);
    X, Y = np.meshgrid(x_grid, y_grid);
    Z = x[0]*X + x[1]*Y;
    ax = plot3d(dq[:,i], tau[:,i], delta_q, 'Asymmetric ID - '+data_folder, ['dq','tau','delta q'],'r ');
    ax.plot_wireframe(X,Y,Z);
    
    ''' Plot 3d surface with only low-velocity data'''
    ind         = np.where(np.abs(dq[:,i])<0.01);
    dq2         = dq[:,i][ind];
    tau2        = tau[:,i][ind];
    delta_q2    = delta_q[ind];
    x_grid = np.linspace(min(dq2), max(dq2), 15);
    y_grid = np.linspace(min(tau2), max(tau2), 15);
    X, Y = np.meshgrid(x_grid, y_grid);
    Z = x[0]*X + x[1]*Y;
    ax = plot3d(dq2, tau2, delta_q2, 'Low velocity data '+data_folder, ['dq','tau','delta q'],'r ');
    ax.plot_wireframe(X,Y,Z);
    
    plt.show();