# -*- coding: utf-8 -*-
# flake8: noqa
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""

import numpy as np
import matplotlib.pyplot as plt
from plot_utils import *
from compute_estimates_from_sensors import compute_estimates_from_sensors

FOLDER_ID = 13

EST_DELAY = 50
''' delay introduced by the estimation in number of samples '''
DELTA_Q_MAX = 0.02
COMPUTE_TORQUES_AND_COMPARE = True

if (FOLDER_ID == 1):
    data_folder = '../results/20150223_id_rhp/'
    JOINT_ID = np.array([2])
    DELTA_Q_MAX = 0.015
elif (FOLDER_ID == 2):
    data_folder = '../results/20150223_191245_rhr/'
    JOINT_ID = np.array([1])
    DELTA_Q_MAX = 0.015
elif (FOLDER_ID == 3):
    data_folder = '../results/20150223_193218_rhy/'
    JOINT_ID = np.array([0])
    DELTA_Q_MAX = 0.025
elif (FOLDER_ID == 4):
    data_folder = '../results/20150223_194057_rk/'
    JOINT_ID = np.array([3])
    DELTA_Q_MAX = 0.015
elif (FOLDER_ID == 5):
    data_folder = '../results/20150223_194647_rap/'
    JOINT_ID = np.array([4])
    DELTA_Q_MAX = 0.025
elif (FOLDER_ID == 6):
    data_folder = '../results/20150223_195514_rar/'
    JOINT_ID = np.array([5])
    DELTA_Q_MAX = 0.015
elif (FOLDER_ID == 7):
    data_folder = '../results/20150226_182400_lhp/'
    JOINT_ID = np.array([8])
elif (FOLDER_ID == 8):
    data_folder = '../results/20150226_182400_rleg_multi_joint/'
    JOINT_ID = np.array([1, 2, 3])
    # hip roll, pitch and knee
elif (FOLDER_ID == 9):
    data_folder = '../results/20150317_175631_rhp_chirp/'
    JOINT_ID = np.array([2])
    # hip pitch chirp
elif (FOLDER_ID == 10):
    data_folder = '../results/20150322_184650_rhp/'
    JOINT_ID = np.array([2])
    # hip pitch chirp with higher delay and no F/T sensor in velocity part
    EST_DELAY = 70
elif (FOLDER_ID == 11):
    data_folder = '../results/20150322_190337_rhp/'
    JOINT_ID = np.array([2])
    # hip pitch chirp with higher delay and no F/T sensor in velocity part\
    EST_DELAY = 100
elif (FOLDER_ID == 12):
    data_folder = '../results/20150322_200613_rhp/'
    JOINT_ID = np.array([2])
    # hip pitch chirp with higher delay and no F/T sensor in velocity part
    EST_DELAY = 100
elif (FOLDER_ID == 13):
    data_folder = '../results/20150401_152647_id_rhp_chirp_force_0.1_to_5_Hz_in_55_sec/'
    JOINT_ID = np.array([2])
    # hip pitch chirp with 0 vel
    EST_DELAY = 100
elif (FOLDER_ID == 14):
    data_folder = '../results/20150401_153147_id_rhp_chirp_force_0.1_to_3_Hz_in_55_sec/'
    JOINT_ID = np.array([2])
    # hip pitch chirp with 0 vel
    EST_DELAY = 100

DATA_FILE_NAME = 'data.npz'
TEXT_DATA_FILE_NAME = 'data.txt'

if (FOLDER_ID < 9):
    file_name_q = 'dg_estimator-jointsPositions.dat'
    file_name_qDes = 'dg_jpc-positionDes.dat'
else:
    file_name_q = ''
    file_name_qDes = 'dg_jtc-jointsPositionsDesired.dat'

file_name_enc = 'dg_HRP2LAAS-robotState.dat'
file_name_dq = 'dg_estimator-jointsVelocities.dat'
file_name_tau = 'dg_estimator-jointsTorques.dat'
''' Load data from file '''
try:
    data = np.load(data_folder + DATA_FILE_NAME)
    #    q = data['q'];
    enc = data['enc']
    dq = data['dq']
    tau = data['tau']
    qDes = data['qDes']
except (IOError, KeyError):
    print 'Gonna read text files...'
    #    q       = np.loadtxt(data_folder+file_name_q);
    enc = np.loadtxt(data_folder + file_name_enc)
    dq = np.loadtxt(data_folder + file_name_dq)
    tau = np.loadtxt(data_folder + file_name_tau)
    qDes = np.loadtxt(data_folder + file_name_qDes)
    # check that signals have same length
    n_enc = len(enc[:, 0])
    n_dq = len(dq[:, 0])
    n_tau = len(tau[:, 0])
    n_qDes = len(qDes[:, 0])
    N = np.min([n_enc, n_dq, n_tau, n_qDes])
    if (n_enc != N):
        print 'Gonna reduce size of encoder signal from %d to %d' % (n_enc, N)
        enc = enc[:N, :]
    if (n_dq != N):
        print 'Gonna reduce size of dq signal from %d to %d' % (n_dq, N)
        dq = dq[:N, :]
    if (n_tau != N):
        print 'Gonna reduce size of tau signal from %d to %d' % (n_tau, N)
        tau = tau[:N, :]
    if (n_qDes != N):
        print 'Gonna reduce size of qDes signal from %d to %d' % (n_qDes, N)
        qDes = qDes[:N, :]
    # synchronize qDes with other signals
    N = N - EST_DELAY
    #    q    = q[EST_DELAY-1:-1,1+JOINT_ID].reshape(N,len(JOINT_ID));
    dq = dq[EST_DELAY - 1:-1:, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    tau = tau[EST_DELAY - 1:-1:, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    enc = enc[0:-EST_DELAY, 6 + 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    qDes = qDes[0:-EST_DELAY, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    delta_q = qDes - enc
    #    ind = np.where(np.abs(delta_q)<DELTA_Q_MAX);
    #    print 'Removed %d samples because delta_q too large' % (len(JOINT_ID)*N-len(ind[0]));
    #    plt.figure(); plt.plot(delta_q); plt.plot(np.array(len(delta_q)*[DELTA_Q_MAX,]),'r');
    #    plt.title('Delta q'); plt.show();
    #    q    = q[ind];
    #    dq   = dq[ind];
    #    tau  = tau[ind];
    #    qDes = qDes[ind];
    #    enc = enc[ind];
    np.savez(data_folder + DATA_FILE_NAME, dq=dq, tau=tau, qDes=qDes, enc=enc)
''' Save data as text file for loading it in matlab '''
np.savetxt(data_folder + TEXT_DATA_FILE_NAME, (dq, tau, qDes, enc))
''' Plot data '''
#plt.figure(); plt.plot(q); plt.plot(qDes,'r'); plt.title('q VS qDes (red)');
#plt.figure(); plt.plot(enc); plt.plot(qDes,'r'); plt.title('encoders VS qDes (red)');
for i in range(len(JOINT_ID)):
    #    plt.figure(); plt.plot(enc[:,i]-q[:,i]); plt.title('encoders - q');
    plt.figure()
    plt.plot(enc[:, i] - qDes[:, i])
    plt.title('encoders-qDes')
    plt.figure()
    plt.plot(dq[:, i])
    plt.title('Joint velocity')
    plt.figure()
    plt.plot(tau[:, i])
    plt.title('Joint torques')
    plt.figure()
    plt.plot(tau[:, i])
    plt.plot(5 * dq[:, i])
    plt.plot(1e3 * (enc[:, i] - qDes[:, i]))
    plt.title('tau VS dq VS delta_q')
    plt.legend(['tau', 'dq', 'delta q'])
    #    s = 10; # downsampling factor
    #    plt.figure(); plt.plot(tau[s::s,i]-tau[1:-s:s,i]); plt.title('tau[i]-tau[i-1]');
    #    plot3d(dq[:,i], tau[:,i], qDes[:,i]-enc[:,i], 'Joint '+str(JOINT_ID[i])+' '+data_folder, ['dq','tau','delta q'],'r ');
    plt.show()
