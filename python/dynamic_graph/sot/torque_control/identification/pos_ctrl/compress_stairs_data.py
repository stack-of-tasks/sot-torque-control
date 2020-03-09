# -*- coding: utf-8 -*-
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""

import matplotlib.pyplot as plt
import numpy as np

import plot_utils as plut
from hrp2_motors_parameters import k_d, k_p, k_tau, k_v

FOLDER_ID = 5
EST_DELAY = 40
''' delay introduced by the estimation in number of samples '''
JOINT_ID = np.array([1, 2, 3, 4, 5])
# k_v = np.array([0.006332, 0.007, 0.006561, 0.006928, 0.006])
ZERO_VEL_THR = 0.0001
dt = 0.001

PLOT_TRACKING_ERROR = True
PLOT_TRAJECTORY_TRACKING = False

K_6 = 0.0
if (FOLDER_ID == 1):
    data_folder = '../results/20150324_151451_stairs_pos/'
elif (FOLDER_ID == 2):
    data_folder = '../results/20150325_170301_stairs_tc_gain_100/'
    K_6 = 1.0
elif (FOLDER_ID == 3):
    data_folder = '../results/20150325_175521_stairs_tc_gain_50/'
    K_6 = 0.5
elif (FOLDER_ID == 4):
    data_folder = '../results/20150325_181352_stairs_tc_gain_25/'
    K_6 = 0.25
elif (FOLDER_ID == 5):
    data_folder = '../results/20150325_183800_stairs_tc_gain_10/'
    K_6 = 0.1

plut.SAVE_FIGURES = True
plut.FIGURE_PATH = data_folder
SHOW_LEGEND = False
SHOW_PLOTS = False

DATA_FILE_NAME = 'data.npz'
TEXT_DATA_FILE_NAME = 'data.txt'

file_name_qDes = 'dg_jtc-jointsPositionsDesired.dat'
file_name_enc = 'dg_HRP2LAAS-robotState.dat'
file_name_qRef = 'dg_jtg-q.dat'
file_name_dqRef = 'dg_jtg-dq.dat'
file_name_ddqRef = 'dg_jtg-ddq.dat'
file_name_dq = 'dg_estimator-jointsVelocities.dat'
file_name_ddq = 'dg_estimator-jointsAccelerations.dat'
file_name_tau = 'dg_estimator-jointsTorques.dat'
file_name_delta_q_ff = 'dg_jtc-deltaQ_ff.dat'
file_name_delta_q_fb = 'dg_jtc-deltaQ_fb.dat'
file_name_delta_q_friction = 'dg_jtc-deltaQ_friction.dat'
''' Load data from file '''
try:
    data = np.load(data_folder + DATA_FILE_NAME)
    #    q = data['q'];
    enc = data['enc']
    qRef = data['qRef']
    dqRef = data['dqRef']
    dq = data['dq']
    tau = data['tau']
    qDes = data['qDes']
    if ('delta_q_ff' in data.keys()):
        delta_q_ff = data['delta_q_ff']
        delta_q_fb = data['delta_q_fb']
    else:
        delta_q_ff = np.zeros(enc.shape)
        delta_q_fb = np.zeros(enc.shape)
    N = len(enc[:, 0])
except (IOError, KeyError):
    print('Gonna read text files...')
    #    q       = np.loadtxt(data_folder+file_name_q);
    enc = np.loadtxt(data_folder + file_name_enc)
    qRef = np.loadtxt(data_folder + file_name_qRef)
    dqRef = np.loadtxt(data_folder + file_name_dqRef)
    dq = np.loadtxt(data_folder + file_name_dq)
    tau = np.loadtxt(data_folder + file_name_tau)
    qDes = np.loadtxt(data_folder + file_name_qDes)
    delta_q_ff = np.loadtxt(data_folder + file_name_delta_q_ff)
    delta_q_fb = np.loadtxt(data_folder + file_name_delta_q_fb)
    # check that signals have same length
    n_enc = len(enc[:, 0])
    n_qRef = len(qRef[:, 0])
    n_dqRef = len(dqRef[:, 0])
    n_dq = len(dq[:, 0])
    n_tau = len(tau[:, 0])
    n_qDes = len(qDes[:, 0])
    n_delta_q_ff = len(delta_q_ff[:, 0])
    n_delta_q_fb = len(delta_q_fb[:, 0])
    N = np.min([n_enc, n_qRef, n_dq, n_tau, n_qDes, n_delta_q_ff, n_delta_q_fb])
    if (n_enc != N):
        print('Gonna reduce size of encoder signal from %d to %d' % (n_enc, N))
        enc = enc[:N, :]
    if (n_qRef != N):
        print('Gonna reduce size of qRef signal from %d to %d' % (n_qRef, N))
        qRef = qRef[:N, :]
    if (n_dqRef != N):
        print('Gonna reduce size of dqRef signal from %d to %d' % (n_dqRef, N))
        dqRef = dqRef[:N, :]
    if (n_dq != N):
        print('Gonna reduce size of dq signal from %d to %d' % (n_dq, N))
        dq = dq[:N, :]
    if (n_tau != N):
        print('Gonna reduce size of tau signal from %d to %d' % (n_tau, N))
        tau = tau[:N, :]
    if (n_qDes != N):
        print('Gonna reduce size of qDes signal from %d to %d' % (n_qDes, N))
        qDes = qDes[:N, :]
    if (n_delta_q_ff != N):
        print('Gonna reduce size of delta_q_ff signal from %d to %d' % (n_delta_q_ff, N))
        delta_q_ff = delta_q_ff[:N, :]
    if (n_delta_q_fb != N):
        print('Gonna reduce size of delta_q_fb signal from %d to %d' % (n_delta_q_fb, N))
        delta_q_fb = delta_q_fb[:N, :]
    # synchronize qDes with other signals
    N = N - EST_DELAY
    for i in range(N):
        if (np.linalg.norm(dqRef[i, 1:]) > ZERO_VEL_THR):
            print('First sample with non-zero reference velocity is %d' % i)
            FIRST_SAMPLE = i - 100
            break

    for i in range(N):
        if (np.linalg.norm(dqRef[-i, 1:]) > ZERO_VEL_THR):
            print('Last sample with non-zero reference velocity is %d' % (N - i))
            LAST_SAMPLE = N - i + 100
            break

    # plt.figure(); plt.plot(np.linalg.norm(dqRef[:,1:])); plt.title('dqRef');

    N = LAST_SAMPLE - FIRST_SAMPLE

    dq = dq[FIRST_SAMPLE + EST_DELAY - 1:LAST_SAMPLE + EST_DELAY - 1:, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    tau = tau[FIRST_SAMPLE + EST_DELAY - 1:LAST_SAMPLE + EST_DELAY - 1:, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    enc = enc[FIRST_SAMPLE:LAST_SAMPLE, 6 + 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    qRef = qRef[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    dqRef = dqRef[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    qDes = qDes[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    delta_q_ff = delta_q_ff[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    delta_q_fb = delta_q_fb[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))

    np.savez(data_folder + DATA_FILE_NAME,
             dq=dq,
             tau=tau,
             qDes=qDes,
             enc=enc,
             qRef=qRef,
             dqRef=dqRef,
             delta_q_ff=delta_q_ff,
             delta_q_fb=delta_q_fb)
    ''' Save data as text file for loading it in matlab '''
#    np.savetxt(data_folder+TEXT_DATA_FILE_NAME, (dq,tau,qDes,enc,qRef));
''' Plot data '''
time = np.arange(0, N * dt, dt)
for i in range(len(JOINT_ID)):
    print('Max position tracking error for joint %d:         %f' %
          (JOINT_ID[i], np.max(np.abs(enc[:, i] - qRef[:, i]))))
    print('Avg Squared position tracking error for joint %d: %f' %
          (JOINT_ID[i], np.linalg.norm(enc[:, i] - qRef[:, i]) / N))

    if (PLOT_TRACKING_ERROR):
        plt.figure()
        plt.plot(time, 1e3 * (enc[:, i] - qRef[:, i]), rasterized=True)
        plt.xlabel('Time [s]')
        plt.ylabel(r'$q_j-q_j^d$ [$10^3$ rad]')
        if (JOINT_ID[i] == 3):
            plt.ylim([-65, 20])
        else:
            plt.ylim([-20, 20])
        title = 'Joint ' + str(JOINT_ID[i]) + ' pos track error'
        plut.saveCurrentFigure(title)
        plt.title(title)

    if (PLOT_TRAJECTORY_TRACKING):
        plt.figure()
        plt.plot(time, enc[:, i])
        plt.plot(time, qRef[:, i], 'r--', rasterized=True)
        plt.xlabel('Time [s]')
        plt.ylabel(r'$q_j$ [rad]')
        if (SHOW_LEGEND):
            leg = plt.legend([r'$q_j$', r'$q_j^d$'])
            leg.get_frame().set_alpha(plut.LEGEND_ALPHA)
        title = 'Joint ' + str(JOINT_ID[i]) + ' pos track'
        plut.saveCurrentFigure(title)
        plt.title(title)

    j = JOINT_ID[i]
    # compute delta_q_friction from velocity estimation
    delta_q_friction = k_v[j] * dq[:, i]
    # compute delta_q_fb from other components of delta_q (there was a bug in the c++ code computing delta_q_fb)
    delta_q_fb[:, i] = qDes[:, i] - enc[:, i] - delta_q_ff[:, i] - delta_q_friction
    delta_q_fb_pos = K_6 * (qRef[:, i] - enc[:, i])
    delta_q_fb_vel = k_tau[j] * (1 + k_p[j]) * k_d[j] * (dqRef[:, i] - dq[:, i])
    delta_q_fb_force = delta_q_fb[:, i] - delta_q_fb_pos - delta_q_fb_vel

    plt.figure()
    plt.plot(time, 1e3 * delta_q_ff[:, i], rasterized=True)
    plt.plot(time, 1e3 * delta_q_friction, rasterized=True)
    plt.plot(time, 1e3 * delta_q_fb_force, rasterized=True)
    plt.plot(time, 1e3 * delta_q_fb_pos, rasterized=True)
    #    plt.plot(time, 1e3*delta_q_fb_vel);
    plt.plot(time, 1e3 * (qDes[:, i] - enc[:, i]), '--', rasterized=True)
    plt.xlabel('Time [s]')
    plt.ylabel(r'$\Delta_q$ [$10^3$ rad]')
    if (SHOW_LEGEND):
        leg = plt.legend(['ff torque', 'ff friction', 'fb force', 'fb pos', 'total'], loc='upper left')
        leg.get_frame().set_alpha(plut.LEGEND_ALPHA)
    title = 'Joint ' + str(JOINT_ID[i]) + ' delta_q components'
    plut.saveCurrentFigure(title)
    plt.title(title)

#    delta_q_friction = k_v[i]*dq[:,i];
#    plt.figure(); plt.plot(time, delta_q_ff[:,i]);
#    plt.plot(time, delta_q_fb[:,i]);
#    plt.plot(time, delta_q_friction);
#    plt.plot(time, qDes[:,i]-enc[:,i]);
#    plt.xlabel('Time [s]');
#    plt.ylabel('Delta_q [rad]');
#    leg = plt.legend(['feedforward', 'feedback', 'friction', 'total']);
#    leg.get_frame().set_alpha(plut.LEGEND_ALPHA);
#    title = 'Joint '+str(JOINT_ID[i])+' delta_q components';
#    plut.saveCurrentFigure(title);
#    plt.title(title);

if (SHOW_PLOTS):
    plt.show()
