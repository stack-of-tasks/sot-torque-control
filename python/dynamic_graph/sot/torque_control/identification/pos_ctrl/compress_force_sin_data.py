# -*- coding: utf-8 -*-
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""

import matplotlib.pyplot as plt
import numpy as np

import plot_utils as plut
from hrp2_motors_parameters import k_d, k_p, k_s, k_tau

FOLDER_ID = 1
EST_DELAY = 40
""" delay introduced by the estimation in number of samples """
JOINT_ID = np.array([0, 1, 2, 3, 4, 5])
k_v = np.array([0.013, 0.006332, 0.007, 0.006561, 0.006928, 0.006])
ZERO_VEL_THR = 0.0001
dt = 0.001

PLOT_FORCE_TRACKING = True

LAST_SAMPLE = -1
if FOLDER_ID == 1:
    data_folder = "../results/20150326_101548_force_track_sin/"
    LAST_SAMPLE = 25 * 1000
elif FOLDER_ID == 2:
    data_folder = "../results/20150401_135207_moment_tracking/"
elif FOLDER_ID == 3:
    data_folder = "../results/20150401_135959_force_z_tracking/"

plut.SAVE_FIGURES = True
plut.FIGURE_PATH = data_folder
SHOW_PLOT = False
SHOW_LEGEND = True
DATA_FILE_NAME = "data.npz"
TEXT_DATA_FILE_NAME = "data.txt"

file_name_f = "dg_estimator-contactWrenchRightFoot.dat"
file_name_fRef = "dg_jtg-fRightFoot.dat"
file_name_qDes = "dg_jtc-jointsPositionsDesired.dat"
file_name_enc = "dg_HRP2LAAS-robotState.dat"
file_name_qRef = "dg_jtg-q.dat"
file_name_dq = "dg_estimator-jointsVelocities.dat"
file_name_tau = "dg_estimator-jointsTorques.dat"
file_name_tauDes = "dg_inv_dyn-tauDes.dat"
file_name_delta_q_ff = "dg_jtc-deltaQ_ff.dat"
file_name_delta_q_fb = "dg_jtc-deltaQ_fb.dat"
file_name_delta_q_friction = "dg_jtc-deltaQ_friction.dat"
""" Load data from file """
try:
    data = np.load(data_folder + DATA_FILE_NAME)
    f = data["f"]
    fRef = data["fRef"]
    enc = data["enc"]
    qDes = data["qDes"]
    qRef = data["qRef"]
    dq = data["dq"]
    tau = data["tau"]
    tauDes = data["tauDes"]
    delta_q_ff = data["delta_q_ff"]
    delta_q_fb = data["delta_q_fb"]
    N = len(enc[:, 0])
except (IOError, KeyError):
    print("Gonna read text files...")
    f = np.loadtxt(data_folder + file_name_f)
    fRef = np.loadtxt(data_folder + file_name_fRef)
    enc = np.loadtxt(data_folder + file_name_enc)
    qDes = np.loadtxt(data_folder + file_name_qDes)
    qRef = np.loadtxt(data_folder + file_name_qRef)
    dq = np.loadtxt(data_folder + file_name_dq)
    tau = np.loadtxt(data_folder + file_name_tau)
    tauDes = np.loadtxt(data_folder + file_name_tauDes)
    delta_q_ff = np.loadtxt(data_folder + file_name_delta_q_ff)
    delta_q_fb = np.loadtxt(data_folder + file_name_delta_q_fb)

    # check that signals have same length
    n_f = len(f[:, 0])
    n_fRef = len(fRef[:, 0])
    n_enc = len(enc[:, 0])
    n_qDes = len(qDes[:, 0])
    n_qRef = len(qRef[:, 0])
    n_dq = len(dq[:, 0])
    n_tau = len(tau[:, 0])
    n_tauDes = len(tauDes[:, 0])
    n_delta_q_ff = len(delta_q_ff[:, 0])
    n_delta_q_fb = len(delta_q_fb[:, 0])
    N = np.min(
        [
            n_f,
            n_fRef,
            n_enc,
            n_qRef,
            n_dq,
            n_tau,
            n_tauDes,
            n_qDes,
            n_delta_q_ff,
            n_delta_q_fb,
        ]
    )
    if n_f != N:
        print("Gonna reduce size of force signal from %d to %d" % (n_f, N))
        f = f[:N, :]
    if n_fRef != N:
        print("Gonna reduce size of force reference signal from %d to %d" % (n_fRef, N))
        fRef = fRef[:N, :]
    if n_enc != N:
        print("Gonna reduce size of encoder signal from %d to %d" % (n_enc, N))
        enc = enc[:N, :]
    if n_qRef != N:
        print("Gonna reduce size of qRef signal from %d to %d" % (n_qRef, N))
        qRef = qRef[:N, :]
    if n_dq != N:
        print("Gonna reduce size of dq signal from %d to %d" % (n_dq, N))
        dq = dq[:N, :]
    if n_tau != N:
        print("Gonna reduce size of tau signal from %d to %d" % (n_tau, N))
        tau = tau[:N, :]
    if n_tauDes != N:
        print("Gonna reduce size of tauDes signal from %d to %d" % (n_tauDes, N))
        tauDes = tauDes[:N, :]
    if n_qDes != N:
        print("Gonna reduce size of qDes signal from %d to %d" % (n_qDes, N))
        qDes = qDes[:N, :]
    if n_delta_q_ff != N:
        print("Gonna reduce size of delta_q_ff signal from %d to %d" % (n_delta_q_ff, N))
        delta_q_ff = delta_q_ff[:N, :]
    if n_delta_q_fb != N:
        print("Gonna reduce size of delta_q_fb signal from %d to %d" % (n_delta_q_fb, N))
        delta_q_fb = delta_q_fb[:N, :]

    # synchronize qDes with other signals
    FIRST_SAMPLE = 0
    if LAST_SAMPLE < 0:
        LAST_SAMPLE = N - EST_DELAY - 1
    N = LAST_SAMPLE - FIRST_SAMPLE

    f = f[FIRST_SAMPLE:LAST_SAMPLE, 1:].reshape(N, 6)
    fRef = fRef[FIRST_SAMPLE:LAST_SAMPLE, 1:].reshape(N, 6)
    dq = dq[FIRST_SAMPLE + EST_DELAY - 1 : LAST_SAMPLE + EST_DELAY - 1 :, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    tau = tau[FIRST_SAMPLE + EST_DELAY - 1 : LAST_SAMPLE + EST_DELAY - 1 :, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    tauDes = tauDes[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    enc = enc[FIRST_SAMPLE:LAST_SAMPLE, 6 + 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    qRef = qRef[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    qDes = qDes[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    delta_q_ff = delta_q_ff[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))
    delta_q_fb = delta_q_fb[FIRST_SAMPLE:LAST_SAMPLE, 1 + JOINT_ID].reshape(N, len(JOINT_ID))

    np.savez(
        data_folder + DATA_FILE_NAME,
        f=f,
        fRef=fRef,
        dq=dq,
        tau=tau,
        tauDes=tauDes,
        qDes=qDes,
        enc=enc,
        qRef=qRef,
        delta_q_ff=delta_q_ff,
        delta_q_fb=delta_q_fb,
    )
    """ Save data as text file for loading it in matlab """
#    np.savetxt(data_folder+TEXT_DATA_FILE_NAME, (dq,tau,qDes,enc,qRef));
""" Plot data """
time = np.arange(0, N * dt, dt)
max_f = np.max(np.array([fRef[:, :3], f[:, :3]]))
min_f = np.min(np.array([fRef[:, :3], f[:, :3]]))
max_f += 0.1 * (max_f - min_f)
min_f -= 0.1 * (max_f - min_f)
max_m = np.max(np.array([fRef[:, 3:], f[:, 3:]]))
min_m = np.min(np.array([fRef[:, 3:], f[:, 3:]]))
max_m += 0.1 * (max_m - min_m)
min_m -= 0.1 * (max_m - min_m)
for i in range(6):
    print("Max force tracking error for axis %d:     %f" % (i, np.max(np.abs(f[:, i] - fRef[:, i]))))
    print("Squared force tracking error for axis %d: %f" % (i, np.linalg.norm(f[:, i] - fRef[:, i]) / N))
    if PLOT_FORCE_TRACKING:
        plt.figure()
        plt.plot(time, f[:, i])
        plt.plot(time, fRef[:, i], "r--", rasterized=True)
        plt.xlabel("Time [s]")
        plt.ylabel("Force [N]")
        if i < 3:
            plt.ylim([min_f, max_f])
        else:
            plt.ylim([min_m, max_m])
        if i == 2:
            loc = "bottom right"
        else:
            loc = "best"
        if SHOW_LEGEND:
            leg = plt.legend(["Force", "Desired force"], loc=loc)
            leg.get_frame().set_alpha(plut.LEGEND_ALPHA)
        title = "Force axis " + str(i) + " f VS fRef"
        plut.saveCurrentFigure(title)
        plt.title(title)

for i in range(len(JOINT_ID)):
    # compute delta_q_friction from velocity estimation
    delta_q_friction = k_v[i] * dq[:, i]
    # compute delta_q_fb from other components of delta_q (there was a bug in the c++ code computing delta_q_fb)
    delta_q_fb[:, i] = qDes[:, i] - enc[:, i] - delta_q_ff[:, i] - delta_q_friction
    j = JOINT_ID[i]
    delta_q_fb_pos = k_tau[j] * (1 + k_p[j]) * k_s[j] * (enc[0, i] - enc[:, i])
    delta_q_fb_vel = k_tau[j] * (1 + k_p[j]) * k_d[j] * (-dq[:, i])
    delta_q_fb_force = delta_q_fb[:, i] - delta_q_fb_pos - delta_q_fb_vel

    plt.figure()
    plt.plot(time, 1e3 * delta_q_ff[:, i], rasterized=True)
    plt.plot(time, 1e3 * delta_q_friction, rasterized=True)
    plt.plot(time, 1e3 * delta_q_fb_force, rasterized=True)
    plt.plot(time, 1e3 * delta_q_fb_pos, rasterized=True)
    plt.plot(time, 1e3 * (qDes[:, i] - enc[:, i]), rasterized=True)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\Delta_q$ [$10^3$ rad]")
    if SHOW_LEGEND:
        leg = plt.legend(["ff torque", "ff friction", "fb force", "fb pos", "total"])
        leg.get_frame().set_alpha(plut.LEGEND_ALPHA)
    title = "Joint " + str(JOINT_ID[i]) + " delta_q components"
    plut.saveCurrentFigure(title)
    plt.title(title)

#    print('Max torque tracking error for joint %d:     %f' % (JOINT_ID[i], np.max(np.abs(tau[:,i]-tauDes[:,i])));)
#    print('Squared torque tracking error for joint %d: %f' % (JOINT_ID[i], np.linalg.norm(tau[:,i]-tauDes[:,i])/N);)
#    plt.figure(); plt.plot(tau[:,i]); plt.plot(tauDes[:,i],'r--'); plt.title('Joint '+str(JOINT_ID[i])+' tau VS
#    tauDes');

if SHOW_PLOT:
    plt.show()
