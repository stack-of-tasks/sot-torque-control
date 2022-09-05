# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 11:19:17 2015

Take a joint-trajectory file with a certain sampling time, interpolate it
to make at at 1 ms, compute first 2 derivatives, write a new trajectory file.
@author: adelpret
"""

from time import sleep

import matplotlib.pyplot as plt
import numpy as np

import robotviewer
from estimate_velocity import estimateVelocity

USE_ROBOT_VIEWER = False
DO_PLOTS = True
SAVE_FILE = True
FILE_NAME = "climbing32.pos"
NEW_FILE_NAME = "climbing32_1.25ms.pos"
FILE_PATH = "../../../../../share/"
NJ = 30
WS = 21
DT_DES = 0.00125
VIEWER_PERIOD = 50
MAX_LENGTH = 8.0

DQ_MAX = np.array(
    [
        3.5,
        3.8,
        4.4,
        3.9,
        5.0,
        9.1,
        3.5,
        3.8,
        4.4,
        3.9,
        5.0,
        9.1,
        4.4,
        2.3,
        6.8,
        6.8,
        4.1,
        2.4,
        4.1,
        2.2,
        4.7,
        3.1,
        5.6,
        4.1,
        2.4,
        4.1,
        2.2,
        4.7,
        3.1,
        5.6,
    ]
)
"""read the data """
data = np.loadtxt(FILE_PATH + FILE_NAME)
time = data[:, 0]
q = data[:, 1 : NJ + 1]
""" compute sampling time and length """
dt = time[1] - time[0]
T = time.shape[0]
""" display trajectory on robot viewer """
if USE_ROBOT_VIEWER:
    print("Play original trajectory")
    viewer = robotviewer.client("XML-RPC")
    for i in range(T):
        sleep(dt)
        if i % VIEWER_PERIOD == 0:
            viewer.updateElementConfig(
                "hrp_device",
                [0, 0, 0.7, 0, 0, 0]
                + q[i, :].tolist()
                + 10
                * [
                    0,
                ],
            )

print("Compute vel and acc")
dq = estimateVelocity(time, q, WS)
ddq = estimateVelocity(time, dq, WS)

# if(DO_PLOTS):
#    for j in range(NJ):
#        fig = plt.figure();
#        ax = fig.add_subplot(311); ax.plot(q[:,j]);   ax.set_title('pos');
#        ax = fig.add_subplot(312); ax.plot(dq[:,j]);  ax.set_title('vel');
#        ax = fig.add_subplot(313); ax.plot(ddq[:,j]); ax.set_title('acc');
#    plt.show();

print("Interpolate")
r = int(dt / DT_DES)
T_int = 1 + (T - 1) * r
time_int = np.arange(0, T_int * DT_DES, DT_DES)
q_int = np.zeros((T_int, NJ))
dq_int = np.zeros((T_int, NJ))
ddq_int = np.zeros((T_int, NJ))
for t in range(T - 1):
    #    print '%d'% (t*r+r-1);
    for t_int in range(r):
        q_int[t * r + t_int, :] = ((r - t_int) * q[t, :] + t_int * q[t + 1, :]) / r
        dq_int[t * r + t_int, :] = ((r - t_int) * dq[t, :] + t_int * dq[t + 1, :]) / r
        ddq_int[t * r + t_int, :] = ((r - t_int) * ddq[t, :] + t_int * ddq[t + 1, :]) / r
q_int[-1, :] = q[-1, :]
dq_int[-1, :] = dq[-1, :]
ddq_int[-1, :] = ddq[-1, :]

print("Save interpolated data as text file")
if T_int > MAX_LENGTH / DT_DES:
    print("Gonna remove final part of file")
    T_int = int(MAX_LENGTH / DT_DES)

if SAVE_FILE:
    data_int = np.zeros((T_int, NJ * 3))
    data_int[:, :NJ] = q_int[:T_int, :]
    data_int[:, NJ : 2 * NJ] = dq_int[:T_int, :]
    data_int[:, 2 * NJ :] = ddq_int[:T_int, :]
    np.savetxt(FILE_PATH + NEW_FILE_NAME, data_int, "%.8f")

for j in range(NJ):
    print(
        "Max velocity of joint %d as a percentage of its velocity limit: %f" % (j, 100 * np.max(dq[:, j]) / DQ_MAX[j])
    )

if DO_PLOTS:
    for j in range(NJ - 20):
        fig = plt.figure()
        ax = fig.add_subplot(311)
        ax.plot(time, q[:, j], "r")
        ax.plot(time_int, q_int[:, j], "b:")
        ax.set_title("pos interpolated")
        ax = fig.add_subplot(312)
        ax.plot(time, dq[:, j], "r")
        ax.plot(time_int, dq_int[:, j], "b:")
        ax.set_title("vel interpolated")
        ax = fig.add_subplot(313)
        ax.plot(time, ddq[:, j], "r")
        ax.plot(time_int, ddq_int[:, j], "b:")
        ax.set_title("acc interpolated")
    plt.show()

if USE_ROBOT_VIEWER:
    print("Play interpolated trajectory")
    viewer = robotviewer.client("XML-RPC")
    for i in range(T_int):
        sleep(DT_DES)
        if i % VIEWER_PERIOD == 0:
            viewer.updateElementConfig(
                "hrp_device",
                [0, 0, 0.7, 0, 0, 0]
                + q_int[i, :].tolist()
                + 10
                * [
                    0,
                ],
            )
