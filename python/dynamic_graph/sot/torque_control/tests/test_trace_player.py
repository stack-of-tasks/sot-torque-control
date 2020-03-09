# -*- coding: utf-8 -*-
"""
Created on Tue Jun 20 13:46:05 2017

Simple test of the entity TraceReader
@author: adelpret
"""

from __future__ import print_function

import os

import matplotlib.pyplot as plt
import numpy as np
from dynamic_graph.sot.torque_control.trace_player import TracePlayer

DATA_PATH = os.getcwd() + "/../../../../../data/20170609_154139_poscrtl-pushes/"
DATA_FILE_NAMES = {
    "accelerometer": "dg_HRP2LAAS-accelerometer.dat",
    "gyrometer": "dg_HRP2LAAS-gyrometer.dat",
    "currents": "dg_HRP2LAAS-currents.dat",
    "forceLLEG": "dg_HRP2LAAS-forceLLEG.dat",
    "forceRLEG": "dg_HRP2LAAS-forceRLEG.dat",
    "forceLARM": "dg_HRP2LAAS-forceLARM.dat",
    "forceRARM": "dg_HRP2LAAS-forceRARM.dat",
    "robotState": "dg_HRP2LAAS-robotState.dat",
    #                   "control":       "dg_HRP2LAAS-control.dat",
    "mocap-chest": "dg_rosExportMocap-chest.dat"
}
N = 38000
RESET_TIME = 10000

print("Gonna read data files from path", DATA_PATH)

player = TracePlayer("player")
for signalName in DATA_FILE_NAMES:
    player.addOutputSignal(DATA_PATH + DATA_FILE_NAMES[signalName], signalName)

# CREATE EMPTY NUMPY ARRAYS TO CONTAIN DATA
data = {}
player.trigger.recompute(0)
# alternatively you can also use player.playNext();
for signalName in DATA_FILE_NAMES:
    v = player.signal(signalName).value
    print("Resize data for ", signalName, "to size", len(v))
    data[signalName] = np.empty((N, len(v)))

# FILL NUMPY ARRAYS WITH VALUES OF OUTPUT SIGNALS OF PLAYER
for t in range(N):
    if (t == RESET_TIME):
        player.rewind()
    player.trigger.recompute(t)
    # alternatively you can also use player.playNext();
    for signalName in data:
        data[signalName][t, :] = player.signal(signalName).value

# PLOT SOME DATA
for signalName in data:
    size = data[signalName].shape[1]
    if (size <= 3):
        f, ax = plt.subplots(size, 1)
    elif (size <= 6):
        f, ax = plt.subplots(size / 2, 2)
        ax = ax.reshape(size)
    else:
        continue

    for i in range(size):
        ax[i].plot(data[signalName][:, i])
    ax[0].set_title(signalName)
# plt.show();
