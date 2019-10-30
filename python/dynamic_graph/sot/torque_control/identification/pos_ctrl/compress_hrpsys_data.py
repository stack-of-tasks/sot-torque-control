# -*- coding: utf-8 -*-
# flake8: noqa
"""
Created on Wed Apr 22 11:36:11 2015

Compress old data in the log format of hrpsys in a format that is compatible with the new data
collected using the RealTimeTracer.

@author: adelpret
"""

import matplotlib.pyplot as plt
import numpy as np

from compute_estimates_from_sensors import compute_estimates_from_sensors
from load_hrpsys_log import load_hrpsys_log_astate, load_hrpsys_log_rstate

out_data_folder = '../results/20140807-legTorqueId1/'
OUT_DATA_FILE_NAME = 'data.npz'
A_STATE_FILE = '/home/adelpret/devel/yarp_gazebo/src/motorFrictionIdentification/data/20140807-legTorqueId/legTorqueId_pos1-astate.log'
R_STATE_FILE = '/home/adelpret/devel/yarp_gazebo/src/motorFrictionIdentification/data/20140807-legTorqueId/legTorqueId_pos1-rstate.log'
ESTIMATION_DELAY = 0.2
COMPUTE_TORQUES_WITHOUT_GYRO = False

sensors = load_hrpsys_log_astate(A_STATE_FILE, 'rad')
ref = load_hrpsys_log_rstate(R_STATE_FILE, 'rad')

#sensors = sensors[:5000];
#ref     = ref[:5000];

(torques, dq, ddq) = compute_estimates_from_sensors(sensors, ESTIMATION_DELAY)

if (COMPUTE_TORQUES_WITHOUT_GYRO):
    sensors['gyro'] = np.zeros(sensors['gyro'].shape)
    (torques_no_gyro, dq, ddq) = compute_estimates_from_sensors(sensors, ESTIMATION_DELAY)

for i in range(12):  #torques.shape[1]):
    print "Plot data joint %d out of %d" % (i, torques.shape[1])
    f, ax = plt.subplots(1, 1, sharex=True)
    ax.plot(sensors['time'], sensors['torque'][:, i], 'b')
    delta_q = ref['enc'][:, i] - sensors['enc'][:, i]
    scale = np.mean(sensors['torque'][:, i]) / np.mean(delta_q)
    ax.plot(sensors['time'], scale * delta_q, 'r--')
    ax.plot(sensors['time'], torques[:, i], 'g--')
    ax.legend(['hrpsys', 'delta_q', 'torque'])
    ax.set_title('torque hrpsys')

    f, ax = plt.subplots(3, 1, sharex=True)
    ax[0].plot(sensors['time'], torques[:, i])
    if (COMPUTE_TORQUES_WITHOUT_GYRO):
        ax[0].plot(sensors['time'], torques_no_gyro[:, i])
    ax[0].set_title('Torque joint ' + str(i))
    ax[1].plot(sensors['time'], sensors['enc'][:, i])
    ax[1].plot(sensors['time'], sensors['enc'][:, i] - ref['enc'][:, i])
    ax[1].set_title('Angle joint ' + str(i))
    ax[2].plot(sensors['time'], dq[:, i])
    ax[2].set_title('Velocity joint ' + str(i))

ax[1].legend(['Angle', 'Delta_q'])
if (COMPUTE_TORQUES_WITHOUT_GYRO):
    ax[0].legend(['Torque w/ gyro', 'Torque w/o gyro'])

plt.show()

DT = float(sensors['time'][1] - sensors['time'][0])
# sampling period
LOST_SAMPLES = int(ESTIMATION_DELAY / DT)
print "Gonna shift data of %d samples to compensate for estimation delay" % LOST_SAMPLES

if (COMPUTE_TORQUES_WITHOUT_GYRO):
    np.savez(out_data_folder + OUT_DATA_FILE_NAME,
             dq=dq[:-LOST_SAMPLES, :],
             tau=torques_no_gyro[:-LOST_SAMPLES, :],
             qDes=ref['enc'][LOST_SAMPLES:, :30],
             enc=sensors['enc'][LOST_SAMPLES:, :30])
else:
    np.savez(out_data_folder + OUT_DATA_FILE_NAME,
             dq=dq[:-LOST_SAMPLES, :],
             tau=torques[:-LOST_SAMPLES, :],
             qDes=ref['enc'][LOST_SAMPLES:, :30],
             enc=sensors['enc'][LOST_SAMPLES:, :30])
