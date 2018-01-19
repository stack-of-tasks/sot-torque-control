# -*- coding: utf-8 -*-
"""
Created on Tue Oct  3 14:01:08 2017

@author: adelpret
"""

from dynamic_graph.sot.torque_control.madgwickahrs import MadgwickAHRS

dt = 0.001;
imu_filter = MadgwickAHRS('imu_filter');
imu_filter.init(dt);
imu_filter.setBeta(0.0)

for i in range(10):
    imu_filter.accelerometer.value = (0.0, 0.0, 9.8)
    imu_filter.gyroscope.value = (0.001, -1e-3, 1e-4)
    imu_filter.imu_quat.recompute(i);
    print imu_filter.imu_quat.value