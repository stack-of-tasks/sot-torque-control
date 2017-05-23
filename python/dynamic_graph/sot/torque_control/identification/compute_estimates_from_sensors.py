# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 18:27:53 2015
Use the dynamic-graph entity ForceTorqueEstimator to estimate the joint velocities, accelerations
and torques starting from the following sensor data: f/t sensors, encoders, accelerometer, gyroscope

@author: adelpret
"""

USE_ROBOT_VIEWER = False;
PLOT_SENSOR_DATA = False;

from dynamic_graph.sot.torque_control.force_torque_estimator import ForceTorqueEstimator
from dynamic_graph.sot.torque_control.numerical_difference import NumericalDifference as VelAccEstimator
from load_hrpsys_log import load_hrpsys_log_astate
import numpy as np
import dynamic_graph.sot.torque_control.utils.plot_utils as plut
import matplotlib.pyplot as plt
if USE_ROBOT_VIEWER:
    import robotviewer  # start robotviewer from bash with 'robotviewer -sXML-RPC'.
    
def compute_delta_q_components(enc, delta_q, dq, delta_q_ff, qRef, dqRef, k_tau, k_v, k_p, k_s, k_d):
    NJ = enc.shape[1];
    N = enc.shape[0];
    delta_q_friction    = np.zeros((N,NJ));
    delta_q_fb_pos      = np.zeros((N,NJ));
    delta_q_fb_vel      = np.zeros((N,NJ));
    delta_q_fb_force    = np.zeros((N,NJ));
    for i in range(NJ):
        delta_q_friction[:,i]   = k_v[i]*dq[:,i];
        delta_q_fb_pos[:,i]     = k_tau[i]*(1+k_p[i])*k_s[i]*(qRef[:,i]-enc[:,i]);
        delta_q_fb_vel[:,i]     = k_tau[i]*(1+k_p[i])*k_d[i]*(dqRef[:,i]-dq[:,i]);
        delta_q_fb_force[:,i]   = delta_q[:,i] - delta_q_ff[:,i] - delta_q_fb_pos[:,i] - delta_q_fb_vel[:,i] - delta_q_friction[:,i];
    return (delta_q_friction, delta_q_fb_pos, delta_q_fb_vel, delta_q_fb_force);
        

def set_sensor_data_in_estimator(estimator_ft, estimator_kin, sensor_data):
    estimator_ft.base6d_encoders.value = tuple(6*[0.0,]+sensor_data['enc'][:30].tolist());
    estimator_kin.x.value = tuple(sensor_data['enc'][:30].tolist());
    estimator_ft.accelerometer.value   = tuple(sensor_data['acc'][0:3]);
    estimator_ft.gyroscope.value       = tuple(sensor_data['gyro']);
    estimator_ft.ftSensRightFoot.value = tuple(sensor_data['forceRL']);
    estimator_ft.ftSensLeftFoot.value  = tuple(sensor_data['forceLL']);
    estimator_ft.ftSensRightHand.value = tuple(sensor_data['forceRA']);
    estimator_ft.ftSensLeftHand.value  = tuple(sensor_data['forceLA']);
    

def compute_estimates_from_sensors(sensors, delay, ftSensorOffsets=None, USE_FT_SENSORS=True):
    NJ = 30;                                        # number of joints
    m = sensors['time'].shape[0];                           # number of time steps
    dt = float(np.mean(sensors['time'][1:]-sensors['time'][:-1])); # sampling period
    
    # Create and initialize estimator
    estimator_ft = ForceTorqueEstimator("estimator_ft"+str(np.random.rand()));
    estimator_kin = VelAccEstimator("estimator_kin"+str(np.random.rand()));
    estimator_ft.dqRef.value = NJ*(0.0,);
    estimator_ft.ddqRef.value = NJ*(0.0,);
    #Only use inertia model (not current) to estimate torques.
    estimator_ft.wCurrentTrust.value     = NJ*(0.0,);
    estimator_ft.currentMeasure.value    = NJ*(0.0,);
    estimator_ft.currentMeasure.value    = NJ*(0.0,);
    estimator_ft.saturationCurrent.value = NJ*(0.0,);
    estimator_ft.motorParameterKt_p.value  = tuple(30*[1.,])
    estimator_ft.motorParameterKt_n.value  = tuple(30*[1.,])
    estimator_ft.motorParameterKf_p.value  = tuple(30*[0.,])
    estimator_ft.motorParameterKf_n.value  = tuple(30*[0.,])
    estimator_ft.motorParameterKv_p.value  = tuple(30*[0.,])
    estimator_ft.motorParameterKv_n.value  = tuple(30*[0.,])
    estimator_ft.motorParameterKa_p.value  = tuple(30*[0.,])
    estimator_ft.motorParameterKa_n.value  = tuple(30*[0.,]) 
    
    set_sensor_data_in_estimator(estimator_ft, estimator_kin, sensors[0]);
    print "Time step: %f" % dt;
    print "Estimation delay: %f" % delay;
    if(ftSensorOffsets==None):
        estimator_ft.init(dt,delay,delay,delay,delay,True);
    else:
        estimator_ft.init(dt,delay,delay,delay,delay,False);
    estimator_kin.init(dt,NJ, delay);
    estimator_ft.setFTsensorOffsets(tuple(ftSensorOffsets));
    estimator_ft.setUseRawEncoders(False);
    estimator_ft.setUseRefJointVel(False);
    estimator_ft.setUseRefJointAcc(False);
    estimator_ft.setUseFTsensors(USE_FT_SENSORS);

    
    torques = np.zeros((m,NJ));
    dq      = np.zeros((m,NJ));
    ddq     = np.zeros((m,NJ));
    if(USE_ROBOT_VIEWER):
        viewer=robotviewer.client('XML-RPC');
    for i in range(m):
        if(USE_ROBOT_VIEWER and i%10==0):
            viewer.updateElementConfig('hrp_device', [0,0,0.7,0,0,0] + sensors['enc'][i,:].tolist());

        set_sensor_data_in_estimator(estimator_ft, estimator_kin, sensors[i]);
        estimator_ft.jointsTorques.recompute(i);
        torques[i,:] = np.array(estimator_ft.jointsTorques.value);
        dq[i,:]      = np.array(estimator_kin.dx.value);
        ddq[i,:]     = np.array(estimator_kin.ddx.value);
        
        if(i==2):
            print ("F/T sensor offsets are: ", estimator_ft.getFTsensorOffsets());

        if(i%1000==0):
            print 'Estimation time: \t %.3f' % (i*dt);
    
    return (torques, dq, ddq);
    
    
    
    
if __name__=='__main__':
    sensors = load_hrpsys_log_astate('/home/adelpret/devel/yarp_gazebo/src/motorFrictionIdentification/data/20140807-legTorqueId/legTorqueId_pos1-astate.log',
                                     'rad');
#    sensors = sensors[:2000];
    
    if(PLOT_SENSOR_DATA):
        print "Plot sensor data";
        for i in range(sensors['enc'].shape[1]):
            plut.plot_x_vs_y(sensors['time'], sensors['joint'][:,i], 'Joint angle '+str(i));
        
        f, ax = plt.subplots(3, 1, sharex=True);
        ax[0].set_title('Accelerometer');
        ax[0].plot(sensors['time'], sensors['acc'][:,0]);
        ax[1].plot(sensors['time'], sensors['acc'][:,1]);
        ax[2].plot(sensors['time'], sensors['acc'][:,2]);
        
        f, ax = plt.subplots(3, 1, sharex=True);
        ax[0].set_title('Gyroscope');
        ax[0].plot(sensors['time'], sensors['gyro'][:,0]);
        ax[1].plot(sensors['time'], sensors['gyro'][:,1]);
        ax[2].plot(sensors['time'], sensors['gyro'][:,2]);
        
        f, ax = plt.subplots(3, 2, sharex=True);
        ax[0,0].set_title('Right foot F/T sensor');
        ax[0,0].plot(sensors['time'], sensors['forceRL'][:,0]);
        ax[1,0].plot(sensors['time'], sensors['forceRL'][:,1]);
        ax[2,0].plot(sensors['time'], sensors['forceRL'][:,2]);
        ax[0,1].plot(sensors['time'], sensors['forceRL'][:,3]);
        ax[1,1].plot(sensors['time'], sensors['forceRL'][:,4]);
        ax[2,1].plot(sensors['time'], sensors['forceRL'][:,5]);
        
        f, ax = plt.subplots(3, 2, sharex=True);
        ax[0,0].set_title('Left foot F/T sensor');
        ax[0,0].plot(sensors['time'], sensors['forceLL'][:,0]);
        ax[1,0].plot(sensors['time'], sensors['forceLL'][:,1]);
        ax[2,0].plot(sensors['time'], sensors['forceLL'][:,2]);
        ax[0,1].plot(sensors['time'], sensors['forceLL'][:,3]);
        ax[1,1].plot(sensors['time'], sensors['forceLL'][:,4]);
        ax[2,1].plot(sensors['time'], sensors['forceLL'][:,5]);
        
        f, ax = plt.subplots(3, 2, sharex=True);
        ax[0,0].set_title('Right hand F/T sensor');
        ax[0,0].plot(sensors['time'], sensors['forceRA'][:,0]);
        ax[1,0].plot(sensors['time'], sensors['forceRA'][:,1]);
        ax[2,0].plot(sensors['time'], sensors['forceRA'][:,2]);
        ax[0,1].plot(sensors['time'], sensors['forceRA'][:,3]);
        ax[1,1].plot(sensors['time'], sensors['forceRA'][:,4]);
        ax[2,1].plot(sensors['time'], sensors['forceRA'][:,5]);
        
        f, ax = plt.subplots(3, 2, sharex=True);
        ax[0,0].set_title('Left hand F/T sensor');
        ax[0,0].plot(sensors['time'], sensors['forceLA'][:,0]);
        ax[1,0].plot(sensors['time'], sensors['forceLA'][:,1]);
        ax[2,0].plot(sensors['time'], sensors['forceLA'][:,2]);
        ax[0,1].plot(sensors['time'], sensors['forceLA'][:,3]);
        ax[1,1].plot(sensors['time'], sensors['forceLA'][:,4]);
        ax[2,1].plot(sensors['time'], sensors['forceLA'][:,5]);
        plt.show();
     
    print "Compute estimates from sensor data";       
    (torques, dq, ddq) = compute_estimates_from_sensors(sensors, 0.05);
    
    torques_kdl = np.loadtxt('/home/adelpret/devel/yarp_gazebo/src/motorFrictionIdentification/results/20140807-legTorqueId/withbacc/torqueEstimation_pos1.dat');
    
    for i in range(torques.shape[1]):
        print "Plot data joint %d out of %d" % (i, torques.shape[1]);
        f, ax = plt.subplots(4, 1, sharex=True);
        ax[0].plot(sensors['time'], torques[:,i], sensors['time'], torques_kdl[:,i]);
        ax[0].set_title('Torque joint '+str(i));
        ax[0].legend(['Metapod','kdl']);
        ax[1].plot(sensors['time'], sensors['enc'][:,i]);
        ax[1].set_title('Angle joint '+str(i));
        ax[2].plot(sensors['time'], dq[:,i]);
        ax[2].set_title('Velocity joint '+str(i));
        ax[3].plot(sensors['time'], ddq[:,i]);
        ax[3].set_title('Acceleration joint '+str(i));
        
    plt.show();
    
    
