# -*- coding: utf-8 -*-
import numpy as np
import sys
import dynamic_graph.sot.torque_control.utils.plot_utils as plot_utils
import matplotlib as mpl
mpl.rcParams['lines.linewidth']     = 4;
import matplotlib.pyplot as plt
from motor_model import Motor_model
from dynamic_graph.sot.torque_control.hrp2.control_manager_conf import IN_OUT_GAIN
from identify_motor_static import identify_motor_static
from identify_motor_low_level import identify_motor_low_level
from identify_motor_vel import identify_motor_vel
from identify_motor_acc import identify_motor_acc
from identification_utils import jID
'''
motor model :
i(t) = Kt*tau(t) + Kv*dq(t) + Ka*ddq(t) + Kf*Sign(dq)
'''
#~ DZ=70
DZ                                  = 0.0
dt                                  = 0.001
ZERO_VEL_THRESHOLD                  = 0.1
POSITIVE_VEL_THRESHOLD              = 0.001
ZERO_ACC_THRESHOLD                  = 1.
ZERO_JERK_THRESHOLD                 = 3.0
CURRENT_SATURATION                  = 9.5
Nvel                                = 10
SHOW_THRESHOLD_EFFECT               = True
USING_CONTROL_AS_CURRENT_MEASURE    = False
INVERT_CURRENT                      = False
result_dir                          = '../../../../../results/hrp2_motor_identification/'
IDENTIFICATION_MODE ='static'
IDENTIFICATION_MODE ='vel'
#IDENTIFICATION_MODE ='acc'
#IDENTIFICATION_MODE ='test_model' #Compare Model Vs Measurment

#JOINT_NAME = 'rhy';  
#JOINT_NAME = 'rhr'; 
#JOINT_NAME = 'rhp'; 
#JOINT_NAME = 'rk'; 
#JOINT_NAME = 'rap'; 
JOINT_NAME = 'rar'; 

#JOINT_NAME = 'lhy';  USING_CONTROL_AS_CURRENT_MEASURE = True  # 6   
#JOINT_NAME = 'lhr';  USING_CONTROL_AS_CURRENT_MEASURE = True  # 7   
#JOINT_NAME = 'lhp';  USING_CONTROL_AS_CURRENT_MEASURE = True  # 8 
#JOINT_NAME = 'lk';  # 9 ok
#JOINT_NAME = 'lap'; # 10 ok
#JOINT_NAME = 'lar'; # 11 ok

#data_folder='../../results/20161114_153220_rk_vel/'
#data_folder='../../results/20161114_151812_rhp_vel/'
#data_folder='../../results/20170203_164133_com_sin_z_001/'

#JOINT_NAME = 'rhp'
#data_folder='../../results/20170203_164133_com_sin_z_001/'
#data_folder= '../../results/20161114_152706_rk_acc/' ; INVERT_CURRENT = True
#data_folder = result_dir + '../20170911172000_com_sin_3cm/';

if (IDENTIFICATION_MODE != 'test_model') :
    if(JOINT_NAME == 'rhy' ):
        INVERT_CURRENT = True
        Nvel = 9
        data_folder_static = result_dir+'20161114_135332_rhy_static/';
        data_folder_vel    = result_dir+'20161114_143152_rhy_vel/';
        data_folder_acc    = result_dir+'20161114_142351_rhy_acc/';
    if(JOINT_NAME == 'rhr' ):
        INVERT_CURRENT = True
        Nvel = 10
        data_folder_static = result_dir+'20161114_144232_rhr_static/';
        data_folder_vel    = result_dir+'20161114_150356_rhr_vel/';
        data_folder_acc    = result_dir+'20161114_145456_rhr_acc/';
    if(JOINT_NAME == 'rhp' ):
        data_folder_static = result_dir+'20161114_150722_rhp_static/';
        data_folder_vel    = result_dir+'20161114_151812_rhp_vel/';
        data_folder_acc    = result_dir+'20161114_151259_rhp_acc/';
    if(JOINT_NAME == 'rk' ):
        INVERT_CURRENT = True
        data_folder_static = result_dir+'20161114_152140_rk_static/';
#        data_folder_static = result_dir+'../20170918_180023_rk_current_chirp/'
        data_folder_vel    = result_dir+'20161114_153220_rk_vel/';
        data_folder_acc    = result_dir+'20161114_152706_rk_acc/';
    if(JOINT_NAME == 'rap' ):
        INVERT_CURRENT = True
        data_folder_static = result_dir+'20161114_153739_rap_static/';
        data_folder_vel    = result_dir+'20161114_154559_rap_vel/';
        data_folder_acc    = result_dir+'20161114_154316_rap_acc/';
    if(JOINT_NAME == 'rar' ):
        data_folder_static = result_dir+'20161114_154945_rar_static/';
        data_folder_vel    = result_dir+'20161114_160038_rar_vel/';
        data_folder_acc    = result_dir+'20161114_155545_rar_acc/';

    if(JOINT_NAME == 'lhy' ):
        data_folder_static = result_dir+'20170113_144220_lhy_static/';
        data_folder_vel    = result_dir+'/';
        data_folder_acc    = result_dir+'20170113_144710_lhy_const_acc/';
    if(JOINT_NAME == 'lhr' ):
        data_folder_static = result_dir+'20170113_145227_lhr_static/';
        data_folder_vel    = result_dir+'20170113_150215_lhr_const_vel/';
        data_folder_acc    = result_dir+'20170113_145826_lhr_const_acc/';
    if(JOINT_NAME == 'lhp' ):
        data_folder_static = result_dir+'20170113_150628_lhp_static/';
        data_folder_vel    = result_dir+'20170113_151433_lhp_const_vel/';
        data_folder_acc    = result_dir+'20170113_151103_lhp_const_acc/';
    if(JOINT_NAME == 'lk' ):
        data_folder_static = result_dir+'20170113_151748_lk_static/';
        data_folder_vel    = result_dir+'20170113_152924_lk_const_vel/';
        data_folder_acc    = result_dir+'20170113_152606_lk_const_acc/';
    if(JOINT_NAME == 'lap' ):
        data_folder_static = result_dir+'20170113_154007_lap_static/';
        data_folder_vel    = result_dir+'20170113_154834_lap_const_vel/';
        data_folder_acc    = result_dir+'20170113_154303_lap_const_acc/';
    if(JOINT_NAME == 'lar' ):
        data_folder_static = result_dir+'20170113_155150_lar_static/';
        data_folder_vel    = result_dir+'20170113_160057_lar_const_vel/';
        data_folder_acc    = result_dir+'20170113_155706_lar_const_acc/';

if (IDENTIFICATION_MODE=='static') : data_folder = data_folder_static
if (IDENTIFICATION_MODE=='vel')    : data_folder = data_folder_vel
if (IDENTIFICATION_MODE=='acc')    : data_folder = data_folder_acc

JOINT_ID = jID[JOINT_NAME]
DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';

''' Load data from file '''
try:
    data = np.load(data_folder+DATA_FILE_NAME);
    enc = np.squeeze(data['enc']);
    dq = np.squeeze(data['dq']);
    ddq = np.squeeze(data['ddq']);
    tau = np.squeeze(data['tau']);
    ctrl = np.squeeze(data['ctrl']);
    current = np.squeeze(data['current']);
except IOError:
    print "Impossible to read data file %s" % (data_folder+DATA_FILE_NAME);
    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");
        
maskSaturation=np.logical_and( (current>-CURRENT_SATURATION) , (current<CURRENT_SATURATION))
if USING_CONTROL_AS_CURRENT_MEASURE:
    maskCtrlNotInDZ = np.logical_and( ctrl >= (DZ *0.8) ,ctrl <= -(DZ*0.8)  )
    maskSaturation=np.logical_or(maskSaturation, maskCtrlNotInDZ)

if INVERT_CURRENT:
    current = - current

enc     = enc    [maskSaturation];
dq      = dq     [maskSaturation];
ddq     = ddq    [maskSaturation];
tau     = tau    [maskSaturation];
ctrl    = ctrl   [maskSaturation];
current = current[maskSaturation];
   
if USING_CONTROL_AS_CURRENT_MEASURE:
    current_est = current.copy()
    #~ maskUpDZ = ctrl > DZ
    #~ maskDnDZ = ctrl < DZ
    #~ maskInDZ = np.logical_and( ctrl <= DZ ,ctrl >= -DZ )
    current = ctrl / IN_OUT_GAIN
    #~ current[maskUpDZ] =  (ctrl[maskUpDZ]-DZ) /IN_OUT_GAIN 
    #~ current[maskDnDZ] =  (ctrl[maskDnDZ]+DZ) /IN_OUT_GAIN 
    #~ current[maskInDZ] =  0.0

#IDENTIFICATION_MODE='low_level'
if(IDENTIFICATION_MODE=='low_level'):
    identify_motor_low_level(dq, ctrl, current);
    
#Ktau,Tau0 Identification
if(IDENTIFICATION_MODE=='static'):
    (Ktp, Ktn, Ks, DZ) = identify_motor_static(enc, dq, ctrl, current, tau, JOINT_ID, JOINT_NAME, ZERO_VEL_THRESHOLD, 
                                               POSITIVE_VEL_THRESHOLD, SHOW_THRESHOLD_EFFECT);
    #save parameters for next identification level**********************
    np.savez(data_folder+'motor_param_'+JOINT_NAME+'.npz',Ktp=Ktp, Ktn=Ktn, Ks=Ks, DZ=DZ)
    plt.savefig(data_folder+"static_"+JOINT_NAME+".jpg")
    plt.show()

if(IDENTIFICATION_MODE=='vel' or IDENTIFICATION_MODE=='acc'):
    #load parameters from last identification level*********************
    try:
        data_motor_param = np.load(data_folder_static+'motor_param_'+JOINT_NAME+'.npz')
        Ktp=(data_motor_param['Ktp'].item())
        Ktn=(data_motor_param['Ktn'].item())
    except IOError:
        print "Impossible to read data file %s" % (data_folder_static+'motor_param_'+JOINT_NAME+'.npz');
        sys.exit("Run identification on static experiments.");

#Kd Identification         
if(IDENTIFICATION_MODE=='vel'):
    Ks =(data_motor_param['Ks'].item())
    (Kvp, Kvn, Kfp, Kfn, DeadZone, K_bemf) = identify_motor_vel(dt, dq, ddq, ctrl, current, tau, Ktp, Ktn, Ks,
                                                              ZERO_VEL_THRESHOLD, ZERO_ACC_THRESHOLD, 
                                                              Nvel, SHOW_THRESHOLD_EFFECT);
    np.savez(data_folder+'motor_param_'+JOINT_NAME+'.npz', Ktp=Ktp, Ktn=Ktn, Kvp=Kvp, Kvn=Kvn,
             Kfp=Kfp, Kfn=Kfn, DeadZone=DeadZone, K_bemf=K_bemf)
    warning = ""
    if USING_CONTROL_AS_CURRENT_MEASURE :
        warning = " (Current sensor not used)"
    
    print "cur_sens_gain[%d] = %f #Using %s"% (JOINT_ID, Ks, data_folder_static.split('/')[-2] + warning);    
    print "deadzone[%d]      = %f #Using %s"% (JOINT_ID, data_motor_param['DZ'].item(), data_folder_static.split('/')[-2] + warning);    
    print "deadzone[%d]      = %f #Using %s"% (JOINT_ID, DeadZone, data_folder_vel.split('/')[-2] + warning);
    print "K_bemf[%d]        = %f # [Amp/Rad.s-1] Using %s"% (JOINT_ID, K_bemf, data_folder_vel.split('/')[-2] + warning);
    print 'Kt_p[%d]          = %f #Using %s' % (JOINT_ID, Ktp, data_folder_static.split('/')[-2] + warning);
    print 'Kt_n[%d]          = %f #Using %s' % (JOINT_ID, Ktn, data_folder_static.split('/')[-2] + warning);
    print 'Kv_p[%d]          = %f #Using %s' % (JOINT_ID, Kvp, data_folder_vel.split('/')[-2] + warning);
    print 'Kv_n[%d]          = %f #Using %s' % (JOINT_ID, Kvn, data_folder_vel.split('/')[-2] + warning);
    print 'Kf_p[%d]          = %f #Using %s' % (JOINT_ID, Kfp, data_folder_vel.split('/')[-2] + warning);
    print 'Kf_n[%d]          = %f #Using %s' % (JOINT_ID, Kfn, data_folder_vel.split('/')[-2] + warning);

    plt.savefig(data_folder+"vel_"+JOINT_NAME+".jpg")
    
#J Identification
if(IDENTIFICATION_MODE=='acc'):
    Kvp=(data_motor_param['Kvp'].item())
    (Kap, Kan, Kfp, Kfn) = identify_motor_acc(dt, dq, ddq, current, tau, Ktp, Kvp, 
                                              POSITIVE_VEL_THRESHOLD, ZERO_JERK_THRESHOLD, 
                                              SHOW_THRESHOLD_EFFECT);
    print 'Ka_p[%d] = %f' % (JOINT_ID,Kap);
    print 'Ka_n[%d] = %f' % (JOINT_ID,Kan);
    print 'Kf_p[%d] = %f' % (JOINT_ID,Kfp);
    print 'Kf_n[%d] = %f' % (JOINT_ID,Kfn);

#model vs measurement
if (IDENTIFICATION_MODE=='test_model'):
    #load motor parameters
    import dynamic_graph.sot.torque_control.hrp2.motors_parameters as hrp2_motors_parameters
    Kt_p = hrp2_motors_parameters.Kt_p
    Kt_n = hrp2_motors_parameters.Kt_n
    
    Kf_p = hrp2_motors_parameters.Kf_p
    Kf_n = hrp2_motors_parameters.Kf_n
    
    Kv_p = hrp2_motors_parameters.Kv_p
    Kv_n = hrp2_motors_parameters.Kv_n
    
    Ka_p = hrp2_motors_parameters.Ka_p
    Ka_n = hrp2_motors_parameters.Ka_n
    
    motor = Motor_model(Kt_p[JOINT_ID], Kt_n[JOINT_ID], 
                        Kf_p[JOINT_ID], Kf_n[JOINT_ID],
                        Kv_p[JOINT_ID], Kv_n[JOINT_ID],
                        Ka_p[JOINT_ID], Ka_n[JOINT_ID],dqThreshold=0.01)
    
    tau_motor=np.zeros(len(tau))
    tau_motor_current=np.zeros(len(tau))
    tau_motor_vel=np.zeros(len(tau))
    tau_motor_acc=np.zeros(len(tau))
    i_motor=np.zeros(len(current))
    for idx in range(len(tau)):
        tau_motor[idx]          =motor.getTorque    (current[idx], dq[idx], ddq[idx])
        tau_motor_current[idx]  =motor.getTorque    (current[idx], 0.0,     0.0)
        tau_motor_vel[idx]      =motor.getTorque    (0.0,          dq[idx], 0.0)
        tau_motor_acc[idx]      =motor.getTorque    (0.0,          0.0,     ddq[idx])
        i_motor[idx]            =motor.getCurrent   (tau[idx],     dq[idx], ddq[idx])

    plt.figure()
    alpha = 0.7
    plt.plot(tau,               alpha=alpha, label='torque from dynamic model')
    plt.plot(tau_motor,         alpha=alpha, label='torque from motor model')
    plt.plot(tau_motor_current, alpha=alpha, label='torque from motor model (cur only)')
    plt.plot(tau_motor_vel,     alpha=alpha, label='torque from motor model (vel only)')
    plt.plot(tau_motor_acc,     alpha=alpha, label='torque from motor model (acc only)')
    plt.legend()
    
    plt.figure()
    plt.subplot(211)
    plt.plot(dq, label='dq')
    plt.legend()
    plt.subplot(212)
    plt.plot(current)
    plt.plot(i_motor)
    plt.legend(['measured current','Estimated current with model'])    
    
    plt.figure()
    plt.plot(current, label='current')
    plt.plot(ctrl/IN_OUT_GAIN, label='ctrl')
    plt.legend()
    plt.show()
