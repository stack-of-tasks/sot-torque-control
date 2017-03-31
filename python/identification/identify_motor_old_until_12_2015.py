# -*- coding: utf-8 -*-
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""

import numpy as np
import sys

import matplotlib as mpl
from plot_utils import plot3d
from plot_utils import plot_x_vs_y
from plot_utils import saveCurrentFigure
import plot_utils
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.legend_handler import HandlerLine2D

import hrp2_motors_parameters_pwl as pwl

from qpoases import PyQProblemB as QProblemB # QP with simple bounds only
from qpoases import PySQProblem as SQProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from qpoases import PyReturnValue


def update_line(num, data, force, line, line_head, line_torque, line_force):
    ns = 200;
    # make sure that time is as long as data by selecting only the first ns*num elements
    time = np.arange(0,ns*num*DT,DT)[:ns*num];
    line_torque.set_data(time, data[0,:(ns*num)]);
    line_force.set_data(time, force[:(ns*num)]);
    if(num<=1):
        line.set_data(data[...,:(ns*num)]);
    else:
        line.set_data(data[...,:(ns*(num-1))]);
        line_head.set_data(data[...,(ns*(num-1)):(ns*num)]);
    return (line, line_head, line_torque, line_force);
    
''' Solve the least square problem:
    minimize   || A*x-b ||^2
    subject to lb_in <= A_in*x <= ub_in
                lb <= x <= ub
'''
def solveLeastSquare(A, b, lb=None, ub=None, A_in=None, lb_in=None, ub_in=None):
    n                   = A.shape[1];
    m_in                = 0;
    if(A_in!=None):
        m_in = A_in.shape[0];
        if(lb_in==None):
            lb_in           = np.array(m_in*[-1e99]);
        if(ub_in==None):
            ub_in           = np.array(m_in*[1e99]);
            
    if(lb==None):
        lb                  = np.array(n*[-1e99]);
    if(ub==None):
        ub                  = np.array(n*[1e99]);
        
    Hess                = np.dot(A.transpose(),A);
    grad                = -np.dot(A.transpose(),b);
    maxActiveSetIter    = np.array([100+2*m_in+2*n]);
    maxComputationTime  = np.array([600.0]);
    options             = Options();
    options.printLevel  = PrintLevel.LOW; #NONE, LOW, MEDIUM
    options.enableRegularisation = True;
    print 'Gonna solve QP...';
    if(m_in==0):
        qpOasesSolver       = QProblemB(n); #, HessianType.SEMIDEF);
        qpOasesSolver.setOptions(options);
        imode = qpOasesSolver.init(Hess, grad, lb, ub, maxActiveSetIter, maxComputationTime);
    else:
        qpOasesSolver       = SQProblem(n, m_in); #, HessianType.SEMIDEF);
        qpOasesSolver.setOptions(options);
        imode = qpOasesSolver.init(Hess, grad, A_in, lb, ub, lb_in, ub_in, maxActiveSetIter, maxComputationTime);
    print 'QP solved in %f seconds and %d iterations' % (maxComputationTime[0],maxActiveSetIter[0]);
    if(imode!=0 and imode!=63):
        print "ERROR Qp oases %d " % (imode);    
    x_norm  = np.zeros(n);  # solution of the normalized problem
    qpOasesSolver.getPrimalSolution(x_norm);
    return x_norm;

FOLDER_ID = 23;
data_folder_2 = '../results/20150423_102257_rhp_torque_id/';
DATA_FILE_NAME_2 = 'data_j2.npz';
DELTA_Q_MAX = 0.02;
ZERO_VEL_THR = 0.02;
VEL_SIGN_THRESHOLD = 0.0;
DT = 0.001;
DATA_FILE_NAME = 'data.npz';

if(FOLDER_ID==1):
    data_folder = '../results/20150223_id_rhp/';
    JOINT_ID = 2;
    DELTA_Q_MAX = 0.015;
    ZERO_VEL_THR = 0.04;
elif(FOLDER_ID==2):
    data_folder = '../results/20150223_191245_rhr/';
    JOINT_ID = 1;
    DELTA_Q_MAX = 0.015;
elif(FOLDER_ID==3):
    data_folder = '../results/20150223_193218_rhy/';
    JOINT_ID = 0;
    DELTA_Q_MAX = 0.025;
elif(FOLDER_ID==4):
    data_folder = '../results/20150223_194057_rk/';
    JOINT_ID = 3;
    DELTA_Q_MAX = 0.015;
elif(FOLDER_ID==5):
    data_folder = '../results/20150223_194647_rap/';
    JOINT_ID = 4;
    DELTA_Q_MAX = 0.025;
elif(FOLDER_ID==6):
    data_folder = '../results/20150223_195514_rar/';
    JOINT_ID = 5;
    DELTA_Q_MAX = 0.015;
elif(FOLDER_ID==7):
    data_folder = '../results/20150226_182400_lhp/';
    JOINT_ID = 8;
elif(FOLDER_ID==9): # hip pitch chirp
    data_folder = '../results/20150317_175631_rhp_chirp/';
    JOINT_ID = 2; 
elif(FOLDER_ID==10): # hip pitch chirp with higher delay and no F/T sensor in velocity part
    data_folder = '../results/20150322_184650_rhp/';
    JOINT_ID = 2; 
    ZERO_VEL_THR = 0.22;
elif(FOLDER_ID==11): # hip pitch chirp with higher delay and no F/T sensor in velocity part
    data_folder = '../results/20150322_190337_rhp/';
    JOINT_ID = 2; 
    ZERO_VEL_THR = 0.22;
elif(FOLDER_ID==12): # hip pitch chirp with higher delay and no F/T sensor in velocity part
    data_folder = '../results/20150322_200613_rhp/';
    JOINT_ID = 2; 
    ZERO_VEL_THR = 0.22;
    VEL_SIGN_THRESHOLD = 0.02;
elif(FOLDER_ID==13): # old data taken at 5 ms pushing the robot on both feet
    data_folder = '../results/20140807-legTorqueId1/';
    JOINT_ID = 2; 
elif(FOLDER_ID==14): # new data taken dumping only sensor data and computing torques afterwards
    data_folder = '../results/20150422_182432_rhp_torque_id/';
    JOINT_ID = 2; 
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';
elif(FOLDER_ID==15): # new data taken dumping only sensor data and computing torques afterwards
    data_folder = '../results/20150423_102257_rhp_torque_id/';
    JOINT_ID = 2; 
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';
elif(FOLDER_ID==16): # data collected with torque ctrl (robot pushing)
    data_folder = '../results/20150423_173259_rhp_torque_id_contact/';
    JOINT_ID = 2;
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';
    ZERO_VEL_THR = 0.1;
elif(FOLDER_ID==17): # data collected with torque ctrl first (robot pushing) and pos ctrl then (me pushing)
    data_folder = '../results/20150424_100759_rhp_torque_id_2ways/';
    JOINT_ID = 2;
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';
    ZERO_VEL_THR = 0.1;
elif(FOLDER_ID==18): # data collected with pos ctrl (me pushing) changing the configuration of the leg during the experiment
    data_folder = '../results/20150424_112301_rhp_torque_id_multi_conf/';
    JOINT_ID = 2;
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';    
elif(FOLDER_ID==19): # data collected with pos ctrl (me pushing) changing the normal force on the right foot during the experiment
    data_folder = '../results/20150424_135254_rhp_torque_id_multi_norm_force/';
    JOINT_ID = 2;
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';
    ZERO_VEL_THR = 0.04;
elif(FOLDER_ID==20): # data collected with pos ctrl (me pushing) 
    data_folder = '../results/20150430_095211_rhp_id_zero_normal_force/';
    JOINT_ID = 2;
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';
    ZERO_VEL_THR = 0.04;
elif(FOLDER_ID==21): # data collected with pos ctrl (me pushing)
    data_folder = '../results/20150430_103057_rhp_id_100_normal_force/';
    JOINT_ID = 2;
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';
    ZERO_VEL_THR = 0.04;
elif(FOLDER_ID==22): # data collected with pos ctrl (me pushing) using clamp to change force at F/T sensor
    data_folder = '../results/20150505_113323_rhp_id_clamp/';
    JOINT_ID = 3;
    DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';
else:
    print 'ERROR: UNKNOWN FOLDER_ID';

USE_LOW_VELOCITY_DATA_ONLY = False;
DO_IDENTIFICATION_WITH_FIRST_ORDER_DYNAMICS = False;
MAX_N_SAMPLES = 300;  ''' max number of samples handled by the QP solver '''

w = 100;    # weight used for positive penalty

PLOT_LOW_VEL_DATA_3D    = False;
PLOT_LOW_VEL_DATA       = False;
PLOT_RESIDUALS          = False;
PLOT_3D_SURFACE         = False;
PLOT_FRICTION_DATA      = False;
PLOT_PIECE_WISE_LINEAR  = False;
PLOT_ANIMATED_DATA      = True;
COMPARE_TWO_DATASETS    = False;
SHOW_PLOTS              = True;

plot_utils.FIGURE_PATH      = data_folder;
plot_utils.SAVE_FIGURES     = False;
plot_utils.SHOW_FIGURES     = True;
plot_utils.SHOW_LEGENDS     = True;
plot_utils.LINE_ALPHA       = 0.7;


''' Load data from file '''
try:
    data = np.load(data_folder+DATA_FILE_NAME);
    if(len(data['enc'].shape)==1):
        enc = np.squeeze(data['enc']);
        dq = np.squeeze(data['dq']);
        tau = np.squeeze(data['tau']);
        qDes = np.squeeze(data['qDes']);
    else:
        enc = np.squeeze(data['enc'][:,JOINT_ID]);
        dq = np.squeeze(data['dq'][:,JOINT_ID]);
        tau = np.squeeze(data['tau'][:,JOINT_ID]);
        qDes = np.squeeze(data['qDes'][:,JOINT_ID]);
#    if(FOLDER_ID==13):
#        tau = np.loadtxt(data_folder+'torque.dat');
#        tau = tau[-enc.shape[0]:];
#        np.savez_compressed(data_folder+'data_new.npz', enc=enc,dq=dq,tau=tau,qDes=qDes);
except IOError:
    print "Impossible to read data file %f" % (data_folder+DATA_FILE_NAME);
    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");
    
#''' Load sensors data from file '''
#try:
#    sensor_data = np.load(data_folder+'data.npz');
#    acc = sensor_data['acc'];
#    gyro = sensor_data['gyro'];
#    forceLA = sensor_data['forceLA'];
#    forceRA = sensor_data['forceRA'];
#    forceLL = sensor_data['forceLL'];
#    forceRL = sensor_data['forceRL'];
#except IOError:
#    print "Impossible to read data file %f" % (data_folder+'data.npz');
#    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");
    
if(COMPARE_TWO_DATASETS):
    ''' Load data from file '''
    try:
        data_2 = np.load(data_folder_2+DATA_FILE_NAME_2);
        if(len(data_2['enc'].shape)==1):
            enc_2  = np.squeeze(data_2['enc']);
            dq_2   = np.squeeze(data_2['dq']);
            tau_2  = np.squeeze(data_2['tau']);
            qDes_2 = np.squeeze(data_2['qDes']);
        else:
            enc_2  = np.squeeze(data_2['enc'][:,JOINT_ID]);
            dq_2   = np.squeeze(data_2['dq'][:,JOINT_ID]);
            tau_2  = np.squeeze(data_2['tau'][:,JOINT_ID]);
            qDes_2 = np.squeeze(data_2['qDes'][:,JOINT_ID]);
        delta_q_2 = qDes_2-enc_2;
    except IOError:
        print "Impossible to read data file %f" % (data_folder_2+DATA_FILE_NAME);
        sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");

if(USE_LOW_VELOCITY_DATA_ONLY==True):
    i_zero_vel = np.where(np.abs(dq)<ZERO_VEL_THR);
#    q = q[i_zero_vel];
    enc = enc[i_zero_vel];
    dq = dq[i_zero_vel];
    tau = tau[i_zero_vel];
    qDes = qDes[i_zero_vel];

m = len(dq);
delta_q = qDes-enc;

print '   Perform identification with symmetric penalty function ';
dq_max = np.max(dq);
tau_max = np.max(tau);
delta_q_max = np.max(delta_q);
A_norm  = np.zeros((m,2));
A_norm[:,0]  = dq/dq_max;
A_norm[:,1]  = tau/tau_max;
b_norm       = delta_q/delta_q_max;
x_norm = solveLeastSquare(A_norm, b_norm);
k_v_ls   = x_norm[0]*delta_q_max/dq_max;
k_tau_ls = x_norm[1]*delta_q_max/tau_max;
x_ls       = np.array([k_v_ls, k_tau_ls, 0, 0,0,0]);  # solution of the real problem
print 'k_v[%d] = %f\nk_tau[%d] = %f' % (JOINT_ID, k_v_ls, JOINT_ID, k_tau_ls);

if(DO_IDENTIFICATION_WITH_FIRST_ORDER_DYNAMICS):
    print '   Perform identification with 1-st order dynamics';
    A_norm  = np.zeros((m-1,3));
    A_norm[:,0]  = dq[:-1]/dq_max;
    A_norm[:,1]  = tau[:-1]/tau_max;
    A_norm[:,2]  = tau[1:]/tau_max;
    b_norm       = delta_q[:-1]/delta_q_max;
    x_norm = solveLeastSquare(A_norm, b_norm);
    k_v_dyn   = x_norm[0]*delta_q_max/dq_max;
    k_tau1_dyn = x_norm[1]*delta_q_max/tau_max;
    k_tau2_dyn = x_norm[2]*delta_q_max/tau_max;
    print 'k_v[%d] = %f\nk_tau[%d] = %f\tk_tau[%d] = %f' % (JOINT_ID, k_v_dyn, JOINT_ID, k_tau1_dyn, JOINT_ID, k_tau2_dyn);

print '   Perform identification with symmetric penalty function and Couloumb friction';
i_zero_vel = np.where(np.abs(dq)<ZERO_VEL_THR);
tau_max = np.max(tau[i_zero_vel]);
delta_q_max = np.max(delta_q[i_zero_vel]);
A_norm  = np.zeros((len(i_zero_vel[0]),3));
A_norm[:,0]  = tau[i_zero_vel]/tau_max;
A_norm[np.where(tau[i_zero_vel]>0),1]  = 1;
A_norm[np.where(tau[i_zero_vel]<0),2]  = -1;
b_norm       = delta_q[i_zero_vel]/delta_q_max;
x_norm = solveLeastSquare(A_norm, b_norm);
k_tau_c1 = x_norm[0]*delta_q_max/tau_max;
k_tp_c1   = x_norm[1]*delta_q_max;
k_tn_c1   = x_norm[2]*delta_q_max;

A_tau  = np.zeros((m,3));
A_tau[:,0]  = tau/tau_max;
A_tau[np.where(tau>0),1]  = 1;
A_tau[np.where(tau<0),2]  = -1;
b_norm  = delta_q/delta_q_max;
b_norm  = b_norm - np.dot(A_tau,x_norm);
A_norm  = np.zeros((m,3));
A_norm[:,0]  = dq/dq_max;
A_norm[np.where(dq>ZERO_VEL_THR), 1]  = 1;
A_norm[np.where(dq<-ZERO_VEL_THR),2]  = -1;
x_norm = solveLeastSquare(A_norm, b_norm);
k_v_c1    = x_norm[0]*delta_q_max/dq_max;
k_cp_c1   = x_norm[1]*delta_q_max;
k_cn_c1   = x_norm[2]*delta_q_max;
x_c1       = np.array([k_v_c1, k_tau_c1, k_cp_c1, k_cn_c1, k_tp_c1, k_tn_c1]);  # solution of the real problem
print 'k_v[%d] = %f\nk_tau[%d] = %f\nk_cp[%d] = %f\nk_cn[%d] = %f' % (JOINT_ID, k_v_c1, JOINT_ID, k_tau_c1, JOINT_ID, k_cp_c1, JOINT_ID, k_cn_c1);
print 'k_tp[%d] = %f\nk_tn[%d] = %f' % (JOINT_ID, k_tp_c1, JOINT_ID, k_tn_c1);


print '    Perform identification with asymmetric penalty function ';
''' If necessary, downsample data '''
if(m>MAX_N_SAMPLES):
    ds   = int(m/MAX_N_SAMPLES)+1;
#    q    = q[0:-1:ds];
    enc  = enc[0:-1:ds];
    dq   = dq[0:-1:ds];
    tau  = tau[0:-1:ds];
    qDes = qDes[0:-1:ds];
    delta_q = qDes-enc; #qDes-q;
    print 'Downsample data from %d to %d' % (m, len(dq));
    m = len(dq);
    
n = 2;
A       = np.zeros((m,2*m+n));
b       = np.zeros(m);
A[:,n:n+m]  = w*np.identity(m);
A[:,n+m:] = np.identity(m);
lb = np.zeros(2*m+n);
lb[0:n] = -1e99;

i_pos = np.where(delta_q>0);
m_pos = len(i_pos[0]);
i_neg = np.where(delta_q<=0);
m_neg = len(i_neg[0]);
A_bar = np.zeros((m,n));
b_bar = np.zeros(m);
dq_max = np.max(dq);
tau_max = np.max(tau);
delta_q_max = np.max(delta_q);
A_norm  = np.zeros((m,2));
A_norm[:,0]  = dq/dq_max;
A_norm[:,1]  = tau/tau_max;
b_norm       = delta_q/delta_q_max;
A_bar[0:m_pos,:]    = A_norm[i_pos,0:2]; 
A_bar[m_pos:m,:]    = -A_norm[i_neg,0:2];
b_bar[0:m_pos]      = b_norm[i_pos]; 
b_bar[m_pos:m]      = -b_norm[i_neg];
A_in = np.zeros((2*m,2*m+n));
ub_in = np.zeros(2*m);
A_in[0:m,0:n]       = A_bar;
A_in[0:m,n:n+m]     = -np.identity(m);
A_in[m:,0:n]        = -A_bar;
A_in[m:,n+m:]       = -np.identity(m);
ub_in[0:m]          = b_bar;
ub_in[m:]           = -b_bar;
x_norm = solveLeastSquare(A, b, lb, ub=None, A_in=A_in, lb_in=None, ub_in=ub_in);
#x_norm = solveLeastSquare(A_bar, b_bar);
k_v   = x_norm[0]*delta_q_max/dq_max;
k_tau = x_norm[1]*delta_q_max/tau_max;
x     = np.array([k_v, k_tau, 0, 0,0,0]);  # solution of the real problem
print 'k_v[%d] = %f\nk_tau[%d] = %f' % (JOINT_ID, k_v, JOINT_ID, k_tau);




if(len(data['enc'].shape)==1):
    enc = np.squeeze(data['enc']);
    dq = np.squeeze(data['dq']);
    tau = np.squeeze(data['tau']);
    qDes = np.squeeze(data['qDes']);
else:
    enc = np.squeeze(data['enc'][:,JOINT_ID]);
    dq = np.squeeze(data['dq'][:,JOINT_ID]);
    tau = np.squeeze(data['tau'][:,JOINT_ID]);
    qDes = np.squeeze(data['qDes'][:,JOINT_ID]);
delta_q = qDes-enc;
m = len(dq);

''' Plot only low-velocity data '''
i_zero_vel = np.where(np.abs(dq)<ZERO_VEL_THR);
print 'Zero-velocity samples are %d out of %d'%(len(i_zero_vel[0]),len(dq));
dt = 0.001;
time = np.arange(0, dt*len(dq), dt);

plt.figure(); plt.plot(time, dq); plt.plot([time[0], time[-1]], [ZERO_VEL_THR, ZERO_VEL_THR]);
plt.plot([time[0], time[-1]], [-ZERO_VEL_THR, -ZERO_VEL_THR]);
leg=plt.legend(['dq', 'pos zero-velocity threshold', 'neg zero-velocity threshold']);
leg.get_frame().set_alpha(0.5);

time_zero_vel       = time[i_zero_vel];
dq_zero_vel         = dq[i_zero_vel];
tau_zero_vel        = tau[i_zero_vel];
delta_q_zero_vel    = delta_q[i_zero_vel];

if(PLOT_ANIMATED_DATA):
    fig1, ax = plt.subplots(2,2,sharex=False);
    d = np.empty((2, len(i_zero_vel[0])));
    d[0,:] = tau_zero_vel;
    d[1,:] = delta_q_zero_vel;
    l, = ax[0,0].plot([], [], 'b x', alpha=0.3);
    l_head, = ax[0,0].plot([], [], 'r x', alpha=0.5);
    ax[0,0].set_xlim(np.min(tau), tau_max);
    ax[0,0].set_ylim(np.min(delta_q), delta_q_max);
    ax[0,0].set_xlabel('tau');
    ax[0,0].set_ylabel('delta_q');    
    l_torque, = ax[1,0].plot([], [], 'b-');
    ax[1,0].set_xlim(0, len(i_zero_vel[0])*DT);
    ax[1,0].set_ylim(np.min(tau), tau_max);
    ax[1,0].set_xlabel('Time');
    ax[1,0].set_ylabel('Tau');
    l_force, = ax[1,1].plot([], [], 'b-');
    ax[1,1].set_xlim(0, len(i_zero_vel[0])*DT);
    ax[1,1].set_ylim(np.min(forceRL[:,2]), np.max(forceRL[:,2]));
    ax[1,1].set_xlabel('Time');
    ax[1,1].set_ylabel('Normal force right foot');
    anim_iter = int(len(i_zero_vel[0])/200);
    line_ani = animation.FuncAnimation(fig1, update_line, anim_iter, fargs=(d, forceRL[:,2], l, l_head, l_torque, l_force), 
                                       interval=50, blit=True, repeat=False);
#    plt.show();

''' Display residuals '''
if(PLOT_RESIDUALS):
    A       = np.zeros((m,6));
    A[:,0]  = dq;
    A[:,1]  = tau;
    A[np.where(dq>ZERO_VEL_THR), 2]  = 1;
    A[np.where(dq<-ZERO_VEL_THR),3]  = -1;
    A[np.where(tau>0),4]  = 1;
    A[np.where(tau<0),5]  = -1;
    
#    mpl.rcParams['axes.labelsize']      = 45;
#    mpl.rcParams['font.size']           = 45;
    f, ax = plt.subplots(1,1,sharex=True);
    res = np.abs(np.dot(A,x_ls)) - np.abs(delta_q);
    binwidth = (max(res) - min(res))/20.0;
    ax.hist(res, bins=np.arange(min(res), max(res) + binwidth, binwidth));
    ax.set_xlabel('Residual [rad]');
    ax.set_ylabel('Number of samples');
    ax.yaxis.set_ticks([0, 5e3, 10e3, 15e3]);
    ax.xaxis.set_ticks([-0.004, -0.002, 0, 0.002]);
    ax.set_xlim([-0.004, 0.002]);
    saveCurrentFigure('friction_residuals_histogram_symmetric');
    
    f, ax = plt.subplots(1,1,sharex=True);
    res = np.abs(np.dot(A,x)) - np.abs(delta_q);
    binwidth = (max(res) - min(res))/20.0;
    ax.hist(res, bins=np.arange(min(res), max(res) + binwidth, binwidth));
    ax.set_xlabel('Residual [rad]');
    ax.set_ylabel('Number of samples');
    ax.yaxis.set_ticks([0, 5000, 10000, 15000]);
    ax.xaxis.set_ticks([-0.004, -0.002, 0, 0.002]);
    ax.set_xlim([-0.004, 0.002]);
    saveCurrentFigure('friction_residuals_histogram_asymmetric');
    
#    res = np.abs(np.dot(A,x_c1)) - np.abs(delta_q);
#    binwidth = (max(res) - min(res))/20.0;
#    ax[2].hist(res, bins=np.arange(min(res), max(res) + binwidth, binwidth));
#    ax[2].set_title('Residuals with symmetric penalty and Coloumb friction term');

if(PLOT_3D_SURFACE):
    ''' Plot 3d surface '''
    x_grid = np.linspace(min(dq), max(dq), 15);
    y_grid = np.linspace(min(tau), max(tau), 15);
    X, Y = np.meshgrid(x_grid, y_grid);
    Xp = np.zeros(X.shape);
    Xn = np.zeros(X.shape);
    Yp = np.zeros(X.shape);
    Yn = np.zeros(X.shape);
    Xp[np.where(X>ZERO_VEL_THR)] = 1;
    Xn[np.where(X<-ZERO_VEL_THR)] = -1;
    Yp[np.where(Y>0)] = 1;
    Yn[np.where(Y<0)] = -1;
    Z = k_v*X + k_tau*Y;
    fig = plt.figure();
    ax = fig.add_subplot(221, projection='3d');
    plot3d(dq, tau, delta_q, 'Asymmetric ID - '+data_folder, ['dq','tau','delta q'],'r ', ax=ax);
    ax.plot_wireframe(X,Y,Z);
    ax = fig.add_subplot(222, projection='3d');
    Z = k_v_ls*X + k_tau_ls*Y;
    plot3d(dq, tau, delta_q, 'Symmetric ID - '+data_folder, ['dq','tau','delta q'],'r ', ax=ax);
    ax.plot_wireframe(X,Y,Z);
    ax = fig.add_subplot(223, projection='3d');
    Z = k_v_c1*X + k_tau_c1*Y + k_cp_c1*Xp + k_cn_c1*Xn + k_tp_c1*Yp + k_tn_c1*Yn;
    plot3d(dq, tau, delta_q, 'Symmetric ID with Coloumb friction- '+data_folder, ['dq','tau','delta q'],'r ', ax=ax);
    ax.plot_wireframe(X,Y,Z);


if(PLOT_LOW_VEL_DATA_3D==True):
    x_grid = np.linspace(min(dq_zero_vel), max(dq_zero_vel), 15);
    y_grid = np.linspace(min(tau_zero_vel), max(tau_zero_vel), 15);
    X, Y = np.meshgrid(x_grid, y_grid);
    Yp = np.zeros(X.shape);
    Yn = np.zeros(X.shape);
    Yp[np.where(Y>0)] = 1;
    Yn[np.where(Y<0)] = -1;
    fig = plt.figure();
    ax = fig.add_subplot(221, projection='3d');
    Z = k_v*X + k_tau*Y;
    plot3d(dq_zero_vel, tau_zero_vel, delta_q_zero_vel, 'Low velocity data asymmetric penalty '+data_folder, ['dq','tau','delta q'],'r ', ax=ax);
    ax.plot_wireframe(X,Y,Z);
    ax = fig.add_subplot(222, projection='3d');
    Z = k_v_ls*X + k_tau_ls*Y;
    plot3d(dq_zero_vel, tau_zero_vel, delta_q_zero_vel, 'Low velocity data symmetric penalty '+data_folder, ['dq','tau','delta q'],'r ', ax=ax);
    ax.plot_wireframe(X,Y,Z);
    ax = fig.add_subplot(223, projection='3d');
    Z = k_v_c1*X + k_tau_c1*Y + k_tp_c1*Yp + k_tn_c1*Yn;
    plot3d(dq_zero_vel, tau_zero_vel, delta_q_zero_vel, 'Low velocity data Coloumb '+data_folder, ['dq','tau','delta q'],'r ', ax=ax);
    ax.plot_wireframe(X,Y,Z);

if(PLOT_LOW_VEL_DATA==True):
    tau_min = np.min(tau_zero_vel);
    tau_max = np.max(tau_zero_vel);
    tau_model = np.array([tau_min, -1e-3, 1e-3, tau_max]);
    tau_pos_model = np.zeros(tau_model.shape);
    tau_neg_model = np.zeros(tau_model.shape);
    tau_pos_model[np.where(tau_model>0)] = 1;
    tau_neg_model[np.where(tau_model<0)] = -1;
    
    f, ax = plt.subplots(1,1,sharex=True);
    delta_q_model = k_tau*tau_model;
    data_line, = ax.plot(tau_zero_vel, delta_q_zero_vel-k_v*dq_zero_vel,'r .',rasterized=True);
    model_line, = ax.plot(tau_model, delta_q_model,'b',rasterized=True);
    ax.set_xlabel(r'$\tau$ [Nm]');
    ax.set_ylabel(r'$\Delta_q$ [rad]');
    leg = ax.legend([data_line, model_line],['Data','Model'],loc='lower right',numpoints=1, markerscale=5);
    saveCurrentFigure('Low velocity data  asymmetric penalty');
    ax.set_title('Low velocity data asymmetric penalty');
    
    f, ax = plt.subplots(1,1,sharex=True);
    delta_q_model = k_tau_ls*tau_model;
    data_line, = ax.plot(tau_zero_vel, delta_q_zero_vel-k_v_ls*dq_zero_vel,'r .',rasterized=True);
    model_line, = ax.plot(tau_model, delta_q_model,'b',rasterized=True);
    ax.set_xlabel(r'$\tau$ [Nm]');
    ax.set_ylabel(r'$\Delta_q$ [rad]');
    leg = ax.legend(['Data','Model'],loc='lower right',numpoints=1, markerscale=5);
    saveCurrentFigure('Low velocity data  symmetric penalty');
    ax.set_title('Low velocity data symmetric penalty');
    
    f, ax = plt.subplots(1,1,sharex=True);
    delta_q_model = k_tau_c1*tau_model + k_tp_c1*tau_pos_model + k_tn_c1*tau_neg_model;
    ax.plot(tau_zero_vel, delta_q_zero_vel-k_v_c1*dq_zero_vel,'r .',rasterized=True);
    ax.plot(tau_model, delta_q_model,'b',rasterized=True);
    ax.set_xlabel(r'$\tau$ [Nm]');
    ax.set_ylabel(r'$\Delta_q$ [rad]');
    leg = ax.legend(['Data','Model'],loc='lower right',numpoints=1, markerscale=5);
    saveCurrentFigure('Low velocity data  coulomb');
    ax.set_title('Low velocity data Coulomb');

    i_dq_pos = np.where(dq_zero_vel>VEL_SIGN_THRESHOLD);
    i_dq_unc = np.where(np.abs(dq_zero_vel)<VEL_SIGN_THRESHOLD);
    i_dq_neg = np.where(dq_zero_vel<-VEL_SIGN_THRESHOLD);
    f, ax = plt.subplots(1,1,sharex=True);
    pos_line, = ax.plot(tau_zero_vel[i_dq_pos], delta_q_zero_vel[i_dq_pos]-k_v*dq_zero_vel[i_dq_pos],'r .',rasterized=True);
    neg_line, = ax.plot(tau_zero_vel[i_dq_neg], delta_q_zero_vel[i_dq_neg]-k_v*dq_zero_vel[i_dq_neg],'b .',rasterized=True);
    if(len(i_dq_unc[0])>0):
        ax.plot(tau_zero_vel[i_dq_unc], delta_q_zero_vel[i_dq_unc]-k_v*dq_zero_vel[i_dq_unc],'g .',rasterized=True);
        leg = ax.legend(['Positive velocity','Negative velocity','Uncertain velocity'],numpoints=1,markerscale=5);
    else:
        leg = ax.legend([pos_line, neg_line],['Positive velocity','Negative velocity'],loc='lower right',numpoints=1, markerscale=5);
    leg.get_frame().set_alpha(plot_utils.LEGEND_ALPHA);
    ax.set_xlabel(r'$\tau$ [Nm]');
    ax.set_ylabel(r'$\Delta_q$ [rad]');
    saveCurrentFigure('Low velocity data by velocity sign');
    ax.set_title('Low velocity data separated by velocity sign');    
    
    f, ax = plt.subplots(1,1,sharex=True);    
    ax.plot(time_zero_vel[i_dq_pos], tau_zero_vel[i_dq_pos],'r .',rasterized=True);
    ax.plot(time_zero_vel[i_dq_neg], tau_zero_vel[i_dq_neg],'b .',rasterized=True);
    ax.plot(time_zero_vel[i_dq_unc], tau_zero_vel[i_dq_unc],'g .',rasterized=True);
    leg = ax.legend(['Positive','Negative','Uncertain'],loc='upper left',numpoints=1,markerscale=5);
    leg.get_frame().set_alpha(plot_utils.LEGEND_ALPHA);
    saveCurrentFigure('Torque vs time separated by velocity sign');
    ax.set_title('Torque vs time separated by velocity sign');
    
if(PLOT_PIECE_WISE_LINEAR):    
    tau_min = np.min([np.min(tau_zero_vel), pwl.f_tau1p[JOINT_ID], pwl.f_tau1n[JOINT_ID]]);
    tau_max = np.max([np.max(tau_zero_vel), pwl.f_tau2p[JOINT_ID], pwl.f_tau2n[JOINT_ID]]);
    tau_model_pos = np.array([tau_min, pwl.f_tau1p[JOINT_ID], pwl.f_tau2p[JOINT_ID], tau_max]);
    tau_model_neg = np.array([tau_min, pwl.f_tau1n[JOINT_ID], pwl.f_tau2n[JOINT_ID], tau_max]);
    delta_q_model_pos = np.array([pwl.f_k1p[JOINT_ID]*tau_min              +pwl.f_q1p[JOINT_ID], 
                                  pwl.f_k1p[JOINT_ID]*pwl.f_tau1p[JOINT_ID]+pwl.f_q1p[JOINT_ID],
                                  pwl.f_k2p[JOINT_ID]*pwl.f_tau2p[JOINT_ID]+pwl.f_q2p[JOINT_ID],
                                  pwl.f_k3p[JOINT_ID]*tau_max              +pwl.f_q3p[JOINT_ID]]);
    delta_q_model_neg = np.array([pwl.f_k1n[JOINT_ID]*tau_min              +pwl.f_q1n[JOINT_ID], 
                                  pwl.f_k1n[JOINT_ID]*pwl.f_tau1n[JOINT_ID]+pwl.f_q1n[JOINT_ID],
                                  pwl.f_k2n[JOINT_ID]*pwl.f_tau2n[JOINT_ID]+pwl.f_q2n[JOINT_ID],
                                  pwl.f_k3n[JOINT_ID]*tau_max              +pwl.f_q3n[JOINT_ID]]);
    
    f, ax = plt.subplots(1,1,sharex=True);
#    ax.plot(tau_zero_vel, delta_q_zero_vel-k_v*dq_zero_vel,'r .');
    ax.plot(tau_zero_vel, delta_q_zero_vel,'r .', alpha=1,rasterized=True);
    ax.plot(tau_model_pos, delta_q_model_pos,'b--',rasterized=True);
    ax.plot(tau_model_neg, delta_q_model_neg,'b--',rasterized=True);
    ax.set_xlabel(r'$\tau$ [Nm]');
    ax.set_ylabel(r'$\Delta_q$ [rad]');
    leg = ax.legend(['Data','Model'],loc='lower right',numpoints=1,markerscale=5);
    title = 'Torque vs delta_q  piece-wise linear model';
    saveCurrentFigure(title);
    ax.set_title(title);
    
    if(COMPARE_TWO_DATASETS):
        ax.plot(tau_2, delta_q_2,'c .', alpha=0.1);
        ax.legend(['dataset 1','','','dataset 2'], loc='upper left');
    

if(PLOT_FRICTION_DATA):
    ''' Plot 2d relationship between velocity and delta_q '''
    dq_min = np.min(dq);
    dq_max = np.max(dq);
    dq_model = np.array([dq_min, -1e-3, 1e-3, dq_max]);
    dq_pos_model = np.zeros(dq_model.shape);
    dq_neg_model = np.zeros(dq_model.shape);
    dq_pos_model[np.where(dq_model>0)] = 1;
    dq_neg_model[np.where(dq_model<0)] = -1;
    
    f, ax = plt.subplots(1,1,sharex=True);
    delta_q_model = k_v*dq_model;
    ax.plot(dq, delta_q-k_tau*tau,'r .',rasterized=True);
    ax.plot(dq_model, delta_q_model,'b--',rasterized=True);
    ax.set_xlabel(r'$\dot{q}_j$ [rad/s]');
    ax.set_ylabel(r'$\Delta_q - K_{\tau}\tau$ [rad]');
    leg = plt.legend(['Data', 'Model'], loc='upper left',numpoints=1,markerscale=5);
    leg.get_frame().set_alpha(plot_utils.LEGEND_ALPHA);
    saveCurrentFigure('dq vs delta_q  asymmetric penalty');
    
    f, ax = plt.subplots(1,1,sharex=True);
    delta_q_model = k_v_ls*dq_model;
    ax.plot(dq, delta_q-k_tau_ls*tau,'r .',rasterized=True);
    ax.plot(dq_model, delta_q_model,'b--',rasterized=True);
    ax.set_xlabel(r'$\dot{q}_j$ [rad/s]');
    ax.set_ylabel(r'$\Delta_q - K_{\tau}\tau$ [rad]');
    leg = plt.legend(['Data', 'Model'], loc='upper left',numpoints=1,markerscale=5);
    leg.get_frame().set_alpha(plot_utils.LEGEND_ALPHA);
    saveCurrentFigure('dq vs delta_q  symmetric penalty');
    
    f, ax = plt.subplots(1,1,sharex=True);
    delta_q_model = k_v_c1*dq_model + k_cp_c1*dq_pos_model + k_cn_c1*dq_neg_model;
    ax.plot(dq, delta_q-k_tau_c1*tau,'r .',rasterized=True);
    ax.plot(dq_model, delta_q_model,'b--',rasterized=True);
    ax.set_xlabel(r'$\dot{q}_j$ [rad/s]');
    ax.set_ylabel(r'$\Delta_q - K_{\tau}\tau$ [rad]');
    leg = plt.legend(['Data', 'Model'], loc='upper left',numpoints=1,markerscale=5);
    leg.get_frame().set_alpha(plot_utils.LEGEND_ALPHA);
    saveCurrentFigure('dq vs delta_q  coulomb');
    
if(SHOW_PLOTS):
    plt.show();