# -*- coding: utf-8 -*-
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""

import numpy as np
import matplotlib.pyplot as plt
import plot_utils as plut
from compute_estimates_from_sensors import compute_estimates_from_sensors
from compute_estimates_from_sensors import compute_delta_q_components


def plotForceTracking(time, f, fRef):
    max_f = np.max(np.array([fRef[:,:3],f[:,:3]]));
    min_f = np.min(np.array([fRef[:,:3],f[:,:3]]));
    max_f += 0.1*(max_f-min_f);
    min_f -= 0.1*(max_f-min_f);
    max_m = np.max(np.array([fRef[:,3:],f[:,3:]]));
    min_m = np.min(np.array([fRef[:,3:],f[:,3:]]));
    max_m += 0.1*(max_m-min_m);
    min_m -= 0.1*(max_m-min_m);
    for i in range(6):
        print 'Max force tracking error for axis %d:         %.2f' % (i, np.max(np.abs(f[:,i]-fRef[:,i])));
        print 'Mean square force tracking error for axis %d: %.2f' % (i, np.linalg.norm(f[:,i]-fRef[:,i])/N);
        plt.figure(); plt.plot(time, f[:,i]); plt.plot(time, fRef[:,i],'r--'); 
#        plt.figure(); plt.plot(time, f[:,i]-fRef[:,i],'r-'); 
        plt.xlabel('Time [s]');
        plt.ylabel('Force [N]');
        if(i<3):
            plt.ylim([min_f, max_f]);
        else:
            plt.ylim([min_m, max_m]);
        leg = plt.legend(['Force', 'Desired force'], loc='best');
        leg.get_frame().set_alpha(plut.LEGEND_ALPHA);
        title = 'Force axis '+str(i)+' f VS fRef';
        plut.saveCurrentFigure(title);
        plt.title(title);

FOLDER_ID = 10;
EST_DELAY = 0.04;       ''' delay introduced by the estimation in seconds '''
NJ = 30;                ''' number of joints '''
DT = 0.001;             ''' sampling period '''
PLOT_DATA = True;
plut.SAVE_FIGURES = True;
N_INIT = 0;             ''' first sample to use '''
N_MAX = -1;             ''' max number of samples to use (negative means no limit) '''
FORCE_ESTIMATE_RECOMPUTATION = False;
NEGLECT_GYROSCOPE = True;
NEGLECT_ACCELEROMETER = False;
JOINT_ID = np.array(range(6)); ''' IDs of the joints to save '''
FT_SENSOR_OFFSETS = 24*(0,);
USE_TORQUE_CTRL = True;
k_s = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
k_p = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]);
k_v = np.array([0.012999999999999999, 0.006332, 0.0070000000000000001, 0.006561, 0.0069280000000000001, 0.0060000000000000001, 0.0, 0.0, 0.007561, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

if(FOLDER_ID==1):
    data_folder = '20150617_115259_adm_ctrl_z_force_sin_300N_kf_005';
    USE_TORQUE_CTRL = False;
elif(FOLDER_ID==2):
    data_folder = '20150618_161110_adm_ctrl_zmp_sin_kf_008_tt_2';
    USE_TORQUE_CTRL = False;
    FT_SENSOR_OFFSETS = (-0.25803891255892331, 0.34893698067370726, 9.5586487496531589, -0.010963988277572827, 0.12616441728508024, -0.095441483209842831, 0.14130377550003242, 0.29186202052555654, 9.1720207142777479, 0.021541499621588377, 0.10748135976805924, -0.0062602131114518032, -4.6943207209520788, -7.7852135656700776, 7.7185208032116206, -0.45695906866903535, 0.37958489350312763, 0.033497931736880995, -3.4077405415469748, 9.385957482245427, 7.8045929052694518, 0.54418924646387923, 0.21164237486669485, -0.050856762053280831);
elif(FOLDER_ID==3):
    data_folder = '20150618_171007_force_ctrl_zmp_sin_kf_1_tt_3';
    USE_TORQUE_CTRL = False;
    FT_SENSOR_OFFSETS = (-0.35015835865875633, 0.34564663661057349, 10.166711727781307, -0.03958873550297054, 0.13252000584783169, -0.040668818211910193, 0.11574825724265021, 0.47719708054978721, 9.3213743828484503, 0.03779712152546541, 0.071589059142790304, 0.018008064295673896, -4.1115537447274013, -7.7903074341423046, 7.4035011799377211, -0.4439099465207515, 0.28860269831347263, 0.068492026731883177, -3.111672044556502, 9.280329921252207, 7.662339987053846, 0.54521188536766962, 0.16018198194528482, -0.068076805660857845);
elif(FOLDER_ID==4):
    data_folder = '20150619_170103_adm_ctrl_zmp_sin_15Nm';
    USE_TORQUE_CTRL = False;
    FT_SENSOR_OFFSETS = (0.67113017065484792, -0.093123790840048071, 6.5748420372619307, -0.043834378448082342, 0.2192938325512104, -0.11210593671064101, 0.96180525468985756, 0.64291964008947922, 1.4722479182740182, 0.22110365427281353, 0.28478470243710713, 0.062226180674346335, -5.8976458837816459, -13.773502924174652, 7.280088177341244, -0.80873982941937028, 0.50531824351613497, 0.026505504525980723, -6.6228656936073298, 12.829412325038932, 3.8575556635487844, 0.67858531332915561, 0.39891239714619475, -0.07170272984011912);
elif(FOLDER_ID==5):
    data_folder = '20150619_172209_force_ctrl_zmp_sin_15Nm';
    FT_SENSOR_OFFSETS = (0.55936633982100814, -0.30134880409581605, 5.8399134206977985, 0.033729892656392264, 0.097189085801329775, -0.078415739216042127, 0.77933410995905172, 0.86156294969123148, 1.3738634670790049, 0.23710603963062246, 0.53784085327824138, 0.049435619850016231, -5.3177507815564224, -13.712652946717435, 6.7144232680485123, -0.78127216755860074, 0.40651408410438905, 0.056879176406262055, -6.9870092620146558, 13.178751725465789, 4.2743027445189936, 0.70946952776895467, 0.43080160693458114, -0.060974886871219297);
    k_d = np.array([15.0, 35.0, 15.0, 25.0, 10.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    k_f = np.array([0.29999999999999999, 0.29999999999999999, 2.0, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999]);
    k_i = np.array([0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001, 0.00020000000000000001]);
    k_tau = np.array([0.000212, 3.0000000000000001e-05, 0.00012, 5.1e-05, 0.00017699999999999999, 0.00024000000000000001, 0.0, 0.0, 8.3999999999999995e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
elif(FOLDER_ID==6):
    data_folder = '20150622_120244_force_ctrl_z_sin_200N';
    N_MAX = 30*1000;
    FT_SENSOR_OFFSETS = (0.11163059113781248, -0.13830343782134111, 9.6152785228032407, -0.016921945126330201, -0.058691075936893697, 0.023886055749320716, -0.34710651108130991, 0.011380131368871815, 8.6038159139452279, 0.11749017221258912, 0.013581120127724585, -0.028058405020922519, -4.7059416767808431, -10.263308810484057, 8.0566653295934696, -0.62015746319142373, 0.35684230460894623, 0.049337131732377887, -5.8675343325310907, 10.24746542610022, 6.6147324133040701, 0.59967461800314004, 0.31977265358025742, -0.0052908233354046746);
    k_d = np.array([30.0, 70.0, 30.0, 50.0, 20.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    k_f = np.array([0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999, 0.29999999999999999]);
    k_i = np.array([0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]);
    k_tau = np.array([0.000212, 3.0000000000000001e-05, 0.00020900000000000001, 6.7999999999999999e-05, 0.00017699999999999999, 0.00024000000000000001, 0.0, 0.0, 8.3999999999999995e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
elif(FOLDER_ID==7):
    data_folder = '20150622_131959_adm_ctrl_z_sin_200N';
    N_MAX = 30*1000;
    USE_TORQUE_CTRL = False;
    FT_SENSOR_OFFSETS = (0.3569464850556453, -0.16524362456986438, 8.5035927702557945, -0.048277512380837932, 0.13868959782994777, 0.0067763569005248317, 1.635158290482672, 0.57409332536349855, 3.7996565853521411, 0.074101708845542527, 0.44045549037960718, -0.010423953227082443, -4.9580924405046396, -10.274804495538802, 8.5012796247699214, -0.63478935291394689, 0.37802820185525121, 0.043733679469091805, -5.7021209159725039, 10.278693151152517, 6.4942268225819628, 0.61087055891440545, 0.30608699674639672, -0.011422936519029765);
elif(FOLDER_ID==9):
    data_folder = '20150624_153958_force_ctrl_fixed_fz_sin_70N_kf_1_ki_0032';
    N_INIT = 40*1000;
    N_MAX = 80*1000;
    USE_TORQUE_CTRL = False;
    FT_SENSOR_OFFSETS = (0.3167813010973024, -0.15802378757242125, 10.74864053514623, -0.1027392578858498, -0.069576634407988031, -0.043787212893826172, 1.7506530902874688, 0.93049659203177937, 10.380544057789002, -0.043151144483774176, 0.44009613537349451, 0.0030331431414830344, -2.1619742586633022, -5.9124291399041864, 5.9281285326604642, -0.3184490578581774, 0.1671325439139108, 0.04821209777488504, -3.4187799534855143, 4.814519032594621, 6.475659395120716, 0.28119249090630183, 0.21411159046764539, -0.043677931102360706);
    k_d=np.array([30.0, 70.0, 30.0, 50.0, 20.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    k_f =np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
    k_i=np.array([0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002, 0.0032000000000000002])
    k_tau=np.array([0.000212, 3.0000000000000001e-05, 0.00031100000000000002, 9.0000000000000006e-05, 0.00017699999999999999, 0.00024000000000000001, 0.0, 0.0, 8.3999999999999995e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
elif(FOLDER_ID==10):
    data_folder = '20150624_155045_adm_ctrl_fixed_fz_sin_70N_kf_005';
    N_MAX = 40*1000;
    USE_TORQUE_CTRL = False;
    FT_SENSOR_OFFSETS = (-0.028519816682695001, 0.21222076701116588, 9.7957729329948808, -0.021271625216717983, -0.10253292155092933, -0.041334226549629489, -0.1349525961322359, 0.22506415478344668, 9.4525206787522169, 0.020900866789873143, 0.3194157128264804, 0.03521947383775826, -2.2349959559942105, -5.6939866752745258, 5.7209405276445846, -0.30014399257673779, 0.17298989305585005, 0.04333550913581611, -3.459541904773114, 4.5782900565454403, 5.7893403601425231, 0.25406126452570355, 0.20807692157419438, -0.042499241242810869);
elif(FOLDER_ID==11):
    data_folder = '20150624_170744_force_ctrl_fixed_fz_sin_70N_kf_05_ki_015';
    N_MAX = 15*1000;
    USE_TORQUE_CTRL = False;
    FT_SENSOR_OFFSETS = (1.1485669162718557, 1.4721480657015067, 9.7011332043183245, -0.060114374839545848, -0.18900062545791579, 0.055257043144740933, 1.9170522093775426, 0.89004172772506251, 9.8783556603316782, 0.00064585196813472323, 0.41263230217320046, 0.042996494916329543, -2.2265210243947973, -5.7633633097975903, 5.7594696589466192, -0.30292226577201076, 0.1710723556528074, 0.044797046217611151, -3.5074360222181422, 4.6253369968314688, 5.849686945016912, 0.2556724678324519, 0.20956883193209869, -0.041065188472155095);
    
    
data_folder = '../results/'+data_folder+'/';
FILE_READ_SUCCEEDED = False;    
DATA_FILE_NAME = 'data';
TEXT_DATA_FILE_NAME = 'data.txt';
plut.FIGURE_PATH = data_folder;
N_DELAY = int(EST_DELAY/DT);

file_name_qDes    = 'dg_HRP2LAAS-control.dat';
file_name_enc     = 'dg_HRP2LAAS-robotState.dat';
file_name_acc     = 'dg_HRP2LAAS-accelerometer.dat';
file_name_gyro    = 'dg_HRP2LAAS-gyrometer.dat';
file_name_forceLA = 'dg_HRP2LAAS-forceLARM.dat';
file_name_forceRA = 'dg_HRP2LAAS-forceRARM.dat';
file_name_forceLL = 'dg_HRP2LAAS-forceLLEG.dat';
file_name_forceRL = 'dg_HRP2LAAS-forceRLEG.dat';
file_name_forceRL_ref   = 'dg_jtg-fRightFoot.dat';
file_name_delta_q_ff    = 'dg_jtc-deltaQ_ff.dat';
    
''' Load data from file '''
try:
    data = np.load(data_folder+DATA_FILE_NAME+'.npz');
    time = data['time'];
    enc = data['enc'];
    acc = data['acc'];
    gyro = data['gyro'];
    forceLA = data['forceLA'];
    forceRA = data['forceRA'];
    forceLL = data['forceLL'];
    forceRL = data['forceRL'];
    forceRL_ref = data['forceRL_ref'];

    N = acc.shape[0];    
    qDes = np.empty((N,len(JOINT_ID)));
    dq = np.empty((N,len(JOINT_ID)));
    ddq = np.empty((N,len(JOINT_ID)));
    tau = np.empty((N,len(JOINT_ID)));
    
    FILE_READ_SUCCEEDED = True; 
    
    for i in range(len(JOINT_ID)):
        data = np.load(data_folder+DATA_FILE_NAME+'_j'+str(JOINT_ID[i])+'.npz');    
        qDes[:,i] = data['qDes'];
        if(FORCE_ESTIMATE_RECOMPUTATION==False):
            dq[:,i] = data['dq'];
            ddq[:,i] = data['ddq'];
            tau[:,i] = data['tau'];

except (IOError, KeyError):
    print 'Gonna read text files...'
    
    qDes    = np.loadtxt(data_folder+file_name_qDes);
    enc     = np.loadtxt(data_folder+file_name_enc);
    acc     = np.loadtxt(data_folder+file_name_acc);
    gyro    = np.loadtxt(data_folder+file_name_gyro);
    forceLA = np.loadtxt(data_folder+file_name_forceLA);
    forceRA = np.loadtxt(data_folder+file_name_forceRA);
    forceLL = np.loadtxt(data_folder+file_name_forceLL);
    forceRL = np.loadtxt(data_folder+file_name_forceRL);
    forceRL_ref = np.loadtxt(data_folder+file_name_forceRL_ref);
    
    # check that largest signal has same length of smallest signal
    n_enc  = len(enc[:,0]);
    n_acc  = len(acc[:,0]);
    if(n_acc!=n_enc):
        print "Reducing size of signals from %d to %d" % (n_acc, n_enc);
    N = np.min([n_enc,n_acc]);
    if(N_MAX>0 and N>N_MAX):
        N = N_MAX;
    time = enc[N_INIT:N,0];
    qDes = qDes[N_INIT:N,1:];
    qDes = qDes[:,JOINT_ID].reshape(N-N_INIT,len(JOINT_ID));
    enc  = enc[N_INIT:N,7:];
    acc  = acc[N_INIT:N,1:];
    gyro = gyro[N_INIT:N,1:];
    forceLA = forceLA[N_INIT:N,1:];
    forceRA = forceRA[N_INIT:N,1:];
    forceLL = forceLL[N_INIT:N,1:];
    forceRL = forceRL[N_INIT:N,1:];
    forceRL_ref = forceRL_ref[N_INIT:N,1:];
    
    # save sensor data
    np.savez(data_folder+DATA_FILE_NAME+'.npz', 
             time=time, 
             enc=enc.reshape(N-N_INIT,NJ), 
             acc=acc.reshape(N-N_INIT,3), 
             gyro=gyro.reshape(N-N_INIT,3), 
             forceLA=forceLA.reshape(N-N_INIT,6), 
             forceRA=forceRA.reshape(N-N_INIT,6), 
             forceLL=forceLL.reshape(N-N_INIT,6), 
             forceRL=forceRL.reshape(N-N_INIT,6),
             forceRL_ref=forceRL_ref.reshape(N-N_INIT,6));

delta_q_ff = np.loadtxt(data_folder+file_name_delta_q_ff);
delta_q_ff = delta_q_ff[N_INIT:N,1:];
delta_q = qDes-enc[:,JOINT_ID];
N = len(enc[:,0]);

if(FORCE_ESTIMATE_RECOMPUTATION or FILE_READ_SUCCEEDED==False):
    print 'Gonna estimate dq, ddq, tau';
    dt='f4';
    a = np.zeros(N, dtype=[ ('enc',dt,NJ)
                           ,('forceLA',dt,6)
                           ,('forceRA',dt,6)
                           ,('forceLL',dt,6)
                           ,('forceRL',dt,6)
                           ,('acc',dt,3)
                           ,('gyro',dt,3)
                           ,('time',dt,1)]);
    a['enc']      = enc;
    a['forceLA']  = forceLA;
    a['forceRA']  = forceRA;
    a['forceLL']  = forceLL;
    a['forceRL']  = forceRL;
    if(NEGLECT_ACCELEROMETER):
        a['acc']      = np.mean(acc,0);
    else:
        a['acc']      = acc;
    if(NEGLECT_GYROSCOPE==False):
        a['gyro']     = gyro;
    a['time']     = np.squeeze(time*DT);
    (tau, dq, ddq) = compute_estimates_from_sensors(a, EST_DELAY, FT_SENSOR_OFFSETS);
    
    # shift estimate backward in time to compensate for estimation delay
    dq[:-N_DELAY,:]  = dq[N_DELAY::,:];
    ddq[:-N_DELAY,:] = ddq[N_DELAY::,:];
    tau[:-N_DELAY,:] = tau[N_DELAY::,:];
    # set last N_DELAY sample to constant value
    dq[-N_DELAY:,:]  = dq[-N_DELAY,:];
    ddq[-N_DELAY:,:] = ddq[-N_DELAY,:];
    tau[-N_DELAY:,:] = tau[-N_DELAY,:];
    # eliminate data of joints not to save
    dq   = dq[:,JOINT_ID].reshape(N,len(JOINT_ID));
    ddq  = ddq[:,JOINT_ID].reshape(N,len(JOINT_ID));
    tau  = tau[:,JOINT_ID].reshape(N,len(JOINT_ID));    
    
    for i in range(len(JOINT_ID)):
        np.savez(data_folder+DATA_FILE_NAME+'_j'+str(JOINT_ID[i])+'.npz', qDes=qDes[:,i], 
                 enc=enc[:,JOINT_ID[i]], tau=tau[:,i], dq=dq[:,i], ddq=ddq[:,i]);


if(USE_TORQUE_CTRL):
    (delta_q_friction, delta_q_fb_pos, delta_q_fb_vel, delta_q_fb_force) = compute_delta_q_components(enc[:,JOINT_ID], delta_q, dq, 
        delta_q_ff[:,JOINT_ID], enc[:,JOINT_ID], np.zeros(delta_q.shape), 
        k_tau[:,JOINT_ID], k_v[:,JOINT_ID], k_p[:,JOINT_ID], k_s[:,JOINT_ID], k_d[:,JOINT_ID]);

if(PLOT_DATA):
    ''' Plot data '''
    time = np.arange(0,N*DT,DT);
    plotForceTracking(time, forceRL, forceRL_ref);
            
    

#    plt.figure(); plt.plot(acc); plt.title('Acc');
#    plt.figure(); plt.plot(gyro); plt.title('Gyro');
#    plt.figure(); plt.plot(forceLA); plt.title('Force Left Arm');
#    plt.figure(); plt.plot(forceRA); plt.title('Force Right Arm');
#    plt.figure(); plt.plot(forceLL); plt.title('Force Left Leg');

    if(USE_TORQUE_CTRL):
        for i in range(len(JOINT_ID)):
            plt.figure();
            plt.plot(time, 1e3*delta_q_ff[:,i]);
            plt.plot(time, 1e3*delta_q_friction[:,i]); 
            plt.plot(time, 1e3*delta_q_fb_force[:,i]);
            plt.plot(time, 1e3*delta_q_fb_vel[:,i]);    
            plt.plot(time, 1e3*(delta_q[:,i])); 
            plt.xlabel('Time [s]');
            plt.ylabel('Delta_q [rad*1000]');
            leg = plt.legend(['ff torque', 'ff friction', 'fb force', 'fb vel', 'total']);
            leg.get_frame().set_alpha(plut.LEGEND_ALPHA);
            title = 'Joint '+str(JOINT_ID[i])+' delta_q components';
            plut.saveCurrentFigure(title);
            plt.title(title);
            
        
    #        plt.figure(); plt.plot(enc[:,JOINT_ID[i]]-qDes[:,i]); plt.title('Delta_q '+str(JOINT_ID[i]));
    #        plt.figure(); plt.plot(dq[:,i]); plt.title('Joint velocity '+str(JOINT_ID[i]));
    #        plt.figure(); plt.plot(tau[:,i]); plt.title('Joint torque '+str(JOINT_ID[i]));

#        for i in range(len(JOINT_ID)):    
    #        f, ax = plt.subplots(1,1,sharex=True);
    #        ax.plot(tau[:,i], qDes[:,i]-enc[:,JOINT_ID[i]],'r. ');
    #        tau_model = np.array([tau_min[i], tau_max[i]]);
    #        delta_q_model = k_tau[i]*tau_model;
    #        ax.plot(tau_model, delta_q_model,'b');
    #        ax.set_title('Torque VS delta_q '+str(JOINT_ID[i]));
    #        ax.set_xlabel('Torque [Nm]');
    #        ax.set_ylabel('Delta_q [rad]');
            
        ''' 3D PLOT '''
        tau_min = np.min(tau,0);
        tau_max = np.max(tau,0);    
        for i in range(len(JOINT_ID)):
            x_grid = np.linspace(min(dq[:,i]), max(dq[:,i]), 15);
            y_grid = np.linspace(min(tau[:,i]), max(tau[:,i]), 15);
            X, Y = np.meshgrid(x_grid, y_grid);
            Z = k_v[i]*X + k_tau[i]*Y;
            fig = plt.figure();
            ax = fig.add_subplot(111, projection='3d');
            plut.plot3d(dq[:,i], tau[:,i], delta_q[:,i], 'Joit '+str(JOINT_ID[i]), ['dq','tau','delta q'],'r ', ax=ax);
            ax.plot_wireframe(X,Y,Z);
    #        plt.figure(); plt.plot(tau[:,i], currents[:,i]); plt.title('Torque VS current '+str(JOINT_ID[i]));
    #        plt.figure(); plt.plot(p_gains[:,i]); plt.title('Proportional gain '+str(JOINT_ID[i]));
    plt.show(); 

