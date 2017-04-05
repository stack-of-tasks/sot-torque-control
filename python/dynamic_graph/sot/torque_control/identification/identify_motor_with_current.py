# -*- coding: utf-8 -*-
from scipy import signal
from scipy import ndimage
import numpy as np
import sys
from IPython import embed
from plot_utils import plot3d
from plot_utils import plot_x_vs_y
from plot_utils import saveCurrentFigure
import plot_utils
import matplotlib.pyplot as plt
from motor_model import Motor_model
from scipy.cluster.vq import kmeans
import hrp2_motors_parameters


'''
motor model :
i(t) = Kt*tau(t) + Kv*dq(t) + Ka*ddq(t) + Kf*Sign(dq)
'''

#~ motor.display()
#~ # Test smooth tauf
#~ dqs=[]
#~ taufs=[]
#~ for dq in np.linspace(-1,1,1000):
    #~ dqs.append(dq)
    #~ taufs.append(motor.getCurrent(dq))
#~ plt.plot(dqs,taufs)
#~ plt.show()
jID = { "rhy" : 0,
        "rhr" : 1,
        "rhp" : 2,
        "rk"  : 3,
        "rap" : 4,
        "rar" : 5,
        "lhy" : 6,
        "lhr" : 7,
        "lhp" : 8,
        "lk"  : 9,
        "lap" : 10,
        "lar" : 11,
        "ty"  : 12,
        "tp"  : 13,
        "hy"  : 14,
        "hp"  : 15,
        "rsp" : 16,
        "rsr" : 17,
        "rsy" : 18,
        "re"  : 19,
        "rwy" : 20,
        "rwp" : 21,
        "rh"  : 22,
        "lsp" : 23,
        "lsr" : 24,
        "lsy" : 25,
        "le"  : 26,
        "lwy" : 27,
        "lwp" : 28,
        "lh"  : 29 }

Kt_p=np.array(30*(1.0,))
Kt_n=np.array(30*(1.0,))

Kf_p=np.array(30*(0.0,))
Kf_n=np.array(30*(0.0,))

Kv_p=np.array(30*(0.0,))
Kv_n=np.array(30*(0.0,))

Ka_p=np.array(30*(0.0,))
Ka_n=np.array(30*(0.0,))


#~ DZ=70
DZ=0.0


''' Solve the least square problem:
    solve   y=ax+b in L2 norm
'''
def solve1stOrderLeastSquare(x,y):
    Q=np.vstack([np.ones(len(x)),x])
    coef = solveLeastSquare(Q.T,y)
    (a,b)=coef[1,0],coef[0,0]
    return (a,b);

''' Solve the least square problem:
    minimize   || A*x-b ||^2
'''
def solveLeastSquare(A, b):
    return np.linalg.pinv(A)*np.matrix(b).T;

ZERO_VELOCITY_THRESHOLD     = 0.1
ZERO_ACCELERATION_THRESHOLD = 1.
ZERO_JERK_THRESHOLD         = 3.0
SHOW_THRESHOLD_EFFECT = True

USING_CONTROL_AS_CURRENT_MEASURE = False
INVERT_CURRENT = False

#~ JOINT_NAME = 'rhy';  
#~ JOINT_NAME = 'rhr'; 
#~ JOINT_NAME = 'rhp'; 
#~ JOINT_NAME = 'rk'; 
#~ JOINT_NAME = 'rap'; 
#~ JOINT_NAME = 'rar'; 

#~ JOINT_NAME = 'lhy';  USING_CONTROL_AS_CURRENT_MEASURE = True  # 6   
#~ JOINT_NAME = 'lhr';  USING_CONTROL_AS_CURRENT_MEASURE = True  # 7   
#~ JOINT_NAME = 'lhp';  USING_CONTROL_AS_CURRENT_MEASURE = True  # 8 
#~ JOINT_NAME = 'lk';  # 9 ok
#~ JOINT_NAME = 'lap'; # 10 ok
#~ JOINT_NAME = 'lar'; # 11 ok

#~ IDENTIFICATION_MODE='static'
#~ IDENTIFICATION_MODE='vel'
#~ IDENTIFICATION_MODE='acc'

IDENTIFICATION_MODE='low_level'
JOINT_NAME = 'rk'
data_folder='../../results/20161114_153220_rk_vel/'
#~ data_folder='../../results/20161114_151812_rhp_vel/'
#~ data_folder='../../results/20170203_164133_com_sin_z_001/'
#Compare Model Vs Measurment
#~ IDENTIFICATION_MODE='test_model'
#~ JOINT_NAME = 'rhp'
#~ data_folder='../../results/20170203_164133_com_sin_z_001/'
#~ data_folder= '../../results/20161114_152706_rk_acc/' ; INVERT_CURRENT = True
if (IDENTIFICATION_MODE != 'test_model') :
    Nvel = 10
    if(JOINT_NAME == 'rhy' ):
        INVERT_CURRENT = True
        Nvel = 9
        data_folder_static = '../../results/20161114_135332_rhy_static/';
        data_folder_vel    = '../../results/20161114_143152_rhy_vel/';
        data_folder_acc    = '../../results/20161114_142351_rhy_acc/';
    if(JOINT_NAME == 'rhr' ):
        INVERT_CURRENT = True
        Nvel = 10
        data_folder_static = '../../results/20161114_144232_rhr_static/';
        data_folder_vel    = '../../results/20161114_150356_rhr_vel/';
        data_folder_acc    = '../../results/20161114_145456_rhr_acc/';
    if(JOINT_NAME == 'rhp' ):
        data_folder_static = '../../results/20161114_150722_rhp_static/';
        data_folder_vel    = '../../results/20161114_151812_rhp_vel/';
        data_folder_acc    = '../../results/20161114_151259_rhp_acc/';
    if(JOINT_NAME == 'rk' ):
        INVERT_CURRENT = True
        data_folder_static = '../../results/20161114_152140_rk_static/';
        data_folder_vel    = '../../results/20161114_153220_rk_vel/';
        data_folder_acc    = '../../results/20161114_152706_rk_acc/';
    if(JOINT_NAME == 'rap' ):
        INVERT_CURRENT = True
        data_folder_static = '../../results/20161114_153739_rap_static/';
        data_folder_vel    = '../../results/20161114_154559_rap_vel/';
        data_folder_acc    = '../../results/20161114_154316_rap_acc/';
    if(JOINT_NAME == 'rar' ):
        data_folder_static = '../../results/20161114_154945_rar_static/';
        data_folder_vel    = '../../results/20161114_160038_rar_vel/';
        data_folder_acc    = '../../results/20161114_155545_rar_acc/';

    if(JOINT_NAME == 'lhy' ):
        data_folder_static = '../../results/20170113_144220_lhy_static/';
        data_folder_vel    = '../../results//';
        data_folder_acc    = '../../results/20170113_144710_lhy_const_acc/';
    if(JOINT_NAME == 'lhr' ):
        data_folder_static = '../../results/20170113_145227_lhr_static/';
        data_folder_vel    = '../../results/20170113_150215_lhr_const_vel/';
        data_folder_acc    = '../../results/20170113_145826_lhr_const_acc/';
    if(JOINT_NAME == 'lhp' ):
        data_folder_static = '../../results/20170113_150628_lhp_static/';
        data_folder_vel    = '../../results/20170113_151433_lhp_const_vel/';
        data_folder_acc    = '../../results/20170113_151103_lhp_const_acc/';
    if(JOINT_NAME == 'lk' ):
        data_folder_static = '../../results/20170113_151748_lk_static/';
        data_folder_vel    = '../../results/20170113_152924_lk_const_vel/';
        data_folder_acc    = '../../results/20170113_152606_lk_const_acc/';
    if(JOINT_NAME == 'lap' ):
        data_folder_static = '../../results/20170113_154007_lap_static/';
        data_folder_vel    = '../../results/20170113_154834_lap_const_vel/';
        data_folder_acc    = '../../results/20170113_154303_lap_const_acc/';
    if(JOINT_NAME == 'lar' ):
        data_folder_static = '../../results/20170113_155150_lar_static/';
        data_folder_vel    = '../../results/20170113_160057_lar_const_vel/';
        data_folder_acc    = '../../results/20170113_155706_lar_const_acc/';

if (IDENTIFICATION_MODE=='static') : data_folder = data_folder_static
if (IDENTIFICATION_MODE=='vel')    : data_folder = data_folder_vel
if (IDENTIFICATION_MODE=='acc')    : data_folder = data_folder_acc

JOINT_ID = jID[JOINT_NAME]

DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';

#~ 
#~ PLOT_LOW_VEL_DATA_3D    = True;
#~ PLOT_LOW_VEL_DATA       = True;
#~ PLOT_RESIDUALS          = False;
#~ PLOT_3D_SURFACE         = True;
#~ PLOT_FRICTION_DATA      = False;
#~ PLOT_PIECE_WISE_LINEAR  = True;
#~ PLOT_ANIMATED_DATA      = False;
#~ COMPARE_TWO_DATASETS    = False;
#~ SHOW_PLOTS              = True;

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
        ctrl = np.squeeze(data['ctrl']);
        current = np.squeeze(data['current']);
    else:
        enc = np.squeeze(data['enc'][:,JOINT_ID]);
        dq = np.squeeze(data['dq'][:,JOINT_ID]);
        tau = np.squeeze(data['tau'][:,JOINT_ID]);
        ctrl = np.squeeze(data['ctrl'][:,JOINT_ID]);
        current = np.squeeze(data['current'][:,JOINT_ID]);

except IOError:
    print "Impossible to read data file %s" % (data_folder+DATA_FILE_NAME);
    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");
    
#''' Load sensors data from file '''
try:
    sensor_data = np.load(data_folder+'data.npz');
    acc = sensor_data['acc'];
    gyro = sensor_data['gyro'];
    forceLA = sensor_data['forceLA'];
    forceRA = sensor_data['forceRA'];
    forceLL = sensor_data['forceLL'];
    forceRL = sensor_data['forceRL'];
except IOError:
    print "Impossible to read data file %f" % (data_folder+'data.npz');
    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");
    
if(len(data['enc'].shape)==1):
    enc = np.squeeze(data['enc']);
    dq = np.squeeze(data['dq']);
    ddq = np.squeeze(data['ddq']);
    tau = np.squeeze(data['tau']);
    ctrl = np.squeeze(data['ctrl']);

else:
    enc = np.squeeze(data['enc'][:,JOINT_ID]);
    dq = np.squeeze(data['dq'][:,JOINT_ID]);
    ddq = np.squeeze(data['ddq'][:,JOINT_ID]);
    tau = np.squeeze(data['tau'][:,JOINT_ID]);
    ctrl = np.squeeze(data['ctrl'][:,JOINT_ID]);
    
maskSaturation=np.logical_and( (current>-9.5) , (current<9.5))
if USING_CONTROL_AS_CURRENT_MEASURE:
    maskCtrlNotInDZ = np.logical_and( ctrl >= (DZ *0.8) ,ctrl <= -(DZ*0.8)  )
    maskSaturation=np.logical_or(maskSaturation, maskCtrlNotInDZ)
maskPosVel=(dq> 0.001)
maskNegVel=(dq<-0.001)


if INVERT_CURRENT:
    current = - current

enc     = enc    [maskSaturation];
dq      = dq     [maskSaturation];
ddq     = ddq    [maskSaturation];
tau     = tau    [maskSaturation];
ctrl    = ctrl   [maskSaturation];
current = current[maskSaturation];
maskPosVel = maskPosVel[maskSaturation]
maskNegVel = maskNegVel[maskSaturation]

#~ maskInDZ=abs(ctrl/102.4) < 0.5
#~ tt=np.arange(maskInDZ.size)
#~ plt.plot(tt,ctrl/102.4,'.')
#~ plt.plot(tt,current,'.')
#~ plt.plot(tt[maskInDZ],ctrl[maskInDZ]/102.4,'.')


#~ plt.figure()
#~ plt.plot(current,ctrl/102.4-current)
#~ plt.plot(ctrl,ctrl/102.4-current)
#~ plt.show()
#~ embed()

    #~ plt.ylabel('control')
    #~ plt.xlabel('current')
    #~ plt.plot(ctrl /102.4,current,'x')
    #~ 
    #~ (v1min,v1max) = ( 0.7 , 0.9)
    #~ (v2min,v2max) = (0.25 , 0.3)
    #~ (v3min,v3max) = (0.18 , 0.21)
    #~ 
    #~ maskv1= np.logical_and( dq > v1min , dq <v1max  )
    #~ maskv2= np.logical_and( dq > v2min , dq <v2max  )
    #~ maskv3= np.logical_and( dq > v3min , dq <v3max  )
    #~ 
    #~ plt.plot(ctrl[maskv1] /102.4,current[maskv1],'xr')
    #~ plt.plot(ctrl[maskv2] /102.4,current[maskv2],'xg')
    #~ plt.plot(ctrl[maskv3] /102.4,current[maskv3],'xy')
    #~ plt.show()   

   
if USING_CONTROL_AS_CURRENT_MEASURE:
    current_est = current.copy()
    #~ maskUpDZ = ctrl > DZ
    #~ maskDnDZ = ctrl < DZ
    #~ maskInDZ = np.logical_and( ctrl <= DZ ,ctrl >= -DZ )
    current = ctrl /102.4
    #~ current[maskUpDZ] =  (ctrl[maskUpDZ]-DZ) /102.4 
    #~ current[maskDnDZ] =  (ctrl[maskDnDZ]+DZ) /102.4 
    #~ current[maskInDZ] =  0.0

    
if(IDENTIFICATION_MODE=='low_level'):
    #Filter current*****************************************************
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from matplotlib import cm
    from matplotlib.ticker import LinearLocator, FormatStrFormatter
    import numpy as np
    
    DZ = 0.4
    K3 = 1.0#1.06
    mask=abs(K3*ctrl/102.4-current)<DZ
    times = np.arange(len(enc))*0.001
    plt.figure()
    
    plt.subplot(411)
    plt.plot(times,current      ,'.')
    plt.plot(times[mask],current[mask],'.')
    plt.title('current')
    
    plt.subplot(412)
    plt.plot(times,dq            ,'.')
    plt.plot(times[mask],dq[mask],'.')
    plt.title('dq')
    
    plt.subplot(413)
    plt.plot(times,ctrl            ,'.')
    plt.plot(times[mask],ctrl[mask],'.')
    plt.title(ctrl)
    plt.title('ctrl')
    
    plt.subplot(414)
    plt.plot(times,K3*ctrl/102.4-current            ,'.')
    plt.plot(times[mask],K3*ctrl[mask]/102.4-current[mask],'.')
    plt.title(ctrl)
    plt.title('ctrl-current ')
    
    
    plt.show()
    
    #~ embed()
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    NDZ= 100
    NK3= 100
    lDZs = np.linspace(0.3,0.5,NDZ)
    lK3s = np.linspace(0.95,1.2,NK3)
    DZs, K3s = np.meshgrid(lDZs, lK3s)
    cost=np.zeros([NDZ,NK3])
    for i in range(NDZ):
        print str(int(1000*i/NDZ)/10.0) + '%'
        for j in range(NK3):
            #~ print (i,j)
            #~ if i == 50: embed()
            DZ=lDZs[i]
            K3=lK3s[j]
            mask=abs(K3*ctrl/102.4-current) < DZ
            not_mask = np.logical_not(mask)           
            #plt.plot(current[mask],dq[mask],'.')
            cost[i,j]=-np.corrcoef(current[mask],dq[mask])[0,1]            #* np.sum(not_mask) / (np.sum(mask) + np.sum(not_mask))
            #~ embed()
            #~ cost[i,j]+=np.corrcoef(ctrl[not_mask],dq[not_mask])[0,1]       * np.sum(mask) / (np.sum(mask) + np.sum(not_mask))
            #~ cost[i,j]+=np.corrcoef(ctrl[not_mask],current[not_mask])[0,1]  * np.sum(mask) / (np.sum(mask) + np.sum(not_mask))
            #~ cost[i,j]=cost[i,j]/3.0

            #~ ax.scatter(DZ, K3, cost,cmap=cm.coolwarm)
    surf = ax.plot_surface(DZs, K3s, cost.T, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
                       
    # Customize the z axis.
    ax.set_zlim(np.min(cost), 1.01)
    ax.zaxis.set_major_locator(LinearLocator(2))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

    # Add a color bar which maps values to colors.
    fig.colorbar(surf, shrink=0.5, aspect=5)                       
    plt.show()
    
    #plot a particular case
    DZ=0.4
    K3=1.0
    mask=abs(K3*ctrl/102.4-current) < DZ
    plt.xlabel('current') 
    plt.ylabel('dq') 
    plt.plot(current[mask],dq[mask],'.')
    plt.show()
    
    #plot the optimum
    iDZ,iK3 = np.unravel_index(np.argmax(cost),cost.shape)
    DZ=lDZs[iDZ]
    K3=lK3s[iK3]
    print 'DZ = ' + str(DZ)
    print 'K3 = ' + str(K3)
    mask=abs(K3*ctrl/102.4-current) < DZ
    plt.xlabel('current') 
    plt.ylabel('dq') 
    plt.plot(current[mask],dq[mask],'.')
    print -np.corrcoef(current[mask],dq[mask])[0,1]
    print cost[iDZ,iK3]
    
    plt.show()
    embed()
#Ktau,Tau0 Identification
if(IDENTIFICATION_MODE=='static'):
    #Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode='same')/sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    #~ current = filtered_current
    # Mask valid data***************************************************
    m = len(dq);
    # remove high velocity
    maskConstAng = (abs (dq)<ZERO_VELOCITY_THRESHOLD)
    # erode to get only steady phases where velocity is small 
    maskConstAng=ndimage.morphology.binary_erosion(maskConstAng,None,100)
    #~ plt.figure()
    #~ plt.plot(ddq);
    maskConstPosAng=np.logical_and( maskConstAng ,maskPosVel )
    maskConstNegAng=np.logical_and( maskConstAng ,maskNegVel ) 
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(enc); plt.ylabel('q')
        q_const=enc.copy()
        q_const[np.logical_not(maskConstAng)]=np.nan
        plt.plot(q_const); plt.ylabel('q_const')
        
    # plot dead zone effect ********************************************
    plt.figure()
    plt.plot(current)
    plt.plot(ctrl/102.4)
    
    
    plt.figure()
    y = current
    y_label = r'$i(t)$'
    x = ctrl/102.4 - current
    x_label =r'$ctrl(t)-i(t)$'
    plt.ylabel(y_label)
    plt.xlabel(x_label)    
    plt.plot(x,y,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosAng],y[maskConstPosAng],'rx',lw=3,markersize=1); 
    plt.plot(x[maskConstNegAng],y[maskConstNegAng],'bx',lw=3,markersize=1); 
    
    plt.figure()
    y = ctrl/102.4
    y_label = r'$ctrl(t)$'
    x = ctrl/102.4 - current
    x_label =r'$ctrl(t)-i(t)$'
    plt.ylabel(y_label)
    plt.xlabel(x_label)    
    plt.plot(x,y,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosAng],y[maskConstPosAng],'rx',lw=3,markersize=1); 
    plt.plot(x[maskConstNegAng],y[maskConstNegAng],'bx',lw=3,markersize=1); 
    
    plt.figure()
    y = ctrl/102.4
    y_label = r'$ctrl(t)$'
    x = current
    x_label =r'$i(t)$'
    plt.ylabel(y_label)
    plt.xlabel(x_label)    
    plt.plot(x,y,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot([-3,3],[-3,3]);  
    
    
    plt.show()
    embed()
    #~ y = a. x   +  b
    #~ i = Kt.tau + Kf
    #~ 
    # Identification ***************************************************
    y = current
    y_label = r'$i(t)$'
    x = tau
    x_label =r'$\tau(t)$'
    (a,b)=solve1stOrderLeastSquare(x[maskConstPosAng],y[maskConstPosAng])
    Ktp=a
    Kfp=b
    (a,b)=solve1stOrderLeastSquare(x[maskConstNegAng],y[maskConstNegAng])
    Ktn=a
    Kfn=-b
    
    # Plot *************************************************************
    plt.figure()    
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    plt.plot(x     ,y     ,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosAng],y[maskConstPosAng],'rx',lw=3,markersize=1); 
    plt.plot(x[maskConstNegAng],y[maskConstNegAng],'bx',lw=3,markersize=1); 
    #plot identified lin model
    plt.plot([min(x),max(x)],[Ktp*min(x)+Kfp ,Ktp*max(x)+Kfp],'g:',lw=3)
    plt.plot([min(x),max(x)],[Ktn*min(x)-Kfn ,Ktn*max(x)-Kfn],'g:',lw=3)
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    plt.title('Static experiment - Joint '+JOINT_NAME)
    print 'Kt_p[%d] = %f' % (JOINT_ID,Ktp);
    print 'Kt_n[%d] = %f' % (JOINT_ID,Ktn);
    print 'Kf_p[%d] = %f' % (JOINT_ID,Kfp);
    print 'Kf_n[%d] = %f' % (JOINT_ID,Kfn);
    
    print 'Kt_m[%d] = %f' % (JOINT_ID,(Ktp+Ktn)/2.0);
    print 'Kf_m[%d] = %f' % (JOINT_ID,(Kfp+Kfn)/2.0);
    #save parameters for next identification level**********************
    np.savez(data_folder+'motor_param_'+JOINT_NAME+'.npz',Ktp=Ktp,Ktn=Ktn)
    plt.savefig(data_folder+"static_"+JOINT_NAME+".jpg")
    plt.show()

#Kd Identification 
if(IDENTIFICATION_MODE=='vel'):
    #load parameters from last identification level*********************
    try:
        data_motor_param = np.load(data_folder_static+'motor_param_'+JOINT_NAME+'.npz')
        Kt_p[JOINT_ID]=(data_motor_param['Ktp'].item())
        Kt_n[JOINT_ID]=(data_motor_param['Ktn'].item())
    except IOError:
        print "Impossible to read data file %s" % (data_folder_static+'motor_param_'+JOINT_NAME+'.npz');
        sys.exit("Run identification on static experiments.");
    
    #Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode='same') / sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    #~ current = filtered_current
    #~ plt.show()
    # Mask valid data***************************************************
    m = len(dq);
    # remove high acceleration
    maskConstVel = np.logical_and( (abs (ddq)<ZERO_ACCELERATION_THRESHOLD) , (abs (dq)>ZERO_VELOCITY_THRESHOLD))
    # erode to get only steady phases where velocity is constant 
    maskConstVel=ndimage.morphology.binary_erosion(maskConstVel,None,100)
    #~ plt.figure()
    #~ plt.plot(ddq);
    maskConstPosVel=np.logical_and( maskConstVel ,maskPosVel )
    maskConstNegVel=np.logical_and( maskConstVel ,maskNegVel ) 
    
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(dq); plt.ylabel('dq')
        dq_const=dq.copy()
        dq_const[np.logical_not(maskConstVel)]=np.nan
        plt.plot(dq_const); plt.ylabel('dq_const')

    # Identification od BEMF effect ************************************
    times = np.arange(len(enc))*0.001
    plt.subplot(221)
    plt.plot(times,dq,lw=1)
    vels = kmeans(dq[maskConstVel],Nvel)
    #~ print 'Velocity founds are:'
    #~ print (vels)
    couleurs = [ 'g', 'r', 'c', 'm', 'y', 'k'] * 10 #why not?
    masksVels = []
    av_dq = [] #List of point keept for identification of BEMF effect
    av_delta_i = []
    it=0;
    for vel in vels[0]:
        it+=1
        currentMask = np.logical_and( dq > vel-0.1 , dq < vel+0.1  )
        currentMask = np.logical_and( currentMask,maskConstVel  )
        masksVels.append(currentMask)
        plt.subplot(221)
        plt.plot(times[currentMask],dq[currentMask],'o'+couleurs[it])
        plt.subplot(222)
        plt.xlabel('control')
        plt.ylabel('current')
        plt.plot(ctrl[currentMask] /102.4,current[currentMask],'x'+couleurs[it])
        plt.subplot(223)
        plt.xlabel('control - current')
        plt.ylabel('velocity')
        plt.plot(ctrl[currentMask] /102.4-current[currentMask],dq[currentMask],'x'+couleurs[it])
        av_dq.append(      np.mean(dq[currentMask]                               ))
        av_delta_i.append( np.mean(ctrl[currentMask] /102.4-current[currentMask] ))
    plt.plot(av_delta_i,av_dq,'o')
    
    av_dq      = np.array(av_dq)
    av_delta_i = np.array(av_delta_i)
    av_dq_pos = av_dq[av_dq>0]
    av_dq_neg = av_dq[av_dq<0]
    av_delta_i_pos = av_delta_i[av_dq>0]
    av_delta_i_neg = av_delta_i[av_dq<0]
    (ap,bp)=solve1stOrderLeastSquare(av_delta_i_pos,av_dq_pos)
    (an,bn)=solve1stOrderLeastSquare(av_delta_i_neg,av_dq_neg)
    a=(an+ap)/2
    b=(-bp+bn)/2
    DeadZone = b/a ; #the half of the total dead zone
    print "deadzone[%d] = %f;"% (JOINT_ID, DeadZone);
    print "Kpwm[%d] = %f; # in Amp / Rad.s-1"% (JOINT_ID, 1.0/a);
    
    x=np.array(av_delta_i)
    plt.plot([-b/a,b/a],[0. ,0.          ],'g:',lw=3)    
    plt.plot([min(x),-b/a],[a*min(x)+b ,0.          ],'g:',lw=3)    
    plt.plot([b/a,max(x)],[0.,a*max(x)-b],'g:',lw=3)
    plt.show()
    #~ y        = a. x +  b
    #~ i-Kt.tau = Kv.dq + Kf
    #~ 
    # Identification ***************************************************
    y = current-Kt_p[JOINT_ID]*tau
    y[maskConstPosVel] = current[maskConstPosVel]-Kt_p[JOINT_ID]*tau[maskConstPosVel]
    y[maskConstNegVel] = current[maskConstNegVel]-Kt_n[JOINT_ID]*tau[maskConstNegVel]
    y_label = r'$i(t)-{K_t}{\tau(t)}$'
    x = dq
    x_label =r'$\dot{q}(t)$'
    (a,b)=solve1stOrderLeastSquare(x[maskConstPosVel],y[maskConstPosVel])
    Kvp=a
    Kfp=b
    (a,b)=solve1stOrderLeastSquare(x[maskConstNegVel],y[maskConstNegVel])
    Kvn=a
    Kfn=-b

    # Plot *************************************************************
    plt.figure()    
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    plt.plot(x     ,y     ,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosVel],y[maskConstPosVel],'rx',lw=3,markersize=1); 
    plt.plot(x[maskConstNegVel],y[maskConstNegVel],'bx',lw=3,markersize=1); 
    #plot identified lin model
    plt.plot([0.0,max(dq)],[ Kfp,Kvp*max(dq)+Kfp],'g-')
    plt.plot([0.0,min(dq)],[-Kfn,Kvn*min(dq)-Kfn],'g-')
    plt.ylabel(y_label)
    plt.xlabel(x_label)

    Ktp = Kt_p[JOINT_ID]
    Ktn = Kt_n[JOINT_ID]
    warning = ""
    if USING_CONTROL_AS_CURRENT_MEASURE :
        warning = " (Current sensor not used)"
    print 'Kt_p[%d] = %f #Using %s' % (JOINT_ID, Ktp, data_folder_static + warning);
    print 'Kt_n[%d] = %f #Using %s' % (JOINT_ID, Ktn, data_folder_static + warning);
    print 'Kv_p[%d] = %f #Using %s' % (JOINT_ID, Kvp, data_folder_vel + warning);
    print 'Kv_n[%d] = %f #Using %s' % (JOINT_ID, Kvn, data_folder_vel + warning);
    print 'Kf_p[%d] = %f #Using %s' % (JOINT_ID, Kfp, data_folder_vel + warning);
    print 'Kf_n[%d] = %f #Using %s' % (JOINT_ID, Kfn, data_folder_vel + warning);
    
    #~ print 'Kv_m[%d] = %f' % (JOINT_ID,(Kvp+Kvn)/2.0);
    #~ print 'Kf_m[%d] = %f' % (JOINT_ID,(Kfp+Kfn)/2.0);

    plt.savefig(data_folder+"vel_"+JOINT_NAME+".jpg")
    plt.show()

#J Identification
if(IDENTIFICATION_MODE=='acc'):
    #Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode='same') / sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    current = filtered_current
    # Mask valid data***************************************************
    m = len(dq);
    dt=0.001
    #~ # remove high jerk
    dddq = np.gradient(ddq,1)/dt
    maskConstAcc = (abs (dddq)<ZERO_JERK_THRESHOLD)
    #~ # erode to get only steady phases where acceleration is constant 
    maskConstAcc=ndimage.morphology.binary_erosion(maskConstAcc,None,100)
    maskConstPosAcc=np.logical_and( maskConstAcc ,maskPosVel )
    maskConstNegAcc=np.logical_and( maskConstAcc ,maskNegVel )
    
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(ddq); plt.ylabel('ddq')
        ddq_const=ddq.copy()
        ddq_const[np.logical_not(maskConstAcc)]=np.nan
        plt.plot(ddq_const); plt.ylabel('ddq_const')
        plt.show()

    #~ y              = a. x   +  b
    #~ i-Kt.tau-Kv.dq = Ka.ddq +  Kf
    #~ 
    # Identification ***************************************************
    y = current-Kt_p[JOINT_ID]*tau - Kv_p[JOINT_ID]*dq
    y[maskConstPosAcc] = current[maskConstPosAcc]-Kt_p[JOINT_ID]*tau[maskConstPosAcc] - Kv_p[JOINT_ID]*dq[maskConstPosAcc]
    y[maskConstNegAcc] = current[maskConstNegAcc]-Kt_p[JOINT_ID]*tau[maskConstNegAcc] - Kv_p[JOINT_ID]*dq[maskConstNegAcc]
    y_label = r'$i(t)-{K_t}{\tau(t)}-{K_v}{\dot{q}(t)}$'
    x = ddq
    x_label = r'$\ddot{q}(t)$'
    (a,b)=solve1stOrderLeastSquare(x[maskConstPosAcc],y[maskConstPosAcc])
    Kap=a
    Kfp=b
    (a,b)=solve1stOrderLeastSquare(x[maskConstNegAcc],y[maskConstNegAcc])
    Kan=a
    Kfn=-b
    
    # Plot *************************************************************
    plt.figure()    
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    plt.plot(x     ,y     ,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosAcc],y[maskConstPosAcc],'rx',lw=3,markersize=1); 
    plt.plot(x[maskConstNegAcc],y[maskConstNegAcc],'bx',lw=3,markersize=1); 
    
    #plot identified lin model
    plt.plot([min(x),max(x)],[Kap*min(x)+Kfp ,Kap*max(x)+Kfp],'g:',lw=3)
    plt.plot([min(x),max(x)],[Kan*min(x)-Kfn ,Kan*max(x)-Kfn],'g:',lw=3)
    plt.ylabel(y_label)
    plt.xlabel(x_label)

    print 'Ka_p[%d] = %f' % (JOINT_ID,Kap);
    print 'Ka_n[%d] = %f' % (JOINT_ID,Kan);
    print 'Kf_p[%d] = %f' % (JOINT_ID,Kfp);
    print 'Kf_n[%d] = %f' % (JOINT_ID,Kfn);
    
    
    plt.show()

#model vs measurement
if (IDENTIFICATION_MODE=='test_model'):
    #load motor parameters
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
                        Ka_p[JOINT_ID], Ka_n[JOINT_ID],0.01)
    tau_motor=np.zeros(len(tau))
    i_motor=np.zeros(len(current))
    
    for idx in range(len(tau)):
        tau_motor[idx]=motor.getTorque    (current[idx], dq[idx], ddq[idx])
        i_motor[idx]  =motor.getCurrent   (tau[idx],     dq[idx], ddq[idx])
    

    plt.figure()
    plt.plot(tau)
    plt.plot(tau_motor)
    plt.legend(['Estimated torque with dynamic model','Estimated torque with motor model'])
    
    plt.figure()
    
    plt.subplot(211)
    plt.plot(dq)
    plt.subplot(212)
    plt.plot(current)
    plt.plot(i_motor)
    plt.legend(['measured current','Estimated current with model'])
    
    
    
    
    plt.figure()
    plt.plot(current)
    plt.plot(ctrl/102.4)
    plt.show()
