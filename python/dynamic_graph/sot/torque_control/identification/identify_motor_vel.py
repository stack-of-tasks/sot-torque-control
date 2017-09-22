# -*- coding: utf-8 -*-
"""
Created on Tue Sep 12 18:47:50 2017

@author: adelpret
"""
from scipy import signal
from scipy.cluster.vq import kmeans
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt
from identification_utils import solve1stOrderLeastSquare, solveLeastSquare
from dynamic_graph.sot.torque_control.hrp2.control_manager_conf import IN_OUT_GAIN


def identify_motor_vel(dt, dq, ddq, ctrl, current, tau, Ktp, Ktn, Ks, ZERO_VEL_THRESHOLD, 
                       ZERO_ACC_THRESHOLD, Nvel, SHOW_THRESHOLD_EFFECT):
    # Mask valid data***************************************************
    # remove high acceleration
    maskConstVel = np.logical_and( (abs (ddq)<ZERO_ACC_THRESHOLD) , (abs (dq)>ZERO_VEL_THRESHOLD))
    # erode to get only steady phases where velocity is constant 
    maskConstVel=ndimage.morphology.binary_erosion(maskConstVel,None,100)
    maskPosVel=(dq> ZERO_VEL_THRESHOLD)
    maskNegVel=(dq<-ZERO_VEL_THRESHOLD)
    maskConstPosVel=np.logical_and( maskConstVel ,maskPosVel )
    maskConstNegVel=np.logical_and( maskConstVel ,maskNegVel ) 
    
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(dq); plt.ylabel('dq')
        dq_const=dq.copy()
        dq_const[np.logical_not(maskConstVel)]=np.nan
        plt.plot(dq_const); plt.ylabel('dq_const')

    # Identification of BEMF effect ************************************
    times = np.arange(len(dq))*dt
    plt.subplot(221)
    plt.plot(times,dq,lw=1)
    vels = kmeans(dq[maskConstVel],Nvel)
    #print 'Velocity founds are:', vels
    couleurs = [ 'g', 'r', 'c', 'm', 'y', 'k'] * 10 #why not?
    masksVels = []
    av_dq = [] #List of point kept for identification of BEMF effect
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
        plt.plot(ctrl[currentMask] /IN_OUT_GAIN,Ks*current[currentMask],'x'+couleurs[it])
        plt.subplot(223)
        plt.xlabel('control - current')
        plt.ylabel('velocity')
        plt.plot(ctrl[currentMask] /IN_OUT_GAIN-Ks*current[currentMask],dq[currentMask],'x'+couleurs[it])
        av_dq.append(      np.mean(dq[currentMask]                               ))
        av_delta_i.append( np.mean(ctrl[currentMask] /IN_OUT_GAIN-Ks*current[currentMask] ))
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
    K_bemf = 1.0/a;
    
    x=av_delta_i
    plt.plot([-b/a,b/a],[0. ,0.          ],'g:',lw=3)    
    plt.plot([min(x),-b/a],[a*min(x)+b ,0.          ],'g:',lw=3)    
    plt.plot([b/a,max(x)],[0.,a*max(x)-b],'g:',lw=3)
    plt.show()
    #~ y        = a. x +  b
    #~ i-Kt.tau = Kv.dq + Kf
    #~ 
    # Identification with fixed Kt ***************************************************
    y = Ks*current-Ktp*tau
    y[maskConstPosVel] = Ks*current[maskConstPosVel]-Ktp*tau[maskConstPosVel]
    y[maskConstNegVel] = Ks*current[maskConstNegVel]-Ktn*tau[maskConstNegVel]
    x = dq
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
    plt.ylabel(r'$i(t)-{K_t}{\tau(t)}$')
    plt.xlabel(r'$\dot{q}(t)$')
    plt.title('Fixed Kt identification')

    # Identification with variable Kt ***************************************************
#    y = Ks*current
#    A=np.vstack([np.ones(len(y[maskConstPosVel])),dq[maskConstPosVel], tau[maskConstPosVel]])
#    coef = solveLeastSquare(A.T,y[maskConstPosVel])
#    (Ktp2,Kvp2,Kfp2)=coef[2,0],coef[1,0],coef[0,0]
#    A=np.vstack([np.ones(len(y[maskConstNegVel])),dq[maskConstNegVel], tau[maskConstNegVel]])
#    coef = solveLeastSquare(A.T,y[maskConstNegVel])
#    (Ktn2,Kvn2,Kfn2)=coef[2,0],coef[1,0],-coef[0,0]
#    print 'Ktp2      = ', Ktp2;
#    print 'Kvp2      = ', Kvp2;
#    print 'Kfp2      = ', Kfp2;
#    print 'Ktn2      = ', Ktn2;
#    print 'Kvn2      = ', Kvn2;
#    print 'Kfn2      = ', Kfn2;
#    y = Ks*current-Ktp2*tau
#    y[maskConstPosVel] = Ks*current[maskConstPosVel]-Ktp2*tau[maskConstPosVel]
#    y[maskConstNegVel] = Ks*current[maskConstNegVel]-Ktn2*tau[maskConstNegVel]
#    plt.figure()    
#    plt.axhline(0, color='black',lw=1)
#    plt.axvline(0, color='black',lw=1)
#    plt.plot(x     ,y     ,'.' ,lw=3,markersize=1,c='0.5');  
#    plt.plot(x[maskConstPosVel],y[maskConstPosVel],'rx',lw=3,markersize=1); 
#    plt.plot(x[maskConstNegVel],y[maskConstNegVel],'bx',lw=3,markersize=1); 
#    #plot identified lin model
#    plt.plot([0.0,max(dq)],[ Kfp2,Kvp2*max(dq)+Kfp2],'g-')
#    plt.plot([0.0,min(dq)],[-Kfn2,Kvn2*min(dq)-Kfn2],'g-')
#    plt.ylabel(r'$i(t)-{K_t}{\tau(t)}$')
#    plt.xlabel(r'$\dot{q}(t)$')
#    plt.title('Variable Kt identification')

    # Plot to compare identification with variable/fixed Kt *****************************
#    plt.figure()
#    plt.plot(Ks*current, label='current');
#    plt.plot(tau, '--', label='Ktp*tau')
#    plt.plot(Kvp*dq,   '--', label='Kvp*dq');
#    plt.plot(Ktp*tau+Kvp*dq+Kfp, label='Ktp*tau+Kvp*dq+Kfp');
#    plt.plot(Ktn*tau+Kvn*dq-Kfn, label='Ktn*tau+Kvn*dq-Kfn');
#    plt.plot(Ktp2*tau+Kvp2*dq+Kfp2, label='Ktp2*tau+Kvp2*dq+Kfp2');
#    plt.plot(Ktn2*tau+Kvn2*dq-Kfn2, label='Ktn2*tau+Kvn2*dq-Kfn2');
#    plt.legend();

    plt.show()
    
    return (Kvp, Kvn, Kfp, Kfn, DeadZone, K_bemf)
    