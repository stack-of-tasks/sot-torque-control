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
from identification_utils import solve1stOrderLeastSquare
from dynamic_graph.sot.torque_control.hrp2.control_manager_conf import IN_OUT_GAIN


def identify_motor_vel(dt, dq, ddq, ctrl, current, tau, Kt_p, Kt_n, ZERO_VELOCITY_THRESHOLD, 
                       ZERO_VELOCITY_THRESHOLD_SMALL, ZERO_ACCELERATION_THRESHOLD, Nvel, SHOW_THRESHOLD_EFFECT):
    # Mask valid data***************************************************
    # remove high acceleration
    maskConstVel = np.logical_and( (abs (ddq)<ZERO_ACCELERATION_THRESHOLD) , (abs (dq)>ZERO_VELOCITY_THRESHOLD))
    # erode to get only steady phases where velocity is constant 
    maskConstVel=ndimage.morphology.binary_erosion(maskConstVel,None,100)
    maskPosVel=(dq> ZERO_VELOCITY_THRESHOLD_SMALL)
    maskNegVel=(dq<-ZERO_VELOCITY_THRESHOLD_SMALL)
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
        plt.plot(ctrl[currentMask] /IN_OUT_GAIN,current[currentMask],'x'+couleurs[it])
        plt.subplot(223)
        plt.xlabel('control - current')
        plt.ylabel('velocity')
        plt.plot(ctrl[currentMask] /IN_OUT_GAIN-current[currentMask],dq[currentMask],'x'+couleurs[it])
        av_dq.append(      np.mean(dq[currentMask]                               ))
        av_delta_i.append( np.mean(ctrl[currentMask] /IN_OUT_GAIN-current[currentMask] ))
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
    Kpwm = 1.0/a;
    
    x=av_delta_i
    plt.plot([-b/a,b/a],[0. ,0.          ],'g:',lw=3)    
    plt.plot([min(x),-b/a],[a*min(x)+b ,0.          ],'g:',lw=3)    
    plt.plot([b/a,max(x)],[0.,a*max(x)-b],'g:',lw=3)
    plt.show()
    #~ y        = a. x +  b
    #~ i-Kt.tau = Kv.dq + Kf
    #~ 
    # Identification ***************************************************
    y = current-Kt_p*tau
    y[maskConstPosVel] = current[maskConstPosVel]-Kt_p*tau[maskConstPosVel]
    y[maskConstNegVel] = current[maskConstNegVel]-Kt_n*tau[maskConstNegVel]
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
    plt.show()
    
    return (Kvp, Kvn, Kfp, Kfn, DeadZone, Kpwm)
    