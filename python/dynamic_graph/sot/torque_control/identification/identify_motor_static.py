# -*- coding: utf-8 -*-
"""
Created on Tue Sep 12 18:47:50 2017

@author: adelpret
"""
from scipy import signal
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt
from identification_utils import solve1stOrderLeastSquare
from dynamic_graph.sot.torque_control.hrp2.control_manager_conf import IN_OUT_GAIN


def identify_motor_static(enc, dq, ctrl, current, tau, JOINT_ID, JOINT_NAME, ZERO_VELOCITY_THRESHOLD, SHOW_THRESHOLD_EFFECT):
    # remove high velocity
    maskConstAng = (abs (dq)<ZERO_VELOCITY_THRESHOLD)
    # erode to get only steady phases where velocity is small 
    maskConstAng=ndimage.morphology.binary_erosion(maskConstAng,None,100)
    #~ plt.figure()
    #~ plt.plot(ddq);
    maskPosVel=(dq> 0.001)
    maskNegVel=(dq<-0.001)
    maskConstPosAng=np.logical_and( maskConstAng ,maskPosVel )
    maskConstNegAng=np.logical_and( maskConstAng ,maskNegVel ) 
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(enc); plt.ylabel('q')
        q_const=enc.copy()
        q_const[np.logical_not(maskConstAng)]=np.nan
        plt.plot(q_const); plt.ylabel('q_const')
        
    # plot dead zone effect ********************************************
#    IN_OUT_GAIN = 90.0;
    plt.figure()
    plt.plot(current, label='current')
    plt.plot(ctrl/IN_OUT_GAIN, label='control')
    plt.legend()
    
    plt.figure()
    y = current
    x = ctrl/IN_OUT_GAIN - current
    plt.ylabel(r'$i(t)$')
    plt.xlabel(r'$ctrl(t)-i(t)$')
    plt.plot(x,y,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosAng],y[maskConstPosAng],'rx',lw=3,markersize=1, label='pos vel'); 
    plt.plot(x[maskConstNegAng],y[maskConstNegAng],'bx',lw=3,markersize=1, label='neg vel'); 
    plt.legend()
    
    plt.figure()
    y = ctrl/IN_OUT_GAIN
    x = ctrl/IN_OUT_GAIN - current
    plt.ylabel(r'$ctrl(t)$')
    plt.xlabel(r'$ctrl(t)-i(t)$')    
    plt.plot(x,y,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosAng],y[maskConstPosAng],'rx',lw=3,markersize=1, label='pos vel'); 
    plt.plot(x[maskConstNegAng],y[maskConstNegAng],'bx',lw=3,markersize=1, label='neg vel'); 
    plt.legend()
    
    plt.figure()
    y = ctrl/IN_OUT_GAIN
    x = current
    plt.ylabel(r'$ctrl(t)$')
    plt.xlabel(r'$i(t)$')    
    plt.plot(x,y,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot([-3,3],[-3,3]);  
    
    plt.show()
#    y = a. x   +  b
#    i = Kt.tau + Kf
    
    # Identification ***************************************************
    y = current
    x = tau
    (Ktp,Kfp)=solve1stOrderLeastSquare(x[maskConstPosAng],y[maskConstPosAng])
    (Ktn,b)=solve1stOrderLeastSquare(x[maskConstNegAng],y[maskConstNegAng])
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
    plt.ylabel(r'$i(t)$')
    plt.xlabel(r'$\tau(t)$')
    plt.title('Static experiment - Joint '+JOINT_NAME)
    print 'Kt_p[%d] = %f' % (JOINT_ID,Ktp);
    print 'Kt_n[%d] = %f' % (JOINT_ID,Ktn);
    print 'Kf_p[%d] = %f' % (JOINT_ID,Kfp);
    print 'Kf_n[%d] = %f' % (JOINT_ID,Kfn);
    
    print 'Kt_m[%d] = %f' % (JOINT_ID,(Ktp+Ktn)/2.0);
    print 'Kf_m[%d] = %f' % (JOINT_ID,(Kfp+Kfn)/2.0);
    
    return (Ktp, Ktn);