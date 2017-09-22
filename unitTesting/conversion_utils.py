#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  mocap-utils.py
#  
#  Copyright 2017 Thomas Flayols <tflayols@daisen>
import numpy as np
import numpy.matlib
import pinocchio as se3
from pinocchio import Quaternion
from pinocchio.rpy import matrixToRpy
#~ from pinocchio.utils import *
from scipy import signal
from math import cos,sin

def rotationVector_to_Quaternion(rotVec):
    angle = np.linalg.norm(rotVec)
    if (angle > 1e-6):
        ax,ay,az = rotVec/angle
        qx = ax * np.sin(angle/2)
        qy = ay * np.sin(angle/2)
        qz = az * np.sin(angle/2)
        qw = np.cos(angle/2)
        return np.array([qx,qy,qz,qw]);
    return np.array([0.,0.,0.,1.]);
    
def config_sot_to_urdf(q_sot):
    q_sot=np.array(q_sot)
    qUrdf = np.zeros(37)
    qUrdf[:3] = q_sot[:3];      
    quatMat = se3.utils.rpyToMatrix(np.matrix(q_sot[3:6]).T)
    quatVec = Quaternion(quatMat);
    qUrdf[3:7]   = np.array(quatVec.coeffs()).squeeze();
    qUrdf[7:11]  = q_sot[18:22]; # chest-head
    qUrdf[11:18] = q_sot[29:36]; # larm
    qUrdf[18:25] = q_sot[22:29]; # rarm
    qUrdf[25:31] = q_sot[12:18]; # lleg
    qUrdf[31:37]   = q_sot[6:12];# rleg
    return qUrdf
    
def joints_sot_to_urdf(q):
    # GEPETTO VIEWER Free flyer 0-6, CHEST HEAD 7-10, LARM 11-17, RARM 18-24, LLEG 25-30, RLEG 31-36
    # ROBOT VIEWER # Free flyer0-5, RLEG 6-11, LLEG 12-17, CHEST HEAD 18-21, RARM 22-28, LARM 29-35
    qUrdf = np.zeros(30);
    qUrdf[:4]  = q[12:16]; # chest-head
    qUrdf[4:11] = q[23:]; # larm
    qUrdf[11:18] = q[16:23]; # rarm
    qUrdf[18:24] = q[6:12]; # lleg
    qUrdf[24:]   = q[:6]; # rleg
    return qUrdf;

def velocity_sot_to_urdf(v_sot):
    v_urdf = np.zeros(36);
    v_urdf[:6] = v_sot[:6];
    v_urdf[6:] = joints_sot_to_urdf(v_sot[6:]);
    return v_urdf;
    
def homo16toSE3(M):
    r= np.matrix([ [M[0 ],M[1 ],M[2 ]],
                   [M[4 ],M[5 ],M[6 ]],
                   [M[8 ],M[9 ],M[10]] ])           
    t= np.matrix([ [M[3 ]],[M[7 ]],[M[11]]])
    return se3.SE3(r,t)
    
    
def se3ToXYZQuat(M):
    xyzQuat = np.matlib.zeros(7).T;
    xyzQuat[:3, 0] = M.translation;
    xyzQuat[3:, 0] = Quaternion(M.rotation).coeffs();
    return xyzQuat;
    
def se3ToXYZRPY(M):
    rpy = np.squeeze(np.asarray( matrixToRpy(M.rotation)   )).tolist()
    xyz = np.squeeze(np.asarray(             M.translation )).tolist()
    return np.array(xyz+rpy);

def homoToXYZRPY(H):
    r= np.matrix([ [H[0 ],H[1 ],H[2 ]],
                   [H[4 ],H[5 ],H[6 ]],
                   [H[8 ],H[9 ],H[10]] ])           
    t= np.matrix([ [H[3 ]],[H[7 ]],[H[11]]])
    return np.vstack([t,matrixToRpy(r)]).A1

def xyzrpyToSE3(xyzrpy):
    r = se3.utils.rpyToMatrix(np.matrix(xyzrpy[3:6]).T)
    return se3.SE3(r,np.matrix(xyzrpy[:3]))

def se3Interp(s1,s2,alpha):
  t=        ( s1.translation * alpha+
              s2.translation * (1-alpha))
  r=se3.exp3( se3.log3(s1.rotation) * alpha +
              se3.log3(s2.rotation) * (1-alpha))
  return se3.SE3(r,t)

def rpyDerivativeToAngVel(rpy,drpy):
    ''' file:///home/tflayols/devel/simple-base-estimator/StateEstimation-Humanoids17/python/11_Differential%20Kinematics.pdf '''
    p = rpy[1]
    y = rpy[2]
    
    T = np.matrix([ [cos(p)*cos(y), -sin(y) , 0 ] ,
                    [cos(p)*sin(y),  cos(y) , 0 ] ,
                    [-sin(p)    ,  0        , 1 ]]);
    #~ T = np.matrix([ [1., 0., 0 ] ,
                    #~ [0., 1., 0 ] ,
                    #~ [0., 0 , 1 ]]);
                    #~ 

    return (T*np.matrix(drpy).T).A1
    



def filter_mocap_hann(mocapHomo16, ws = 20):
    N = len(mocapHomo16)
    #convert to xyz,rpy
    xyzrpy_tmp = np.empty((N, 6));
    for i in range(N):
        xyzrpy_tmp[i] = se3ToXYZRPY(homo16toSE3(mocapHomo16[i]))
    #filter
    win = signal.hann(ws)
    for i in range(6):
        tmp = signal.convolve(xyzrpy_tmp[:,i], win, mode='same') / sum(win)
        xyzrpy_tmp[ws:-ws,i] = tmp[ws:-ws]#remove side effect 
        
    #convert back
    res = np.empty((N, 16));
    for i in range(N):
        res[i] = np.asarray(xyzrpyToSE3(xyzrpy_tmp[i]).homogeneous.reshape(1,16))[0].tolist()
    return res
    