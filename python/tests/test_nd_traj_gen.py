# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import random
import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.torque_control.nd_trajectory_generator import NdTrajectoryGenerator
from time import sleep
from plot_utils import create_empty_figure
import plot_utils
import matplotlib.pyplot as plt    

def main(dt):
    np.set_printoptions(precision=2, suppress=True);
    jid = 0;
    T = 2.0;
    x_0 = (1.0, 2.0, 3.0);
    x_f = (2.0, -3.0, 4.0);

    traj_gen = NdTrajectoryGenerator('traj_gen');
    traj_gen.init(dt, 3);
    print "Gonna start sinusoid from %.1f to %.1f" % (x_0[jid], x_f[jid]);
    traj_gen.initial_value.value = x_0;
    traj_gen.x.recompute(0);
    traj_gen.startSinusoid(jid, x_f[jid], T);
    N = int(T/dt);
    xSin = np.zeros(2*N);
    dxSin = np.zeros(2*N);
    ddxSin = np.zeros(2*N);
    for i in range(2*N):
        traj_gen.x.recompute(i);
        traj_gen.dx.recompute(i);
        traj_gen.ddx.recompute(i);
        xSin[i] = traj_gen.x.value[jid];
        dxSin[i] = traj_gen.dx.value[jid];
        ddxSin[i] = traj_gen.ddx.value[jid];
        if(i%10==0):
            print "x(%.3f) = %.3f, \tdx = %.3f" % (i*dt, xSin[i], dxSin[i]);
    traj_gen.stop(jid);
    dx_fd = np.diff(xSin)/dt;
    ddx_fd = np.diff(dxSin)/dt;
    time = np.arange(0.0, 2*T, dt);
    print time.shape, xSin.shape
    (fig,ax) = create_empty_figure(3,1);
    ax[0].plot(time, xSin,'r');
    ax[1].plot(time, dxSin,'r'); ax[1].plot(time[:-1], dx_fd,'g--');
    ax[2].plot(time, ddxSin,'r'); ax[2].plot(time[:-1], ddx_fd,'g--');
    plt.show();
    return (traj_gen);
    
if __name__=='__main__':
    (traj_gen) = main(0.01);
    pass;
    