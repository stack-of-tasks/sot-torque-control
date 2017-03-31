# -*- coding: utf-8 -*-
"""
Created on Wed Jan 28 16:54:01 2015

Test the expressions for the acceleration limits that guarantees the feasibility
of the position and velocity limits in the feature.
These expression have been derived using the viability theory.
@author: adelpret
"""
import numpy as np
from numpy.random import random
from plot_utils import create_empty_figure
import plot_utils
import matplotlib.pyplot as plt


plot_utils.DEFAULT_LINE_WIDTH = 5;

N = 15000;
f0 = 0.3;
f1 = 3;
tt = 15.0;
dt = 0.001;

phi_0 = np.pi*tt*(f0-f1);
t = 0;
x = np.zeros(N);
f = np.zeros(N);
phi = np.zeros(N);
k = 2*(f1-f0)/tt;
for i in range(N):
    if(t<0.5*tt):
        f[i] = f0 + k*t;
        phi[i] = 2*np.pi*t*(f0+0.5*k*t);
    else:
        f[i] = f1 + 0.5*k*tt - k*t;
        phi[i] = phi_0 + 2*np.pi*t*(f1+0.5*k*tt - 0.5*k*t);
    x[i] = 0.5*(1.0-np.cos(phi[i]));
    t = t + dt;

(fig,ax) = create_empty_figure(3,1);
ax[0].plot(x);
ax[1].plot(f);
ax[2].plot(phi);
plt.show();
