# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 13:47:33 2015

@author: adelpret
"""
import numpy as np
from numpy.linalg import lstsq
import matplotlib.pyplot as plt
from math import floor


def estimate(t, x):
    ws = x.shape[0]
    A = np.zeros((ws, 2))
    A[:, 0] = t
    A[:, 1] = 1
    (v, residual, rank, sv) = lstsq(A, x)
    return v


def estimateVelocity(time, x, ws=61, doPlot=False):
    n = x.shape[0]
    ws2 = int(floor(0.5 * (ws - 1)))
    # half-window length
    dx = np.zeros(x.shape)

    for i in range(ws2):
        x_w = x[:i + ws2, :]
        t_w = time[:i + ws2]
        v = estimate(t_w, x_w)
        dx[i, :] = v[0, :]

    for i in range(ws2, n - ws2):
        x_w = x[i - ws2:i + ws2, :]
        t_w = time[i - ws2:i + ws2]
        v = estimate(t_w, x_w)
        dx[i, :] = v[0, :]

    for i in range(n - ws2, n):
        x_w = x[i - ws2:, :]
        t_w = time[i - ws2:]
        v = estimate(t_w, x_w)
        dx[i, :] = v[0, :]

    if (doPlot):
        for j in range(x.shape[1]):
            fig = plt.figure()
            ax = fig.add_subplot(211)
            ax.plot(x[:, j])
            ax.set_title('pos')
            ax = fig.add_subplot(212)
            ax.plot(dx[:, j])
            ax.set_title('vel')
        plt.show()

    return dx
