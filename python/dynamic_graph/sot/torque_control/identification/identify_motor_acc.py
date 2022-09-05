# -*- coding: utf-8 -*-
"""
Created on Tue Sep 12 18:47:50 2017

@author: adelpret
"""
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage, signal

from identification_utils import solve1stOrderLeastSquare


def identify_motor_acc(
    dt,
    dq,
    ddq,
    current,
    tau,
    Kt_p,
    Kv_p,
    ZERO_VELOCITY_THRESHOLD_SMALL,
    ZERO_JERK_THRESHOLD,
    SHOW_THRESHOLD_EFFECT,
):
    # Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode="same") / sum(win)
    current = filtered_current

    # Mask valid data***************************************************
    # # remove high jerk
    dddq = np.gradient(ddq, 1) / dt
    maskConstAcc = abs(dddq) < ZERO_JERK_THRESHOLD
    # # erode to get only steady phases where acceleration is constant
    maskConstAcc = ndimage.morphology.binary_erosion(maskConstAcc, None, 100)
    maskPosVel = dq > ZERO_VELOCITY_THRESHOLD_SMALL
    maskNegVel = dq < -ZERO_VELOCITY_THRESHOLD_SMALL
    maskConstPosAcc = np.logical_and(maskConstAcc, maskPosVel)
    maskConstNegAcc = np.logical_and(maskConstAcc, maskNegVel)

    if SHOW_THRESHOLD_EFFECT:
        plt.figure()
        plt.plot(ddq)
        plt.ylabel("ddq")
        ddq_const = ddq.copy()
        ddq_const[np.logical_not(maskConstAcc)] = np.nan
        plt.plot(ddq_const)
        plt.ylabel("ddq_const")
        plt.show()

    # y              = a. x   +  b
    # i-Kt.tau-Kv.dq = Ka.ddq +  Kf
    #
    # Identification ***************************************************
    y = current - Kt_p * tau - Kv_p * dq
    y[maskConstPosAcc] = current[maskConstPosAcc] - Kt_p * tau[maskConstPosAcc] - Kv_p * dq[maskConstPosAcc]
    y[maskConstNegAcc] = current[maskConstNegAcc] - Kt_p * tau[maskConstNegAcc] - Kv_p * dq[maskConstNegAcc]
    y_label = r"$i(t)-{K_t}{\tau(t)}-{K_v}{\dot{q}(t)}$"
    x = ddq
    x_label = r"$\ddot{q}(t)$"
    (Kap, Kfp) = solve1stOrderLeastSquare(x[maskConstPosAcc], y[maskConstPosAcc])
    (Kan, b) = solve1stOrderLeastSquare(x[maskConstNegAcc], y[maskConstNegAcc])
    Kfn = -b

    # Plot *************************************************************
    plt.figure()
    plt.axhline(0, color="black", lw=1)
    plt.axvline(0, color="black", lw=1)
    plt.plot(x, y, ".", lw=3, markersize=1, c="0.5")
    plt.plot(x[maskConstPosAcc], y[maskConstPosAcc], "rx", lw=3, markersize=1)
    plt.plot(x[maskConstNegAcc], y[maskConstNegAcc], "bx", lw=3, markersize=1)

    # plot identified lin model
    plt.plot([min(x), max(x)], [Kap * min(x) + Kfp, Kap * max(x) + Kfp], "g:", lw=3)
    plt.plot([min(x), max(x)], [Kan * min(x) - Kfn, Kan * max(x) - Kfn], "g:", lw=3)
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    plt.show()

    return (Kap, Kan, Kfp, Kfn)
