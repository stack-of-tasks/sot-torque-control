# -*- coding: utf-8 -*-
"""
Created on Tue Sep 12 18:47:50 2017

@author: adelpret
"""

from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np
from dynamic_graph.sot.torque_control.hrp2.control_manager_conf import IN_OUT_GAIN
from scipy import ndimage

from identification_utils import solve1stOrderLeastSquare


def identify_motor_static(enc, dq, ctrl, current, tau, JOINT_ID, JOINT_NAME, ZERO_VELOCITY_THRESHOLD,
                          ZERO_VELOCITY_THRESHOLD_SMALL, SHOW_THRESHOLD_EFFECT):
    # remove high velocity
    maskConstAng = (abs(dq) < ZERO_VELOCITY_THRESHOLD)
    # erode to get only steady phases where velocity is small
    maskConstAng = ndimage.morphology.binary_erosion(maskConstAng, None, 100)
    maskPosVel = (dq > ZERO_VELOCITY_THRESHOLD_SMALL)
    maskNegVel = (dq < -ZERO_VELOCITY_THRESHOLD_SMALL)
    maskConstPosAng = np.logical_and(maskConstAng, maskPosVel)
    maskConstNegAng = np.logical_and(maskConstAng, maskNegVel)
    if SHOW_THRESHOLD_EFFECT:
        plt.figure()
        plt.plot(enc, label='q')
        q_const = enc.copy()
        q_const[np.logical_not(maskConstAng)] = np.nan
        plt.plot(q_const, label='q_const')
        plt.legend()

    # identify current sensor gain
    x = current[maskConstAng]
    y = ctrl[maskConstAng] / IN_OUT_GAIN
    maskPosErr = np.logical_and(y - x > 0.0, np.abs(x) > 0.5)
    maskNegErr = np.logical_and(y - x < 0.0, np.abs(x) > 0.5)
    print("Number of samples with constant angle:", x.shape[0])
    print("Number of samples with constant angle and pos vel:", x[maskPosErr].shape[0])
    print("Number of samples with constant angle and neg vel:", x[maskNegErr].shape[0])
    if (x[maskPosErr].shape[0] < 10):
        (Ks, DZ) = solve1stOrderLeastSquare(x[maskNegErr], y[maskNegErr])
    elif (x[maskNegErr].shape[0] < 10):
        (Ks, DZ) = solve1stOrderLeastSquare(x[maskPosErr], y[maskPosErr])
    else:
        (Ksn, DZn) = solve1stOrderLeastSquare(x[maskNegErr], y[maskNegErr])
        (Ksp, DZp) = solve1stOrderLeastSquare(x[maskPosErr], y[maskPosErr])
        Ks = 0.5 * (Ksp + Ksn)
        Ks = min([Ksp, Ksn])
        DZ = 0.5 * (DZp - DZn)
        print("Current sensor gains = ", Ksp, Ksn)
        print("Deadzones            = ", DZp, -DZn)

        x_neg = x[maskNegErr]
        y_neg = y[maskNegErr]
        plt.figure()
        plt.plot(x_neg, y_neg, '.', lw=3, markersize=1, c='0.5')
        plt.plot([min(x_neg), max(x_neg)], [Ksn * min(x_neg) + DZn, Ksn * max(x_neg) + DZn], 'g:', lw=3)
        plt.ylabel(r'$i(t)$')
        plt.xlabel(r'$u(t)$')
        plt.title('Negative current errors - Joint ' + JOINT_NAME)

        x_pos = x[maskPosErr]
        y_pos = y[maskPosErr]
        plt.figure()
        plt.plot(x_pos, y_pos, '.', lw=3, markersize=1, c='0.5')
        plt.plot([min(x_pos), max(x_pos)], [Ksp * min(x_pos) + DZp, Ksp * max(x_pos) + DZp], 'g:', lw=3)
        plt.ylabel(r'$i(t)$')
        plt.xlabel(r'$u(t)$')
        plt.title('Positive current errors - Joint ' + JOINT_NAME)
        plt.show()

    if (Ks < 0.0):
        print("ERROR: estimated Ks is negative! Setting it to 1")
        Ks = 1.0

    # plot dead zone effect ********************************************
    plt.figure()
    plt.plot(Ks * current, label='current')
    plt.plot(ctrl / IN_OUT_GAIN, label='control')
    plt.legend()

    plt.figure()
    y = Ks * current[maskConstAng]
    x = ctrl[maskConstAng] / IN_OUT_GAIN - Ks * current[maskConstAng]
    plt.ylabel(r'$i(t)$')
    plt.xlabel(r'$ctrl(t)-i(t)$')
    plt.plot(x, y, '.', lw=3, markersize=1, c='0.5')
    plt.plot(x[maskPosErr], y[maskPosErr], 'rx', lw=3, markersize=1, label='pos err')
    plt.plot(x[maskNegErr], y[maskNegErr], 'bx', lw=3, markersize=1, label='neg err')
    plt.legend()

    plt.figure()
    y = ctrl[maskConstAng] / IN_OUT_GAIN
    x = ctrl[maskConstAng] / IN_OUT_GAIN - Ks * current[maskConstAng]
    plt.ylabel(r'$ctrl(t)$')
    plt.xlabel(r'$ctrl(t)-i(t)$')
    plt.plot(x, y, '.', lw=3, markersize=1, c='0.5')
    plt.plot(x[maskPosErr], y[maskPosErr], 'rx', lw=3, markersize=1, label='pos err')
    plt.plot(x[maskNegErr], y[maskNegErr], 'bx', lw=3, markersize=1, label='neg err')
    plt.legend()

    plt.figure()
    y = ctrl / IN_OUT_GAIN
    x = Ks * current
    plt.ylabel(r'$ctrl(t)$')
    plt.xlabel(r'$i(t)$')
    plt.plot(x, y, '.', lw=3, markersize=1, c='0.5')
    plt.plot([-3, 3], [-3, 3])

    plt.show()
    #    y = a. x   +  b
    #    i = Kt.tau + Kf

    # Identification ***************************************************
    y = current  # *Ks
    x = tau
    (Ktp, Kfp) = solve1stOrderLeastSquare(x[maskConstPosAng], y[maskConstPosAng])
    (Ktn, b) = solve1stOrderLeastSquare(x[maskConstNegAng], y[maskConstNegAng])
    Kfn = -b

    # Plot *************************************************************
    plt.figure()
    plt.axhline(0, color='black', lw=1)
    plt.axvline(0, color='black', lw=1)
    plt.plot(x, y, '.', lw=3, markersize=1, c='0.5')
    plt.plot(x[maskConstPosAng], y[maskConstPosAng], 'rx', lw=3, markersize=1)
    plt.plot(x[maskConstNegAng], y[maskConstNegAng], 'bx', lw=3, markersize=1)
    # plot identified lin model
    plt.plot([min(x), max(x)], [Ktp * min(x) + Kfp, Ktp * max(x) + Kfp], 'g:', lw=3)
    plt.plot([min(x), max(x)], [Ktn * min(x) - Kfn, Ktn * max(x) - Kfn], 'g:', lw=3)
    plt.ylabel(r'$i(t)$')
    plt.xlabel(r'$\tau(t)$')
    plt.title('Static experiment - Joint ' + JOINT_NAME)

    print("cur_sens_gain[%d] = %f" % (JOINT_ID, Ks))
    print('deadzone[%d]      = %f' % (JOINT_ID, DZ))
    print('Kt_p[%d]          = %f' % (JOINT_ID, Ktp))
    print('Kt_n[%d]          = %f' % (JOINT_ID, Ktn))
    print('Kf_p[%d]          = %f' % (JOINT_ID, Kfp))
    print('Kf_n[%d]          = %f' % (JOINT_ID, Kfn))

    print('Kt_m[%d]          = %f' % (JOINT_ID, (Ktp + Ktn) / 2.0))
    print('Kf_m[%d]          = %f' % (JOINT_ID, (Kfp + Kfn) / 2.0))

    return (Ktp, Ktn, Ks, DZ)
