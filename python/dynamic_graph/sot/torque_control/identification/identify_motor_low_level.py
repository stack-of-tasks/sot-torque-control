# -*- coding: utf-8 -*-
"""
Created on Tue Sep 12 18:47:50 2017

@author: adelpret
"""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.ticker import FormatStrFormatter, LinearLocator

from dynamic_graph.sot.torque_control.hrp2.control_manager_conf import IN_OUT_GAIN


def identify_motor_low_level(dq, ctrl, current):
    DZ = 0.4
    K3 = 1.0  # 1.06
    mask = abs(K3 * ctrl / IN_OUT_GAIN - current) < DZ
    times = np.arange(len(dq)) * 0.001

    plt.figure()
    plt.subplot(411)
    plt.plot(times, current, '.')
    plt.plot(times[mask], current[mask], '.')
    plt.title('current')

    plt.subplot(412)
    plt.plot(times, dq, '.')
    plt.plot(times[mask], dq[mask], '.')
    plt.title('dq')

    plt.subplot(413)
    plt.plot(times, ctrl, '.')
    plt.plot(times[mask], ctrl[mask], '.')
    plt.title(ctrl)
    plt.title('ctrl')

    plt.subplot(414)
    plt.plot(times, K3 * ctrl / IN_OUT_GAIN - current, '.')
    plt.plot(times[mask], K3 * ctrl[mask] / IN_OUT_GAIN - current[mask], '.')
    plt.title(ctrl)
    plt.title('ctrl-current ')

    plt.show()

    # embed()
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    NDZ = 100
    NK3 = 100
    lDZs = np.linspace(0.3, 0.5, NDZ)
    lK3s = np.linspace(0.95, 1.2, NK3)
    DZs, K3s = np.meshgrid(lDZs, lK3s)
    cost = np.zeros([NDZ, NK3])
    for i in range(NDZ):
        #        print( str(int(1000*i/NDZ)/10.0) + '%')
        for j in range(NK3):
            # print( (i,j))
            # if i == 50: embed()
            DZ = lDZs[i]
            K3 = lK3s[j]
            mask = abs(K3 * ctrl / IN_OUT_GAIN - current) < DZ
            #            not_mask = np.logical_not(mask)
            # plt.plot(current[mask],dq[mask],'.')
            cost[i, j] = -np.corrcoef(current[mask],
                                      dq[mask])[0, 1]  # * np.sum(not_mask) / (np.sum(mask) + np.sum(not_mask))
            # embed()
            # cost[i, j] += np.corrcoef(ctrl[not_mask],
            # dq[not_mask])[0, 1] * np.sum(mask) / (np.sum(mask) + np.sum(not_mask))
            # cost[i, j] += np.corrcoef(ctrl[not_mask],
            # current[not_mask])[0, 1] * np.sum(mask) / (np.sum(mask) + np.sum(not_mask))
            # cost[i, j] = cost[i, j] / 3.0

            # ax.scatter(DZ, K3, cost,cmap=cm.coolwarm)
    surf = ax.plot_surface(DZs, K3s, cost.T, cmap=cm.coolwarm, linewidth=0, antialiased=False)

    # Customize the z axis.
    ax.set_zlim(np.min(cost), 1.01)
    ax.zaxis.set_major_locator(LinearLocator(2))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

    # Add a color bar which maps values to colors.
    fig.colorbar(surf, shrink=0.5, aspect=5)
    plt.show()

    # plot a particular case
    DZ = 0.4
    K3 = 1.0
    mask = abs(K3 * ctrl / IN_OUT_GAIN - current) < DZ
    plt.xlabel('current')
    plt.ylabel('dq')
    plt.plot(current[mask], dq[mask], '.')
    plt.show()

    # plot the optimum
    iDZ, iK3 = np.unravel_index(np.argmax(cost), cost.shape)
    DZ = lDZs[iDZ]
    K3 = lK3s[iK3]
    print('DZ = ' + str(DZ))
    print('K3 = ' + str(K3))
    mask = abs(K3 * ctrl / IN_OUT_GAIN - current) < DZ
    plt.xlabel('current')
    plt.ylabel('dq')
    plt.plot(current[mask], dq[mask], '.')
    print(-np.corrcoef(current[mask], dq[mask])[0, 1])
    print(cost[iDZ, iK3])

    plt.show()

    # maskInDZ=abs(ctrl/IN_OUT_GAIN) < 0.5


# tt=np.arange(maskInDZ.size)
# plt.plot(tt,ctrl/IN_OUT_GAIN,'.')
# plt.plot(tt,current,'.')
# plt.plot(tt[maskInDZ],ctrl[maskInDZ]/IN_OUT_GAIN,'.')
# (v1min,v1max) = ( 0.7 , 0.9)
# (v2min,v2max) = (0.25 , 0.3)
# (v3min,v3max) = (0.18 , 0.21)
#
# maskv1= np.logical_and( dq > v1min , dq <v1max  )
# maskv2= np.logical_and( dq > v2min , dq <v2max  )
# maskv3= np.logical_and( dq > v3min , dq <v3max  )
#
# plt.plot(ctrl[maskv1] /IN_OUT_GAIN,current[maskv1],'xr')
# plt.plot(ctrl[maskv2] /IN_OUT_GAIN,current[maskv2],'xg')
# plt.plot(ctrl[maskv3] /IN_OUT_GAIN,current[maskv3],'xy')
# plt.show()
