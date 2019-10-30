# -*- coding: utf-8 -*-
"""
Created on Mon Feb 23 09:02:21 2015

@author: adelpret
q.shape"""

import sys

import matplotlib as mpl
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import scipy
import scipy.fftpack
from scipy.signal import blackman

import hrp2_motors_parameters_pwl as pwl
import my_ltisys
import plot_utils
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from qpoases import PyQProblemB as QProblemB  # QP with simple bounds only
from qpoases import PySQProblem as SQProblem


def update_line(num, data, line):
    line.set_data(data[..., :(50 * num)])
    return line,


''' Solve the least square problem:
    minimize   || A*x-b ||^2
    subject to lb_in <= A_in*x <= ub_in
                lb <= x <= ub
'''


def solveLeastSquare(A, b, lb=None, ub=None, A_in=None, lb_in=None, ub_in=None):
    n = A.shape[1]
    m_in = 0
    if (A_in is not None):
        m_in = A_in.shape[0]
        if (lb_in is None):
            lb_in = np.array(m_in * [-1e99])
        if (ub_in is None):
            ub_in = np.array(m_in * [1e99])

    if (lb is None):
        lb = np.array(n * [-1e99])
    if (ub is None):
        ub = np.array(n * [1e99])

    Hess = np.dot(A.transpose(), A)
    grad = -np.dot(A.transpose(), b)
    maxActiveSetIter = np.array([100 + 2 * m_in + 2 * n])
    maxComputationTime = np.array([600.0])
    options = Options()
    options.printLevel = PrintLevel.LOW
    # NONE, LOW, MEDIUM
    options.enableRegularisation = True
    print('Gonna solve QP...')
    if (m_in == 0):
        qpOasesSolver = QProblemB(n)
        # , HessianType.SEMIDEF);
        qpOasesSolver.setOptions(options)
        imode = qpOasesSolver.init(Hess, grad, lb, ub, maxActiveSetIter, maxComputationTime)
    else:
        qpOasesSolver = SQProblem(n, m_in)
        # , HessianType.SEMIDEF);
        qpOasesSolver.setOptions(options)
        imode = qpOasesSolver.init(Hess, grad, A_in, lb, ub, lb_in, ub_in, maxActiveSetIter, maxComputationTime)

    # print('QP solved in %f seconds and %d iterations' % (maxComputationTime[0],maxActiveSetIter[0]);)
    if (imode != 0 and imode != 63):
        print("ERROR Qp oases %d " % (imode))
    x_norm = np.zeros(n)
    # solution of the normalized problem
    qpOasesSolver.getPrimalSolution(x_norm)
    return x_norm


FOLDER_ID = 1
DT = 0.001
FIRST_SAMPLE = 0
LAST_SAMPLE = -1
PLOT_PIECE_WISE_LINEAR = True
PLOT_TRANSFER_FUNCTION = False
PLOT_ANIMATED_DATA = True

if (FOLDER_ID == 1):
    data_folder = '../results/20150401_152647_id_rhp_chirp_force_0.1_to_5_Hz_in_55_sec/'
    JOINT_ID = 2
    # right hip pitch chirp with zero vel
    FIRST_SAMPLE = 13 * 1000
    LAST_SAMPLE = -9 * 1000
else:
    print('ERROR: UNKNOWN FOLDER_ID')

DATA_FILE_NAME = 'data.npz'

plot_utils.FIGURE_PATH = data_folder
plot_utils.SAVE_FIGURES = False
plot_utils.SHOW_FIGURES = True
plot_utils.SHOW_LEGENDS = True
plot_utils.LINE_ALPHA = 0.7
''' Load data from file '''
try:
    data = np.load(data_folder + DATA_FILE_NAME)
    enc = np.squeeze(data['enc'])
    tau = np.squeeze(data['tau'])
    qDes = np.squeeze(data['qDes'])
#    q = data['q'];
#    dq = np.squeeze(data['dq']);
except IOError:
    print("Impossible to read data file %f" % (data_folder + DATA_FILE_NAME))
    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.")

delta_q = qDes - enc

tau = tau[FIRST_SAMPLE:LAST_SAMPLE]
delta_q = delta_q[FIRST_SAMPLE:LAST_SAMPLE]
m = len(tau) - 1

tau_max = np.max(tau)
delta_q_max = np.max(delta_q)

tau_next = tau[1:]
tau = tau[:-1]
delta_q = delta_q[:-1]
time = np.arange(0, m * DT, DT)

if (PLOT_ANIMATED_DATA):
    fig1 = plt.figure()
    d = np.empty((2, m))
    d[0, :] = tau
    d[1, :] = delta_q
    l, = plt.plot([], [], 'b x')
    plt.xlim(np.min(tau), tau_max)
    plt.ylim(np.min(delta_q), delta_q_max)
    plt.xlabel('tau')
    plt.ylabel('delta_q')
    line_ani = animation.FuncAnimation(fig1, update_line, m, fargs=(d, l), interval=50, blit=True, repeat=False)
    plt.show()

plt.figure()
plt.plot(tau, delta_q, 'x ')
plt.title('Tau VS delta_q')
plt.show()

if (PLOT_PIECE_WISE_LINEAR):
    MIN_MARG = 3
    tau_min = np.min([np.min(tau), pwl.f_tau1p[JOINT_ID] - MIN_MARG, pwl.f_tau1n[JOINT_ID] - MIN_MARG])
    tau_max = np.max([tau_max, pwl.f_tau2p[JOINT_ID] + MIN_MARG, pwl.f_tau2n[JOINT_ID] + MIN_MARG])
    tau_model_pos = np.array([tau_min, pwl.f_tau1p[JOINT_ID], pwl.f_tau2p[JOINT_ID], tau_max])
    tau_model_neg = np.array([tau_min, pwl.f_tau1n[JOINT_ID], pwl.f_tau2n[JOINT_ID], tau_max])
    delta_q_model_pos = np.array([
        pwl.f_k1p[JOINT_ID] * tau_min + pwl.f_q1p[JOINT_ID],
        pwl.f_k1p[JOINT_ID] * pwl.f_tau1p[JOINT_ID] + pwl.f_q1p[JOINT_ID],
        pwl.f_k2p[JOINT_ID] * pwl.f_tau2p[JOINT_ID] + pwl.f_q2p[JOINT_ID],
        pwl.f_k3p[JOINT_ID] * tau_max + pwl.f_q3p[JOINT_ID]
    ])
    delta_q_model_neg = np.array([
        pwl.f_k1n[JOINT_ID] * tau_min + pwl.f_q1n[JOINT_ID],
        pwl.f_k1n[JOINT_ID] * pwl.f_tau1n[JOINT_ID] + pwl.f_q1n[JOINT_ID],
        pwl.f_k2n[JOINT_ID] * pwl.f_tau2n[JOINT_ID] + pwl.f_q2n[JOINT_ID],
        pwl.f_k3n[JOINT_ID] * tau_max + pwl.f_q3n[JOINT_ID]
    ])

    f, ax = plt.subplots(1, 1, sharex=True)
    ax.plot(tau, delta_q, 'r .')
    ax.plot(tau_model_pos, delta_q_model_pos, 'b--')
    ax.plot(tau_model_neg, delta_q_model_neg, 'g--')
    ax.set_xlabel('Torque [Nm]')
    ax.set_ylabel('Delta_q [rad]')
    title = 'Torque vs delta_q  piece-wise linear model'
    # saveCurrentFigure(title);
    ax.set_title(title)
    plt.show()

print('   Perform identification with 1-st order dynamics: tau[n+1] = a*tau[n] + b*delta_q[n]')
A_norm = np.zeros((m, 2))
A_norm[:, 0] = tau / tau_max
A_norm[:, 1] = delta_q / delta_q_max
b_norm = tau_next / tau_max
x_norm = solveLeastSquare(A_norm, b_norm)
a = x_norm[0]
b = x_norm[1] * tau_max / delta_q_max
# Compute RC costant and cut-off frequency
RC = DT / (1 - a) - DT
f_c = 1 / (2 * np.pi * RC)
max_kp = a / (1 - a)
print('a[%d] = %f\tb[%d] = %f' % (JOINT_ID, a, JOINT_ID, b))
print('Cut-off frequency = %f' % f_c)
print('Max torque feedback proportional gain = %f' % max_kp)
print('k_tau = %f' % ((1 - a) / b))

print('\n   Perform identification with 1-st order dynamics forcing the previously-identified static relationship: ' +
      'delta_q = k_tau*tau')
k_tau = 0.0005
A_norm = np.zeros((m, 1))
A_norm[:, 0] = tau - delta_q / k_tau
b_norm = tau_next - delta_q / k_tau
x_norm = solveLeastSquare(A_norm, b_norm)
a2 = x_norm[0]
b2 = (1 - a2) / k_tau
# Compute RC costant and cut-off frequency
RC2 = DT / (1 - a2) - DT
f_c2 = 1 / (2 * np.pi * RC2)
max_kp2 = a2 / (1 - a2)
print('a[%d] = %f\tb[%d] = %f' % (JOINT_ID, a2, JOINT_ID, b2))
print('Cut-off frequency = %f' % f_c2)
print('Max torque feedback proportional gain = %f' % max_kp2)

print('   Perform identification with 2-nd order dynamics: tau[n+1] = a*tau[n] + b*delta_q[n]')
tau_next = tau[2:]
tau_curr = tau[1:-1]
tau_prev = tau[:-2]
delta_q = delta_q[:-2]
A_norm = np.zeros((m - 2, 3))
A_norm[:, 0] = tau_curr / tau_max
A_norm[:, 1] = tau_prev / tau_max
A_norm[:, 2] = delta_q / delta_q_max
b_norm = tau_next / tau_max
x_norm = solveLeastSquare(A_norm, b_norm)
a3 = x_norm[0]
b3 = x_norm[1]
c3 = x_norm[2] * tau_max / delta_q_max

tau = np.squeeze(data['tau'])
tau = tau[FIRST_SAMPLE:LAST_SAMPLE]
tau_next = tau[1:]
tau = tau[:-1]

tau_model = np.copy(tau_next)
tau_model2 = np.copy(tau_next)
tau_model3 = np.copy(tau_next)
for i in range(1, m - 2):
    if (i % 100000 == 0):
        tau_model[i + 1] = a * tau[i] + b * delta_q[i]
        tau_model2[i + 1] = a2 * tau[i] + b2 * delta_q[i]
        tau_model3[i + 1] = a3 * tau[i] + b3 * tau[i - 1] + c3 * delta_q[i]
    else:
        tau_model[i + 1] = a * tau_model[i] + b * delta_q[i]
        tau_model2[i + 1] = a2 * tau_model2[i] + b2 * delta_q[i]
        tau_model3[i + 1] = a3 * tau_model3[i] + b3 * tau_model3[i - 1] + c3 * delta_q[i]

print('\n   Compute FFT of signals')
# time = scipy.linspace(0,120,4000);
# acc = lambda t: 10*scipy.sin(2*np.pi*2.0*t) + 5*scipy.sin(2*np.pi*8.0*t) + 2*scipy.random.random(len(t));
# signal = acc(t);
w = blackman(len(tau))
FFT = abs(scipy.fftpack.fft(tau))
FFT_model = abs(scipy.fftpack.fft(tau_model3))
FFT_wf = abs(scipy.fftpack.fft(tau * w))
FFT_model_w = abs(scipy.fftpack.fft(tau_model3 * w))
freqs = scipy.fftpack.fftfreq(tau.size, DT)
w = 2 * np.pi * freqs

if (PLOT_TRANSFER_FUNCTION):
    print('Compute frequency responce of identified system')
    # sys1 = my_ltisys.lti([a], [b], [1], [0]);
    # sys1 = scipy.signal.lti([b], [1, -a]);
    sys1 = scipy.signal.lti(1, [1, 1.0 / RC])
    w, mag, phase = my_ltisys.bode(sys1, w)
    f, ax = plt.subplots(2, 1, sharex=False)
    ax[0].semilogx(w, mag)
    # Bode magnitude plot
    ax[1].semilogx(w, phase)
    # Bode phase plot
    # plt.show();

    mpl.rcParams['lines.linewidth'] = 3
    f, ax = plt.subplots(1, 1, sharex=False)
    # ax[0].plot(time, tau);
    ax.semilogx(freqs, mag)
    # Bode magnitude plot
    ax.semilogx(freqs, 20.0 * np.log10(FFT), 'r-')
    # ax.semilogx(freqs,20.0 * np.log10(FFT_wf),'r--');
    ax.semilogx(freqs, 20.0 * np.log10(FFT_model), 'b-')
    # 20*scipy.log10
    # ax.semilogx(freqs,20.0 * np.log10(FFT_model_w),'b--'); #20*scipy.log10
    # ax[1].set_yscale('log');

f, ax = plt.subplots(1, 1, sharex=True)
ax.plot(time, tau_next, 'r .')
ax.plot(time, tau_model, 'b--')
ax.plot(time, tau_model3, 'c--')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Tau [Nm]')
leg = plt.legend(['Data', 'Model1', 'Model3'], loc='upper left')
leg.get_frame().set_alpha(plot_utils.LEGEND_ALPHA)
# saveCurrentFigure('Real torque VS predicted torque');
plt.show()
