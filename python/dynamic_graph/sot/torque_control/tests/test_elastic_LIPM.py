import numpy as np
from scipy.linalg import expm, eigvals
import matplotlib.pyplot as plt
from numpy.random import normal
from math import sin, pi, cos, sqrt
try:
    from dynamic_graph.sot.torque_control.utils.plot_utils import *
except:
    print "Failed to load plot-utils"

class Robot:
    def __init__(self, omega, m1, mL, mR, C, K, B, g, f_stdev, x1_0, dx1_0, xL_0, dxL_0, xR_0, dxR_0):
        self.omega = omegal
        self.m1 = m1;
        self.mL = mL;
        self.mR = mR;
#        self.B1 = B1;
        self.C = C;
        self.K = K;
        self.B = B;
        self.g = g;
        self.f_stdev = f_stdev;
        self.x1 = x1;
        self.dx1 = dx1;
        self.xL = xL;
        self.dxL = dxL;
        self.xR = xR;
        self.dxR = dxR;
        
    def simulate(self, f1, f2, T, dt=0.0001):
        N = int(T/dt);
        assert(N>0)
        f1_noise = normal(0, self.f_stdev);
        f2_noise = normal(0, self.f_stdev);
        f1_sum = 0;
        f2_sum = 0;
        for i in range(N):        
            # compute forces
            self.fL = -self.K*self.xL - self.B*self.dxL;
            self.fR = -self.K*self.xR - self.B*self.dxR;
            self.f1 = f1 + f1_noise;
#            self.f1 -= self.B1*(self.dx1-self.dx2) + self.C1*np.sign(self.dx1-self.dx2)
            self.f2 = f2 + f2_noise;
            f1_sum += self.f1;
            f2_sum += self.f2;
            # compute accelerations
            self.ddx1 = self.g + (self.f1 )/self.m1;
            self.ddx2 = self.g + (-self.f1 + self.f2)/self.m2;
            # integrate
            self.x1 += dt*self.dx1 + 0.5*dt*dt*self.ddx1;
            self.x2 += dt*self.dx2 + 0.5*dt*dt*self.ddx2;
            self.dx1 += dt*self.ddx1;
            self.dx2 += dt*self.ddx2;
        self.f1 = f1_sum/N;
        self.f2 = f2_sum/N;
        

# *** SYSTEM DESCRIPTION ***
# q1 is the X position of m1 w.r.t. the world
# qL is the Z position of the left  foot w.r.t. the world
# qR is the Z position of the right foot w.r.t. the world
# we want to control q1

# CoP = (fL*xL + fR*xR)/(fL+fR)
# ddq1 = omega*(q1-CoP)
# mL*(ddqL-g) = fL - f1
# mR*(ddqR-g) = fR - f2
# fL = -K*qL - B*dqL
# fR = -K*qR - B*dqR
# f1 = f1d - B1*(dq1-dq2)

omega = 3;
m1 = 60.            # body mass
mL = 1.             # left foot mass
mR = 1.             # right foot mass
M = m1+mL+mR;       # total mass
xL = -0.1;          # left foot position
xR =  0.1;          # right foot position
#B = .5*m1           # motor viscous friction
C = 0.5*B;          # motor Coulomb friction
K = 2e5             # spring stiffness
B = 0.01 * 2*sqrt(K)   # damping
g = -9.81           # gravity
f1_stdev = 0.;      # motor standard deviation

f = 0.5;            # desired trajectory frequency
A = 0.05;           # desired trajectory amplitude
kp = 10.0;          # controller proportional feedback gain
kd = 2.0*sqrt(kp);  # controller derivative feedback gain
kf = 0.0;           # force proportional gain
ki = 5.0;           # force integral gain
B1_ctrl = 0.9*B1;

# at equilibrium we have: 
# f1 = -m1*g
# m1*g + m2*g = K*x2
# x2 = (m1+m2)*g / K
x1_0 = 0.0;
dx1_0 = 0*2*pi*f*A;
xL_0 = 0.5*(m1+mL+mR)*g/K
dxL_0 = 0.0;
xR_0 = xL_0;
dxR_0 = 0.0;

dt = 0.001;         # controller time step
T = 15;             # total simulation time
N = int(T/dt);

robot = Robot(omega, m1, mL, mR, C, K, B, g, f1_stdev, x1_0, dx1_0, xL_0, dxL_0, xR_0, dxR_0);

# integrate LDS
two_pi_f = 2*pi*f;

x1_ref    = np.zeros(N);
dx1_ref   = np.zeros(N);
ddx1_ref  = np.zeros(N);

fL   = np.zeros(N);
fL_ref = np.zeros(N);
fL_des = np.zeros(N);
fL_est = np.zeros(N);
fR   = np.zeros(N);
fR_ref = np.zeros(N);
fR_des = np.zeros(N);
fR_est = np.zeros(N);
CoP_ref = np.zeros(N);
f1   = np.zeros(N);
f2   = np.zeros(N);
x1   = np.zeros(N);
dx1  = np.zeros(N);
ddx1 = np.zeros(N);
xR   = np.zeros(N);
dxR  = np.zeros(N);
ddxR = np.zeros(N);
xR   = np.zeros(N);
dxR  = np.zeros(N);
ddxR = np.zeros(N);

f1_err_int = 0
f1[-1] =  -K*xL_0 - B*dxL_0;
f2[-1] =  -K*xR_0 - B*dxR_0;
for n in range(N):
    # store data
    x1[n] = robot.x1;
    xL[n] = robot.xL;
    xR[n] = robot.xR;
    dx1[n] = robot.dx1;
    dxL[n] = robot.dxL;
    dxR[n] = robot.dxR;
    
    # compute desired trajectory
    t = n*dt;
    x1_ref[n]   =  A*sin(two_pi_f*t);
    dx1_ref[n]  =  two_pi_f*A*cos(two_pi_f*t);
    ddx1_ref[n] = -two_pi_f*two_pi_f*A*sin(two_pi_f*t);
    ddx1_des = ddx1_ref[n] + kd*(dx1_ref[n]-dx1[n]) + kp*(x1_ref[n]-x1[n])

    # ddq1 = omega*(q1-CoP)

    # compute reference CoP
    CoP_ref[n] = x1[n] - ddx1_des/omega
    if(CoP_ref[n]>xR):
        CoP_ref[n] = xR;
    if(CoP_ref[n]<xL):
        CoP_ref[n] = xL;
    # compute reference contact forces
    fL_ref[n] = M*g*(CoP_ref[n]-xL)/(xR-xL);
    fR_ref[n] = M*g - fL_ref[n];
    
#    f1_est[n] = f2[n-1] + m2*g
#    f1_err = f1_ref[n] - f1_est[n]
#    f1_err_int += ki*dt*f1_err
#    f1_des[n] = f1_ref[n] + kf*f1_err + f1_err_int + B1_ctrl*(dx1[n]-dx2[n])

    # simulate
    robot.simulate(fL_ref[n], fR_ref[n], dt);

    # store data
    fL[n] = robot.fL;
    fR[n] = robot.fR;
    ddx1[n] = robot.ddx1;
    ddxL[n] = robot.ddxL;
    ddxR[n] = robot.ddxR;
    
print "Pos tracking error:  ", np.mean(np.abs(x1_ref-x1))
print "Vel tracking error:  ", np.mean(np.abs(dx1_ref-dx1))
#print "Force tracking error:", np.mean(np.abs(f1_ref-f1))

plt.figure()
plt.plot(x1, label='x1');
plt.plot(x1_ref, '-', label='x1 ref', alpha=0.6)
#plt.plot(x2, label='x2');
plt.legend()

plt.figure()
plt.plot(dx1, label='dx1');
plt.plot(dx1_ref, '--', label='dx1 des');
#plt.plot(dx2, label='dx2', alpha=0.5);
plt.legend()
#
#plt.figure()
#plt.plot(ddx1, label='ddx1');
#plt.plot(ddx1_ref, '--', label='ddx1 des');
#plt.plot(ddx2, label='ddx2', alpha=0.5);
#plt.legend()

#plt.figure()
#plt.plot(f1_ref, '--', label='f1 ref');
#plt.plot(f1_est, label='f1 est');
#plt.plot(f1, label='f1', alpha=0.5);
#plt.plot(f2, label='f2');
#plt.legend()
plt.show()
