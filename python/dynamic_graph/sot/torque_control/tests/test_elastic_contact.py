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
    def __init__(self, m1, m2, kd, K, D, g, f1_stdev, x1=0, dx1=0, x2=0, dx2=0):
        self.m1 = m1;
        self.m2 = m2;
        self.kd = kd;
        self.K = K;
        self.D = D;
        self.g = g;
        self.f1_stdev = f1_stdev;
        self.x1 = x1;
        self.dx1 = dx1;
        self.x2 = x2;
        self.dx2 = dx2;
        
    def simulate(self, f1, dt):
        # compute forces
        self.f2 = -self.K*self.x2 - self.D*self.dx2;
        self.f1 = f1 + normal(0, self.f1_stdev) - self.kd*self.dx1
        # compute accelerations
        self.ddx1 = self.g + (self.f1 )/self.m1;
        self.ddx2 = self.g + (-self.f1 + self.f2)/self.m2;
        # integrate
        self.x1 += dt*self.dx1 + 0.5*dt*dt*self.ddx1;
        self.x2 += dt*self.dx2 + 0.5*dt*dt*self.ddx2;
        self.dx1 += dt*self.ddx1;
        self.dx2 += dt*self.ddx2;
        
        
# m1*(ddx1-g) = f1
# m2*(ddx2-g) = f2 - f1
# f2 = -K*x2 - D*dx2
m1 = 60.
m2 = 1.
kv = 3.*m1
l = 0.09 # half distance between feet
K = 2*2e5*l*l
D = 0.2*sqrt(K)
g = -9.81
f1_stdev = 20.;

f = 0.5;
A = 0.05;
kp = 30.0;
kd = 0.0*sqrt(kp);

x1_0 = 0.0;
dx1_0 = 2*pi*f*A;
# at equilibrium we have: 
# f1 = -m1*g
# m1*g + m2*g = K*x2
# x2 = (m1+m2)*g / K
x2_0 = (m1+m2)*g/K
dx2_0 = 0.0;
dt = 0.001;
T = 20;
N = int(T/dt);
PLOT_SPRING_DAMPER_RESPONSE = 0

if(PLOT_SPRING_DAMPER_RESPONSE):
    # plot oscillations of spring damper system: (m1+m2)*ddx = -K*x - D*dx
    x = np.zeros(N)
    dx = np.zeros(N)
    x[0] = 0.01
    for n in range(N-1):
        ddx = (-K*x[n] - D*dx[n])/(m1+m2);
        x[n+1] = x[n] + dt*dx[n] + 0.5*dt*dt*ddx;
        dx[n+1] = dx[n] + dt*ddx;
    plt.figure()
    plt.plot(x, label='x');
    #plt.plot(dx, label='dx');
    plt.legend()
    plt.show()

robot = Robot(m1, m2, kv, K, D, g, f1_stdev, x1_0, dx1_0, x2_0, dx2_0);

# integrate LDS
two_pi_f = 2*pi*f;

x1_ref    = np.zeros(N);
dx1_ref   = np.zeros(N);
ddx1_ref  = np.zeros(N);

f1   = np.zeros(N-1);
f1_des = np.zeros(N-1);
f2   = np.zeros(N-1);
x1   = np.zeros(N);
dx1  = np.zeros(N);
ddx1 = np.zeros(N-1);
x2   = np.zeros(N);
dx2  = np.zeros(N);
ddx2 = np.zeros(N-1);
#e_Adt = expm(dt*G);

for n in range(N-1):
    # store data
    x1[n] = robot.x1 + robot.x2;
    x2[n] = robot.x2;
    dx1[n] = robot.dx1 + robot.dx2;
    dx2[n] = robot.dx2;
    
    # compute desired trajectory
    t = n*dt;
    x1_ref[n]   =  A*sin(two_pi_f*t);
    dx1_ref[n]  =  two_pi_f*A*cos(two_pi_f*t);
    ddx1_ref[n] = -two_pi_f*two_pi_f*A*sin(two_pi_f*t);
    ddx1_des = ddx1_ref[n] + kd*(dx1_ref[n]-dx1[n]) + kp*(x1_ref[n]-x1[n])

    # compute actuator force
    f1_des[n] = m1*(ddx1_des-g);

    # simulate    
    robot.simulate(f1_des[n], dt);

    # store data
    f1[n] = robot.f1;
    f2[n] = robot.f2;
    ddx1[n] = robot.ddx1;
    ddx2[n] = robot.ddx2;    

plt.figure()
plt.plot(x1, label='x1');
plt.plot(x1_ref, '-', label='x1 des', alpha=0.6)
plt.plot(x2, label='x2');
plt.legend()

plt.figure()
plt.plot(dx1, label='dx1');
plt.plot(dx1_ref, '--', label='dx1 des');
plt.plot(dx2, label='dx2', alpha=0.5);
plt.legend()
#
#plt.figure()
#plt.plot(ddx1, label='ddx1');
#plt.plot(ddx1_ref, '--', label='ddx1 des');
#plt.plot(ddx2, label='ddx2', alpha=0.5);
#plt.legend()

#plt.figure()
#plt.plot(f1, label='f1', alpha=0.5);
#plt.plot(f1_des, '--', label='f1 des');
#plt.plot(f2, label='f2');
#plt.legend()
plt.show()
