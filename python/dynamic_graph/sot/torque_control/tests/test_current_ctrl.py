import numpy as np
from numpy.random import normal
import matplotlib.pyplot as plt
from math import sin, pi, exp, log

EPS = 1e-6

def saturate(x, x_max):
    if(x>x_max):
        return x_max;
    if(x<-x_max):
        return -x_max;
    return x;

''' Integrate x' + ax = b until either:
     - you reach the specified time t
     - you reach the specified lower/upper bound
'''
def integrate_1_order_lde(x_0, a, b, t, x_lb=None, x_ub=None):
    if(x_lb is not None and x_0<x_lb):
        print "ERROR: x_0=%.3f < x_lb=%.3f"%(x_0, x_lb)
        return (0, x_lb);
    if(x_ub is not None and x_0>x_ub):
        print "ERROR: x_0=%.3f > x_ub=%.3f"%(x_0, x_ub)
        return (0, x_lb);
    c = b/a;
    e = exp(-a*t);
    x_t = c*(1-e) + x_0*e; # if a<0 => x tends towards c
    if(x_lb is not None and x_t<x_lb):
        t = -log((x_lb-c)/(x_0-c)) / a;
        #t = -log((a*x_lb - b)/(a*x_0-b)) / a;
        return (t, x_lb);
    if(x_ub is not None and x_t>x_ub):
        t = -log((x_ub - b/a)/(x_0-b/a)) / a;
        return (t, x_ub);
    return (t, x_t);
    
class Motor:
    def __init__(self, R, L, Kb, Kt, J, b):
        self.R = R;
        self.L = L;
        self.Kb = Kb;
        self.Kt = Kt;
        self.J = J;
        self.b = b;
        self.i = 0;
        self.dq = 0;
        
    def simulate(self, V, dt):
#        di = (V - self.R*self.i - self.Kb*self.dq)/self.L;
#        di = (K*(u-i) - R*i)/L;
#        di = (K*u - (K+R)*i)/L;
#        self.i += di*dt;
        a = self.R/self.L;
        b = (V - self.Kb*self.dq)/self.L;
        (t,self.i) = integrate_1_order_lde(self.i, a, b, dt);
        
        ddq = (self.Kt*self.i - self.b*self.dq)/self.J;
        self.dq += ddq*dt;
        
class Hrp2CurrentControl:    
    def __init__(self, motor, K, i_dz):
        self.motor = motor;
        self.K = K;         # current-control feedback gain
        self.i_dz = i_dz;   # dead-zone threshold
        self.t = 0;
        self.verbose = False;
        
    def get_deadzone_state(self, e):
        if(e>self.i_dz):
            return 1;
        elif(e<-self.i_dz):
            return -1;
        return 0;
        
    def simulate(self, u, t, max_iter=10):  
        time_left = t;
        motor = self.motor;
        dz_state = 0;
        for n in range(max_iter):
            i_0 = motor.i;
            dz_state = self.get_deadzone_state(u-i_0);
            if(dz_state==1):    #V = self.K*(e-i_dz);
                A = self.K;
                d = self.K*(u-self.i_dz);
                ub = u-self.i_dz+EPS;
                lb = None;
            elif(dz_state==-1): #V = self.K*(e+i_dz);
                A = self.K;
                d = self.K*(u+self.i_dz);
                lb = u+self.i_dz-EPS;
                ub = None;
            else:               #V = 0.0;
                A = 0;
                d = 0;
                lb = u-self.i_dz-EPS;
                ub = u+self.i_dz+EPS;
            #self.motor.simulate(V, dt/N_DT);
            #di = (d-A*i - self.R*self.i - self.Kb*self.dq)/self.L;        
            a = (motor.R+A)/motor.L;
            b = (d - motor.Kb*motor.dq)/motor.L;
            (dt,motor.i) = integrate_1_order_lde(i_0, a, b, time_left, lb, ub);
            self.t += dt;
            
            ddq = (motor.Kt*motor.i - motor.b*motor.dq)/motor.J;
            motor.dq += ddq*dt;
            
            if(dt==time_left):
                return;
                
            #print "Time %.3f, iter %d, integration stopped after t=%.3f ms"%(self.t, n, 1e3*dt);
            new_dz_state = self.get_deadzone_state(u-motor.i);
            time_left -= dt;
            if(new_dz_state==dz_state):
                print "Time %.3f: ERROR deadzone state did not change from %d, i(0)=%.3f, i(t)=%.3f, e(0)=%.3f, e(t)=%.3f"%(self.t, dz_state, i_0, motor.i, u-i_0, u-motor.i);
                return;
            if(self.verbose):
                print "Time %.3f: deadzone state changed from %d to %d, i(0)=%.3f, i(t)=%.3f, e(0)=%.3f, e(t)=%.3f"%(self.t, dz_state, new_dz_state, i_0, motor.i, u-i_0, u-motor.i)
                    
        print "Time %.3f: ERROR max number of iterations reached: %d"%(self.t, max_iter)
        
R = 3.3;    # resistance
L = 2e-3;    # inductance
Kb = 0.19;   # back-EMF constant
Kt = 0.19;   # current-torque constant
J = 1000.0;    # inertia
b = 0.5;    # viscous friction
K = 300;    # proportional current control gain
i_dz = 0.5; # dead-zone threshold

i_off = 0.0;            # current sensor offset
i_stdev = 0.02;         # std dev of current sensor noise
i_trans = 3*i_stdev;    # transition band for dead-zone compensation
dz_comp_perc = 1.0;
ki = 0e-2;

N = 1500;    # number of time steps
dt = 0.002; # time step duration
f = 0.6;
A = 2.0;

motor = Motor(R, L, Kb, Kt, J, b);
hrp2 = Hrp2CurrentControl(motor, K, i_dz)

i_des = np.zeros(N);
u = np.zeros(N);
i = np.zeros(N);
i_mes = np.zeros(N);
dq = np.zeros(N);
err_int = 0;
dz_comp_state = 0
new_dz_comp_state = 0

print "Starting simulation..."
for n in range(N):
    # compute desired current
    t = n*dt - 2.0;
    i_des[n] = A*sin(2*pi*f*t) + t**4 + t**3 -2*t*t -t +1;
    # store data
    i[n] = motor.i;
    dq[n] = motor.dq;
    i_noise = saturate(normal(0,i_stdev), 3*i_stdev);
    i_mes[n] = i[n]+i_off+i_noise;
    # compute control law
    err_int += ki*(i_des[n]-i_mes[n]);
    u[n] = i_des[n] + err_int;
    e = u[n]-i_mes[n];
    
    if(e > i_trans):
        u[n] += dz_comp_perc*i_dz;
        new_dz_comp_state = 1;
    elif(e < -i_trans):
        u[n] -= dz_comp_perc*i_dz;
        new_dz_comp_state = -1;
    elif(u[n] > 0):
        u[n] += dz_comp_perc*i_dz + e-i_trans;
        new_dz_comp_state = 0;
    else:
        u[n] -= dz_comp_perc*i_dz - (e+i_trans);
        new_dz_comp_state = 0;
#    elif(i_trans!=0.0):
#        u[n] += e*dz_comp_perc*i_dz/i_trans;
#        new_dz_comp_state = 0;

    if(dz_comp_state!=new_dz_comp_state):
        print "Time %.3f: dead-zone compensation state changed from %d to %d"%(t+2, dz_comp_state, new_dz_comp_state);
        dz_comp_state=new_dz_comp_state;

    # simulate one time step
    hrp2.simulate(u[n], dt);
    
    
time = np.arange(0, N*dt, dt);
plt.figure();
plt.plot(time, i_des, '-', label='i des');
plt.plot(time, u, '-', label='u');
plt.plot(time, i, '-', label='i');
plt.plot(time, i_mes, '-', label='i measured', alpha=0.3);
#plt.plot(time, dq, label='dq');
#plt.plot(time, i_des-i, label='error');
plt.grid()
plt.legend()
plt.show();