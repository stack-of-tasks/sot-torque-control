from math import cos, pi, sin, sqrt

import matplotlib.pyplot as plt
import numpy as np
from numpy.random import normal


class Robot:
    def __init__(self, m1, m2, B1, C1, K2, B2, g, f1_stdev, x1=0, dx1=0, x2=0, dx2=0):
        self.m1 = m1
        self.m2 = m2
        self.B1 = B1
        self.C1 = C1
        self.K2 = K2
        self.B2 = B2
        self.g = g
        self.f1_stdev = f1_stdev
        self.x1 = x1
        self.dx1 = dx1
        self.x2 = x2
        self.dx2 = dx2

    def simulate(self, f1, T, dt=0.0001):
        N = int(T / dt)
        assert N > 0
        f1_noise = normal(0, self.f1_stdev)
        f1_sum = 0
        f2_sum = 0
        for i in range(N):
            # compute forces
            self.f2 = -self.K2 * self.x2 - self.B2 * self.dx2
            self.f1 = f1 + f1_noise
            self.f1 -= self.B1 * (self.dx1 - self.dx2) + self.C1 * np.sign(self.dx1 - self.dx2)
            f1_sum += self.f1
            f2_sum += self.f2
            # compute accelerations
            self.ddx1 = self.g + (self.f1) / self.m1
            self.ddx2 = self.g + (-self.f1 + self.f2) / self.m2
            # integrate
            self.x1 += dt * self.dx1 + 0.5 * dt * dt * self.ddx1
            self.x2 += dt * self.dx2 + 0.5 * dt * dt * self.ddx2
            self.dx1 += dt * self.ddx1
            self.dx2 += dt * self.ddx2
        self.f1 = f1_sum / N
        self.f2 = f2_sum / N


# *** SYSTEM DESCRIPTION ***
# q1 is the position of m1 w.r.t. the world
# q2 is the position of m2 w.r.t. the world
# we want to control q1

# m1*(ddq1-g) = f1
# m2*(ddq2-g) = f2 - f1
# f2 = -K2*q2 - B2*dq2
# f1 = f1d - B1*(dq1-dq2)

m1 = 60.0  # first mass
m2 = 1.0  # second mass
B1 = 0.5 * m1  # motor viscous friction
C1 = 0.5 * B1
# motor Coulomb friction
hl = 0.09  # half distance between feet
K2 = 10 * 2 * 2e5 * hl * hl  # spring stiffness
B2 = 0.01 * 2 * sqrt(K2)  # damping
g = -9.81  # gravity
f1_stdev = 0.0
# motor standard deviation

f = 0.5
# desired trajectory frequency
A = 0.05
# desired trajectory amplitude
kp = 10.0
# controller proportional feedback gain
kd = 2.0 * sqrt(kp)
# controller derivative feedback gain
kf = 0.0
# force proportional gain
ki = 5.0
# force integral gain
B1_ctrl = 0.9 * B1

# at equilibrium we have:
# f1 = -m1*g
# m1*g + m2*g = K*x2
# x2 = (m1+m2)*g / K
x1_0 = 0.0
dx1_0 = 0 * 2 * pi * f * A
x2_0 = (m1 + m2) * g / K2
dx2_0 = 0.0

dt = 0.001
# controller time step
T = 15
# total simulation time
N = int(T / dt)
PLOT_SPRING_DAMPER_RESPONSE = 0

if PLOT_SPRING_DAMPER_RESPONSE:
    # plot oscillations of spring damper system: (m1+m2)*ddx = -K*x - D*dx
    x = np.zeros(N)
    dx = np.zeros(N)
    x[0] = 0.01
    for n in range(N - 1):
        ddx = (-K * x[n] - D * dx[n]) / (m1 + m2)  # noqa TODO
        x[n + 1] = x[n] + dt * dx[n] + 0.5 * dt * dt * ddx
        dx[n + 1] = dx[n] + dt * ddx
    plt.figure()
    plt.plot(x, label="x")
    # plt.plot(dx, label='dx');
    plt.legend()
    plt.show()

robot = Robot(m1, m2, B1, C1, K2, B2, g, f1_stdev, x1_0, dx1_0, x2_0, dx2_0)

# integrate LDS
two_pi_f = 2 * pi * f

x1_ref = np.zeros(N)
dx1_ref = np.zeros(N)
ddx1_ref = np.zeros(N)

f1 = np.zeros(N)
f1_ref = np.zeros(N)
f1_des = np.zeros(N)
f1_est = np.zeros(N)
f2 = np.zeros(N)
x1 = np.zeros(N)
dx1 = np.zeros(N)
ddx1 = np.zeros(N)
x2 = np.zeros(N)
dx2 = np.zeros(N)
ddx2 = np.zeros(N)

f1_err_int = 0
f2[-1] = -K2 * x2_0 - B2 * dx2_0
for n in range(N):
    # store data
    x1[n] = robot.x1
    x2[n] = robot.x2
    dx1[n] = robot.dx1
    dx2[n] = robot.dx2

    # compute desired trajectory
    t = n * dt
    x1_ref[n] = A * sin(two_pi_f * t)
    dx1_ref[n] = two_pi_f * A * cos(two_pi_f * t)
    ddx1_ref[n] = -two_pi_f * two_pi_f * A * sin(two_pi_f * t)
    ddx1_des = ddx1_ref[n] + kd * (dx1_ref[n] - dx1[n]) + kp * (x1_ref[n] - x1[n])

    # compute actuator force
    f1_ref[n] = m1 * (ddx1_des - g)
    f1_est[n] = f2[n - 1] + m2 * g
    f1_err = f1_ref[n] - f1_est[n]
    f1_err_int += ki * dt * f1_err
    f1_des[n] = f1_ref[n] + kf * f1_err + f1_err_int + B1_ctrl * (dx1[n] - dx2[n])

    # simulate
    robot.simulate(f1_des[n], dt)

    # store data
    f1[n] = robot.f1
    f2[n] = robot.f2
    ddx1[n] = robot.ddx1
    ddx2[n] = robot.ddx2

print("Pos tracking error:  ", np.mean(np.abs(x1_ref - x1)))
print("Vel tracking error:  ", np.mean(np.abs(dx1_ref - dx1)))
print("Force tracking error:", np.mean(np.abs(f1_ref - f1)))

plt.figure()
plt.plot(x1, label="x1")
plt.plot(x1_ref, "-", label="x1 ref", alpha=0.6)
plt.plot(x2, label="x2")
plt.legend()

plt.figure()
plt.plot(dx1, label="dx1")
plt.plot(dx1_ref, "--", label="dx1 des")
plt.plot(dx2, label="dx2", alpha=0.5)
plt.legend()
#
# plt.figure()
# plt.plot(ddx1, label='ddx1');
# plt.plot(ddx1_ref, '--', label='ddx1 des');
# plt.plot(ddx2, label='ddx2', alpha=0.5);
# plt.legend()

plt.figure()
plt.plot(f1_ref, "--", label="f1 ref")
plt.plot(f1_est, label="f1 est")
plt.plot(f1, label="f1", alpha=0.5)
# plt.plot(f2, label='f2');
plt.legend()
plt.show()
