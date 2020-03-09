import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import eigvals, expm

x_0 = np.array([3, 0, 0, 0])
dt = 0.1
T = 5
poles = np.array([-1, -1.1, -1.2, -1.3]) - 1

# define system dynamics matrices
A = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0]])
#              [-kp, -kd, -ka, -kj]]);
B = np.array([[0, 0, 0, 1]]).T

try:
    from scipy.signal import place_poles
    # compute gains by pole placement
    full_state_feedback = place_poles(A, B, poles)
    K = full_state_feedback.gain_matrix
    kp = K[0, 0]
    kd = K[0, 1]
    kj = K[0, 2]
    ka = K[0, 3]
    # 13 + (kd + kj*kj*kp/kd)/kj
except Exception:
    print("ERROR while doing pole placement")
    kp = 30
    kd = 2 * np.sqrt(kp)
    kj = 0
    ka = 0
    K = np.array([[kp, kd, ka, kj]])

# check stability
print("kp=", kp)
print("kd=", kd)
print("ka=", ka)
print("kj=", kj)
if (kj * ka <= kd):
    print("WARNING: System is unstable because kj*ka<=kd")
if (kj * ka * kd <= kd * kd + kj * kj * kp):
    print("WARNING: System is unstable because kj*ka*kd <= kd*kd + kj*kj*kp", kj * ka * kd - (kd * kd + kj * kj * kp))

G = A - np.dot(B, K)
ev = eigvals(G)
print("Eigenvalues of A-BK:\n", ev)

# integrate LDS
N = int(T / dt)
x = np.zeros((4, N))
x[:, 0] = x_0
e_Adt = expm(dt * G)
for n in range(1, N):
    x[:, n] = np.dot(e_Adt, x[:, n - 1])

plt.plot(x[0, :])
plt.title('Position')
plt.show()
