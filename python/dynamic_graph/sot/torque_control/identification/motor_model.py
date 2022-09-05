#!/usr/bin/env python
# -*- coding: utf-8 -*-

# i = Kt*tau + Kv*dq + Ka*ddq + sign(dq)Kf
# tau = i/Kt - (Kv/Kt)*dq - (Ka/Kt)*ddq - sign(dq)(Kf/Kt)
from IPython import embed  # noqa


def smoothSign(value, threshold):
    if value > threshold:
        return 1.0
    elif value < -threshold:
        return -1.0
    return pow(value / threshold, 3)


class Motor_model:
    def __init__(self, Kt_p, Kt_n, Kf_p, Kf_n, Kv_p, Kv_n, Ka_p, Ka_n, dqThreshold=0.8):
        self.Kt_p = Kt_p
        self.Kt_n = Kt_n

        self.Kf_p = Kf_p
        self.Kf_n = Kf_n

        self.Kv_p = Kv_p
        self.Kv_n = Kv_n

        self.Ka_p = Ka_p
        self.Ka_n = Ka_n
        self.dqThreshold = dqThreshold

    def getCurrent(self, tau, dq, ddq):
        Kt_p = self.Kt_p
        Kt_n = self.Kt_n

        Kf_p = self.Kf_p
        Kf_n = self.Kf_n

        Kv_p = self.Kv_p
        Kv_n = self.Kv_n

        Ka_p = self.Ka_p
        Ka_n = self.Ka_n

        signDq = smoothSign(dq, self.dqThreshold)
        # in [-1;1]
        # Smoothly set Coefficients according to velocity sign
        Kt = 0.5 * (Kt_p * (1 + signDq) + Kt_n * (1 - signDq))
        Kv = 0.5 * (Kv_p * (1 + signDq) + Kv_n * (1 - signDq))
        Ka = 0.5 * (Ka_p * (1 + signDq) + Ka_n * (1 - signDq))
        Kf = 0.5 * (Kf_p * (1 + signDq) + Kf_n * (1 - signDq))
        current = Kt * tau + Kv * dq + Ka * ddq + signDq * Kf
        return current

    def getTorque(self, current, dq, ddq):
        Kt_p = self.Kt_p
        Kt_n = self.Kt_n

        Kf_p = self.Kf_p
        Kf_n = self.Kf_n

        Kv_p = self.Kv_p
        Kv_n = self.Kv_n

        Ka_p = self.Ka_p
        Ka_n = self.Ka_n

        signDq = smoothSign(dq, self.dqThreshold)
        # in [-1;1]
        # Smoothly set Coefficients according to velocity sign
        Kt = 0.5 * (Kt_p * (1 + signDq) + Kt_n * (1 - signDq))
        Kv = 0.5 * (Kv_p * (1 + signDq) + Kv_n * (1 - signDq))
        Ka = 0.5 * (Ka_p * (1 + signDq) + Ka_n * (1 - signDq))
        Kf = 0.5 * (Kf_p * (1 + signDq) + Kf_n * (1 - signDq))
        torque = (current / Kt) - (Kv / Kt) * dq - (Ka / Kt) * ddq - signDq * (Kf / Kt)
        return torque
