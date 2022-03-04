import numpy as np

class PID:

    def __init__(self, period):
        self.Kp = 1/100
        self.Ki = 1/200
        self.Kd = 1/400
        self.period = period
        self.I = 0
        self.e_prev = 0

    def get_accel(self, SP, PV):
        e = SP - PV

        P = self.Kp * e
        I = self.I + self.Ki * e * self.period
        D = self.Kd * (e - self.e_prev) / self.period
        self.e_prev = e
        return P+I+D
