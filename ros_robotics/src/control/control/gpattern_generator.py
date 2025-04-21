import time
from math import exp


class GPGenerator:
    def __init__(self, current_time=None):
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.r = 0.0
        self.teta = 0.0
        self.phi = 0.0

        self.mu = 1.0
        self.omega = 1.0
        self.xi = 1.0
        self.a = 1.0

        self.r_prev = 0.0
        self.dr_prev = 0.0

        self.teta_prev = 0.0
        self.phi_prev = 0.0


    def update(self, current_time=None):
        self.current_time = current_time if current_time is not None else time.time()
        t = self.current_time - self.last_time

        k1 = 0.125 * self.a ** 2 * self.mu
        k2 = self.dr_prev + 0.5 * self.a * self.r_prev
        k3 = self.r_prev
        self.r = (k1 * t ** 2 + k2 * t + k3) * exp(-0.5 * self.a * t)

        self.teta = self.omega * t + self.teta_prev
        self.phi = self.xi * t + self.phi_prev


    def modulate(self, mu: float, omega: float, xi: float, current_time=None):
        self.current_time = current_time if current_time is not None else time.time()
        t = self.current_time - self.last_time

        k11 = 0.125 * self.a ** 2 * self.mu
        k12 = self.dr_prev + 0.5 * self.a * self.r_prev
        k13 = self.r_prev

        k21 = -0.0625 * self.a ** 3 * self.mu
        k22 = 0.5 * self.a * (0.5 * self.a * self.mu - 0.5 * self.a * self.r_prev - self.dr_prev)
        k23 = self.dr_prev

        self.r = (k11 * t ** 2 + k12 * t + k13) * exp(-0.5 * self.a * t)
        self.dr_prev = (k21 * t ** 2 + k22 * t + k23) * exp(-0.5 * self.a * t)

        self.teta_prev = self.omega * t + self.teta_prev
        self.phi_prev = self.xi * t + self.phi_prev

        self.mu = mu
        self.omega = omega
        self.xi = xi

        self.last_time = current_time
