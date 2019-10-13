import numpy as np
import bobParam as P

class bobController:
    def __init__(self):
        self.kp_z = P.kp_z
        self.kd_z = P.kd_z
        self.kp_th = P.kp_th
        self.kd_th = P.kd_th
        self.m1 = P.m1
        self.m2 = P.m2
        self.length = P.length
        self.g = P.g

    def update(self, z_r, state):
        z = state[0]
        theta = state[1]
        zdot = state[2]
        thetadot = state[3]

        Fe = self.m1*self.g*z/self.length + self.m2*self.g/2
        # the reference angle for theta comes from the outer loop PD control
        theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot
        # The force applied to the rod comes from the inner loop PD control
        F = self.kp_th * (theta_r - theta) - self.kd_th * thetadot + Fe
        return [F]
