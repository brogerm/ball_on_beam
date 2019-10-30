import numpy as np
from PIDControl import PIDControl
import sys
sys.path.append('..')  # add parent directory
import bobParam as P

class bobController:
    def __init__(self):
        # Instantiates the PD object
        self.zCtrl = PIDControl(P.kp_z, P.ki_z, P.kd_z, P.theta_max, P.beta, P.Ts)
        self.thCtrl = PIDControl(P.kp_th, P.ki_th, P.kd_th, P.F_max, P.beta, P.Ts)
        self.ThetaLimit = P.theta_max
        self.m1 = P.m1
        self.g = P.g
        self.length = P.length
        self.m2 = P.m2

    def update(self, z_r, state):
        z = state[0]
        theta = state[1]

        Fe = self.m1*self.g*z/self.length + self.m2*self.g/2
        # the reference angle for theta comes from the outer loop PD control
        theta_r = self.zCtrl.PID(z_r, z, flag=False)
        theta_r = self.saturate(theta_r, self.ThetaLimit)
        # The force applied to the rod comes from the inner loop PD control
        F = self.thCtrl.PD(theta_r, theta, flag=False) + Fe
        return [F]

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
        return u
