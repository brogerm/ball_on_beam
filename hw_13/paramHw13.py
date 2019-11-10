import numpy as np
import control as cnt
import sys
from scipy import signal
sys.path.append('..')  # add parent directory
import bobParam as P

# sample rate of the controller
Ts = P.Ts

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2 * sigma - Ts) / (2 * sigma + Ts)  # dirty derivative gain

# saturation limits
F_max = 100.0                # Max Force, N
theta_max = 30.0*np.pi/180.0  # Max theta, rads

####################################################
#                 State Space
####################################################
# tuning parameters
tr_theta = 0.5    # rise time for angle
tr_z = 0.9        # rise time for position
zeta_z = 0.707  # damping ratio position
zeta_th = 0.707  # damping ratio angle
integrator_pole = -2.8
tr_z_obs = tr_z/5.0          # rise time for observer - position
tr_theta_obs = tr_theta/5.0  # rise time for observer - angle

# z equilibrium
ze = P.length/2
a1 = -P.m1*P.g/(P.m2*P.length**2/3 + P.m1*ze**2)
b1 = P.length/(P.m2*P.length**2/3 + P.m1*ze**2)

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.matrix([[0.0,  0.0, 1.0, 0.0],
               [0.0,  0.0, 0.0, 1.0],
               [0.0, -P.g, 0.0, 0.0],
               [a1,   0.0, 0.0, 0.0]])

B = np.matrix([[0.0],
               [0.0],
               [0.0],
               [b1]])

C = np.matrix([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])

# Form augmented matrices
Cr = np.matrix([[1.0, 0.0, 0.0, 0.0]])

A1 = np.matrix([[0.0,  0.0, 1.0, 0.0, 0.0],
               [0.0,  0.0, 0.0, 1.0, 0.0],
               [0.0, -P.g, 0.0, 0.0, 0.0],
               [a1,   0.0, 0.0, 0.0, 0.0],
               [-1.0, 0.0, 0.0, 0.0, 0.0]])

B1 = np.matrix([[0.0],
               [0.0],
               [0.0],
               [b1],
               [0.0]])

# gain calculation
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position

des_char_poly = np.convolve(
    np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                [1, 2*zeta_th*wn_th, wn_th**2]),
    np.poly([integrator_pole]))
des_poles = np.roots(des_char_poly)

print(des_poles)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
    print("The system is not controllable")
else:
    K1 = cnt.acker(A1, B1, des_poles)
    K = np.matrix([K1.item(0), K1.item(1), K1.item(2), K1.item(3)])
    ki = K1.item(4)


# computer observer gains
wn_z_obs = 2.2/tr_z_obs
wn_th_obs = 2.2/tr_theta_obs
des_obs_char_poly = np.convolve([1, 2*zeta_z*wn_z_obs, wn_z_obs**2],
                                 [1, 2*zeta_th*wn_th_obs, wn_th_obs**2])
des_obs_poles = np.roots(des_obs_char_poly)

# Compute the gains if the system is observable
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 4:
    print("The system is not observable")
else:
    # place_poles returns an object with various properties.  The gains are accessed through .gain_matrix
    # .T transposes the matrix
    L = signal.place_poles(A.T, C.T, des_obs_poles).gain_matrix.T


print('K: ', K)
print('ki: ', ki)
print('L^T: ', L.T)