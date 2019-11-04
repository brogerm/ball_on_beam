import numpy as np
import control as cnt
import sys
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
tr_theta = 0.15    # rise time for angle
tr_z = 0.9        # rise time for position
zeta_z = 0.707  # damping ratio position
zeta_th = 0.707  # damping ratio angle

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

# gain calculation
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly = np.convolve([1, 2*zeta_z*wn_z, wn_z**2], [1, 2*zeta_th*wn_th, wn_th**2])
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 4:
    print("The system is not controllable")
else:
    K = cnt.acker(A, B, des_poles)
    kr = -1.0/(C[0]*np.linalg.inv(A-B*K)*B)

print('K: ', K)
print('kr: ', kr)