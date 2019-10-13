# Inverted Pendulum Parameter File
import numpy as np
import control as cnt

# General physical parameters
g = 9.81    # m/s^2

# Physical parameters of the ball
m1 = 0.35      # mass of the ball
d = 0.25       # diameter of the ball (m)

# Physical parameters of the beam
m2 = 2      # mass of the beam
b = 0.0     # damping coefficient
length = 0.5  # length of the beam

# Animation params
height = 0.125   # width of the beam

# Initial conditions
z0 = 3*length/4          # initial displacement (m)
zdot0 = 0       # initial velocity (m/s)
theta0 = 0      # initial angular position (rad)
thetadot0 = 0   # initial angular velocity (rad/s)

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 50.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate

####################################################
#       PD Control: Time Design Strategy
####################################################
tr_th = 1
zeta_th = 0.707
tr_z = 10 * tr_th
zeta_z = 0.707

# --------------------------
#           Inner Loop
# --------------------------
# coefficients for desired inner loop
wn_th = 2.2/tr_th
alpha1_th = 2.0 * zeta_th * wn_th
alpha0_th = wn_th**2

kd_th = alpha1_th * (m2 * length / 3 + m1 * length / 4)
kp_th = alpha0_th * (m2 * length / 3 + m1 * length / 4)
DC_gain = 1

# --------------------------
#           Inner Loop
# --------------------------
# coefficients for desired inner loop
wn_z = 2.2/tr_z
alpha1_z = 2.0 * zeta_z * wn_z
alpha0_z = wn_z**2

kd_z = -alpha1_z * DC_gain/g
kp_z = -alpha0_z * DC_gain/g

print('DC_gain', DC_gain)
print('kp_th: ', kp_th)
print('kd_th: ', kd_th)
print('kp_z: ', kp_z)
print('kd_z: ', kd_z)



# saturation limits
F_max = 5.0                # Max Force, N

