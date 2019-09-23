# Inverted Pendulum Parameter File
import numpy as np
import control as cnt

# Physical parameters of the ball
m1 = 5      # mass of the ball
d = 1       # diameter of the ball (m)

# Physical parameters of the beam
m2 = 10     # mass of the beam
length = 4  # length of the beam

# Animation params
height = 0.125   # width of the beam

# Initial conditions
z0 = 4          # initial displacement (m)
zdot0 = 0       # initial velocity (m/s)
theta0 = 0      # initial angular position (rad)
thetadot0 = 0   # initial angular velocity (rad/s)

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 50.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate

# saturation limits
F_max = 5.0                # Max Force, N

