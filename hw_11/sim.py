import matplotlib.pyplot as plt
import numpy as np
import sys
from controller import Controller
sys.path.append('..')  # add parent directory
import bobParam as P
from signalGenerator import signalGenerator
from bobAnimation import bobAnimation
from plotData import plotData
from bobDynamics import bobDynamics

# instantiate reference input classes
bob = bobDynamics()
ctrl = Controller()
reference = signalGenerator(amplitude=0.25, frequency=0.1)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = bobAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    ref_input = reference.square(t)
    ref_input = np.array(ref_input) + 0.25
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, bob.states())
        bob.propagateDynamics(u)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts

    # print(bob.states())
    animation.draw_bob(bob.states())
    dataPlot.updatePlots(t, ref_input, bob.states(), u)

    # t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.1)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
