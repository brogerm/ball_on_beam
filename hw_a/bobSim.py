import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import bobParam as P
from signalGenerator import signalGenerator
from bobAnimation import bobAnimation
from plotData import plotData


# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
zRef = signalGenerator(amplitude=3, frequency=0.1)
thetaRef = signalGenerator(amplitude=1/4*np.pi, frequency=0.1)
fRef = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = bobAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    r = reference.square(t)
    z = zRef.sin(t)
    theta = thetaRef.sin(t)
    f = fRef.sawtooth(t)
    # update animation
    state = [-z[0]+2, theta[0]]
    animation.draw_bob(state)
    dataPlot.updatePlots(t, r, state, f)

    t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.1)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
