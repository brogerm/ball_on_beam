import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import bobParam as P


class bobAnimation:
    '''
        Create ball on beam animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        self.length = P.length        # block width
        self.height = P.height      # block height
        self.ballD = P.d
        plt.axis([-3*self.length, 3*self.length, -0.1, 3*self.height])  # Change the x,y axis limits
        plt.plot([-3*self.length, 3*self.length], [0,0], 'b--')    # Draw a base line
        plt.xlabel('z')

        # Draw vtol is the main function that will call the functions:
        # drawBlock, drawSpring, and drawDamper to create the animation.
    def draw_bob(self, u):
        # Process inputs to function
        z = u[0]        # Position of ball in relation to center of beam rotation (m)
        theta = u[1]    # Angular position of beam

        self.draw_beam(z, theta)
        self.draw_ball(z, theta)
        self.ax.axis('equal') # This will cause the image to not distort

        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def draw_beam(self, z, theta):
        X = [0, self.length * np.cos(theta)]  # X data points
        Y = [0, self.length * np.sin(theta)]  # Y data points

        # When the class is initialized, a line object will be
        # created and added to the axes. After initialization, the
        # line object will only be updated.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, = self.ax.plot(X, Y, lw=5, c='black')
            self.handle.append(line)
        else:
            self.handle[0].set_xdata(X)  # Update the line
            self.handle[0].set_ydata(Y)

    def draw_ball(self, z, theta):
        x = z * np.cos(theta) - self.ballD/1.5 * np.sin(theta)    # x coordinate of center
        y = z * np.sin(theta) + self.ballD/1.5 * np.cos(theta)  # y coordinate of center
        xy = (x, y)  # Center of block

        # When the class is initialized, a Rectangular patch object will
        # be created and added to the axes. After initialization, the
        # Rectangular patch object will only be updated.
        if self.flagInit == True:
            # Create the Circular patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Circle(xy,
                radius=self.ballD/2, fill=True,
                fc='red'))
            self.ax.add_patch(self.handle[1])  # Add the patch to the axes
            self.flagInit = False
        else:
            self.handle[1].center = xy


# Used see the animation from the command line
if __name__ == "__main__":

    simAnimation = bobAnimation()    # Create Animate object
    z = 2                             # Position of cart, m
    theta = np.pi/2
    simAnimation.draw_bob([z, theta])  # Draw the mass spring damper system
    #plt.show()
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()