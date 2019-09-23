import numpy as np 
import random
import bobParam as P


class msdDynamics:
    '''
        Model the physical system
    '''

    def __init__(self):
        # Initial state conditions
        self.state = np.matrix([[P.z0],          # z initial position
                                [P.zdot0]])       # zdot initial velocity
        #################################################
        # The parameters for any physical system are never known exactly.  Feedback
        # systems need to be designed to be robust to this uncertainty.  In the simulation
        # we model uncertainty by changing the physical parameters by a uniform random variable
        # that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
        # may change by up to 20%.  A different parameter value is chosen every time the simulation
        # is run.
        alpha = 0.2  # Uncertainty parameter
        self.m = P.m * (1+2*alpha*np.random.rand()-alpha)  # Mass of the block, kg
        self.k = P.k * (1+2*alpha*np.random.rand()-alpha)   # Spring constant,
        self.b = P.b * (1+2*alpha*np.random.rand()-alpha)  # Damping coefficient, Ns

    def propagateDynamics(self, u):
        '''
            Integrate the differential equations defining dynamics
            P.Ts is the time step between function calls.
            u contains the system input(s).
        '''
        # Integrate ODE using Runge-Kutta RK4 algorithm
        k1 = self.derivatives(self.state, u)
        k2 = self.derivatives(self.state + P.Ts/2*k1, u)
        k3 = self.derivatives(self.state + P.Ts/2*k2, u)
        k4 = self.derivatives(self.state + P.Ts*k3, u)
        self.state = self.state + P.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)

    def derivatives(self, state, u):
        '''
            Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
        '''
        # re-label states and inputs for readability
        F = u[0]
        z = state.item(0)
        zdot = state.item(1)
        zddot = 1/self.m * (F - self.b * zdot - self.k*z)
        # The equations of motion.
        x1dot = zdot
        x2dot = zddot
        # build xdot and return
        xdot = np.matrix([[x1dot], [x2dot]])
        return xdot

    def outputs(self):
        '''
            Returns the measured outputs as a list
            [z] with added Gaussian noise
        '''
        # re-label states for readability
        z = self.state.item(0)
        # add Gaussian noise to outputs
        z_m = z + random.gauss(0, 0.01)
        # return measured outputs
        return [z_m]

    def states(self):
        '''
            Returns all current states as a list
        '''
        return self.state.T.tolist()[0]