import numpy as np


class System:
    """
    Helper class to simulate linear discrete time dynamical systems in state-space form.
    Initiate with system matrices: A,B,C,D such that:

    x_next = A@x + B@u
    y      = C@x + D@u

    Passing D is optional.
    Passing x0 is optional (will results to all zero per default).

    Run a simulation step with make_step method.

    Reset with reset method.

    Query stored results with methods u, y, x.
    """

    def __init__(self, n_x, n_u, x0=None, dt=1):
        # self.A = A
        # self.B = B
        # self.C = C

        self.n_x = n_x
        self.n_u = n_u
        self.n_y = n_x

        # if D is None:
        #     D = np.zeros((self.n_y, self.n_u))

        # self.D = D
        self.R = 1
        self.L = 1
        self.I = 1
        self.masa = 10
        

        if x0 is None:
            x0 = np.zeros((self.n_x, 1),dtype=float)

        self.x0 = x0
        
        self._x = []
        self._u = []
        self._y = []

        self.dt = dt
        self.t_now = 0
        self._time = []

    def debug(self):
        resultado = self.x0[0] + self.dt*np.cos(self.x0[2])*self.x0[3] 
        return resultado


    def make_step(self, u):
        """
        Run a simulation step by passing the current input.
        Returns the current measurement y.
        """

        self._x.append(self.x0)
        self._u.append(u)
        
        y0 = np.zeros((5,1))
        x0 = self.x0[:,-1]
        y0[2] = x0[2] + x0[4]                                          #theta vel
        y0[0] = x0[0] + self.dt*np.cos(x0[2]) * x0[3]                       #x vel
        y0[1] = x0[1] + self.dt*np.sin(x0[2])*x0[3]                       #y vel
        y0[3] = x0[3] + self.dt*self.R/(2*self.masa)*(u[0] + u[1])   #lineal speed
        y0[4] = x0[4] + self.dt*self.R/(self.L*self.I)*(u[0] - u[1])      #angular speed
        
        # y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))
        # matrix_trans = np.array( self.dt* [ [np.cos(x0[2]), 0], [np.sin(x0[2]), 0], [0, 1]] , dtype=float)
        # print("matrix", matrix_trans)
        # x_fut = self.dt*np.dot(matrix_trans,u)
        # print("x_fut", x_fut)
        # y = self.x0 + x_fut
        # print("y", y)
        self._y.append(y0)
        self._time.append(self.t_now)
        self.x0 = y0
        self.t_now += self.dt

        return y0

    def reset(self, x0=None):
        if x0 is not None:
            self.x0 = x0
        else:
            self.x0 = np.zeros((self.n_x, 1))

        self._x = []
        self._u = []
        self._y = []
        self._time = []
        self.t_now = 0

    @property
    def x(self):
        return np.concatenate(self._x, axis=1).T

    @x.setter
    def x(self, *args):
        raise Exception('Cannot set x directly.')

    @property
    def u(self):
        return np.concatenate(self._u, axis=1).T

    @u.setter
    def u(self, *args):
        self._u = args
        # raise Exception('Cannot set u directly.')

    @property
    def y(self):
        return np.concatenate(self._y, axis=1).T

    @y.setter
    def y(self, *args):
        raise Exception('Cannot set y directly.')

    @property
    def time(self):
        return np.array(self._time)

    @time.setter
    def time(self, *args):
        raise Exception('Cannot set time directly.')


def random_u(u0, switch_prob=0.5, max_amp=np.pi):
    # Hold the current value with 80% chance or switch to new random value.
    # New candidate value.
    u_next = (0.5-np.random.rand(u0.shape[0], 1))*max_amp
    switch = np.random.rand() >= (1-switch_prob)  # switching? 0 or 1.
    u0 = (1-switch)*u0 + switch*u_next  # Old or new value.
    return u0
