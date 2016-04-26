import numpy as np
import scipy.integrate as spi
import matplotlib.pyplot as plt

# constants
STATE_SHAPE = (2,)
SOME_COEFFICIENT = 0.9


class some_class(object):
    def __init__(self):
        self._stuff = SOME_COEFFICIENT
    @property
    def stuff(self):
        return self._stuff
    @stuff.setter
    def stuff(self, stuff_in):
        self._stuff = stuff_in


def ode(state, t, useful_class):
    """
    Function that you feed into ODE integration
    :param state: 1-D numpy array of the current state
    :param t: current time (time independent ODE's ignore this)
    :param useful_class: any other stuff you need, including actuation forces, parameters and constants, etc.
    :return: a 1-D numpy array of the current state derivative
    """
    dstate_dt = np.zeros(STATE_SHAPE)
    a = useful_class.stuff
    dstate_dt[0] = state[1]  # dx/dt = xdot
    dstate_dt[1] = -a*state[0]  # dx^2/dt^2 = -a*x
    return dstate_dt


if __name__ == "__main__":
    initial_state = [0., 10.0]
    state = np.array(initial_state)
    t = 0.
    dt = 0.01
    my_thing = some_class()
    time_history = [0.]
    state_history = [initial_state]

    while t < 10.:
        times = np.array([t, t + dt])
        states = spi.odeint(ode, state, times, (my_thing,))
        state = states[-1]
        state_history.append([state[0], state[1]])
        t += dt
        time_history.append(t)
        print "state at t = {:.2f}:  x = {:.3f}, {:.3f}".format(t, state[0], state[1])
    state_history = np.array(state_history)
    x_line, = plt.plot(time_history, state_history[:, 0], 'b-', linewidth=4.0, label="X")
    xdot_line, = plt.plot(time_history, state_history[:, 1], 'c-', linewidth=4.0, label="X_dot")
    plt.legend(handles=(x_line, xdot_line))
    plt.show()