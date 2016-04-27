import numpy as np
import scipy.integrate as spi
import matplotlib.pyplot as plt

# constants
STATE_SHAPE = (2,)
SOME_COEFFICIENT = 0.9
BOWDEN_STIFFNESS = 100000

class some_class(object):
    def __init__(self):
        self._stuff = SOME_COEFFICIENT
        self._stiff = BOWDEN_STIFFNESS
        self._force = 0.
        self._desired_velocity=0.
        self._desired_force=0.
        self._kp=5.
        self._kd=.1


    def pcontrol(self):
        self._desired_velocity=self.kp*(self._desired_force-self._desired_force)

    def pdcontrol(self,vel):
        self._desired_velocity=self.kp*(self._desired_force-self._desired_force)+self._kd*vel

    @property
    def stiff(self):
        return self._stiff

    @property
    def stuff(self):
        return self._stuff
    @stuff.setter
    def stuff(self, stuff_in):
        self._stuff = stuff_in

    @property
    def force(self):
        return self._force
    @force.setter
    def force(self, force_in):
        self._force=force_in

    @property
    def desired_velocity(self):
        return self._desired_velocity
    @desired_velocity.setter
    def desired_velocity(self,vel_in):
        self._desired_velocity=vel_in

    @property
    def desired_force(self):
        return self._desired_force
    @desired_force.setter
    def desired_force(self,force_in):
        self._desired_force=force_in

    @property
    def kp(self):
        return self._kp
    @kp.setter
    def kp(self, kp_in):
        self._kp=kp_in

    @property
    def kd(self):
        return self._kd
    @kd.setter
    def kd(self, kd_in):
        self._kd=kd_in

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
    useful_class.pcontrol()
    des_vel=useful_class.desired_velocity
    dstate_dt[0] = state[1]  # dx/dt = xdot
    dstate_dt[1] =des_vel #
    return dstate_dt


if __name__ == "__main__":
    #State: phi, f
    initial_state = [0.,  1.]
    state = np.array(initial_state)
    t = 0.
    dt = 0.01
    my_thing = some_class()
    time_history = [0.]
    force_history=[0.]
    corce_history=[my_thing.force]
    state_history = [initial_state]

    while t < 10:
        times = np.array([t, t + dt])
        states = spi.odeint(ode, state, times, (my_thing,))
        state = states[-1]
        state_history.append([state[0], state[1]])
        t += dt
        time_history.append(t)
        my_thing.force=(state[0]*my_thing.stiff)
        force_history.append(my_thing.force)
        print "state at t = {:.2f}:  x = {:.3f}, {:.3f}".format(t, state[0], state[1])
    state_history = np.array(state_history)
    force_history=np.array(force_history)
    x_line, = plt.plot(time_history, state_history[:, 0], 'b-', linewidth=4.0, label="X")
    xdot_line, = plt.plot(time_history, state_history[:, 1], 'c-', linewidth=4.0, label="X_dot")
   # plt.legend(handles=(x_line, xdot_line))
    plt.show()

    f_line, = plt.plot(time_history, force_history[:, 0], 'g-', linewidth=4.0, label="Force")
    plt.show()