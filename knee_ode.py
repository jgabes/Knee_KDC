import numpy as np
import scipy.integrate as spi
import matplotlib.pyplot as plt
import random as rand
from pykalman import KalmanFilter

# constants
STATE_SHAPE = (6,)
SOME_COEFFICIENT = 0.9
BOWDEN_STIFFNESS = 2600

BOWDEN_DAMPING = 1


class knee_object(object):
    def __init__(self):
        self._stuff = SOME_COEFFICIENT
        self._stiff = BOWDEN_STIFFNESS
        self._damp = BOWDEN_DAMPING
        self._force = 0.
        self._force2 = 0.
        self._f_hist = [0, 0, 0]
        self._f2_hist = [0, 0, 0]
        self._desired_velocity = 0.
        self._desired_velocity2 = 0.
        self._desired_force = 0.
        self._desired_force2 = 0.
        self._kp = 0.015
        self._kd = 0.01
        self._df = 0.
        self._df2 = 0.
        self._follow_distance = .001

        self._kp_avoid = 9;
        self._kd_avoid = .6;

        self._rand_noise = 0
        self._mass = 10

        self._t = 0
        self._disturb = 0

        self._k_torque = 250

        self._equilib = 0

    def check_rand(self):
        sample = rand.random()
        if self._rand_noise is 0:
            if sample > 0.999:
                self._rand_noise = rand.random() * 10 - 5
                print 'noise on'
        else:
            if sample >= .99:
                self._rand_noise = 0
                print 'noise off'
        return self._rand_noise

    def pcontrol(self):
        self._desired_velocity = self._kp * (self._desired_force - self._force)
        self._desired_velocity2 = self._kp * (self._desired_force2 - self._force2)
        # self._desired_velocity2=0

    def pdcontrol(self):
        self._desired_velocity = self._kp * (self._desired_force - self._force) - self._kd * self._df
        self._desired_velocity2 = (self._kp * (self._desired_force2 - self._force2) - self._kd * self._df2)
        # self._desired_velocity2=0

    def pdcontrol_extend_active(self, state):
        self._desired_velocity = self._kp * (self._desired_force - self._force) - self._kd * self._df
        self._desired_velocity2 = (
        self._kp_avoid * ((state[2] + self._follow_distance) - state[4]) + self._kd_avoid * state[5])

    def pdcontrol_flex_active(self, state):
        self._desired_velocity = (
        self._kp_avoid * ((state[2] - self._follow_distance) - state[0]) + self._kd_avoid * state[2])
        self._desired_velocity2 = self._kp * (self._desired_force2 - self._force2) - self._kd * self._df2

    def pd_handoff_control(self, state):
        final_force = self._k_torque * (self._equilib - state[2])

    def observe_f(self):
        self._df = (self._force - self._f_hist[0]) / 3
        self._df2 = (self._force2 - self._f2_hist[0]) / 3
        self._f_hist.pop(0)
        self._f2_hist.pop(0)
        self._f_hist.append(self._force)
        self._f2_hist.append(self._force2)

    @property
    def t(self):
        return self._t

    @t.setter
    def t(self, t_in):
        self._t = t_in

    @property
    def disturb(self):
        return self._disturb

    @disturb.setter
    def disturb(self, d_in):
        self._disturb = d_in

    @property
    def stiff(self):
        return self._stiff

    @property
    def mass(self):
        return self._mass

    @property
    def damp(self):
        return self._damp

    @property
    def stuff(self):
        return self._stuff

    @stuff.setter
    def stuff(self, stuff_in):
        self._stuff = stuff_in

    @property
    def force(self):
        return self._force

    @property
    def force2(self):
        return self._force2

    @force.setter
    def force(self, force_in):
        self._force = force_in

    @force2.setter
    def force2(self, force2_in):
        self._force2 = force2_in

    @property
    def desired_velocity(self):
        return self._desired_velocity

    @desired_velocity.setter
    def desired_velocity(self, vel_in):
        self._desired_velocity = vel_in

    @property
    def desired_velocity2(self):
        return self._desired_velocity2

    @desired_velocity.setter
    def desired_velocity2(self, vel2_in):
        self._desired_velocity2 = vel2_in

    @property
    def desired_force(self):
        return self._desired_force

    @desired_force.setter
    def desired_force(self, force_in):
        self._desired_force = force_in

    @property
    def desired_force2(self):
        return self._desired_force2

    @desired_force.setter
    def desired_force2(self, force2_in):
        self._desired_force2 = force2_in

    @property
    def kp(self):
        return self._kp

    @kp.setter
    def kp(self, kp_in):
        self._kp = kp_in

    @property
    def kd(self):
        return self._kd

    @kd.setter
    def kd(self, kd_in):
        self._kd = kd_in

    @property
    def df(self):
        return self.df

    @kd.setter
    def df(self, df_in):
        self._df = df_in


def ode(state, t, knee_class):
    """
    Function that you feed into ODE integrationst
    :param state: 1-D numpy array of the current state
    :param t: current time (time independent ODE's ignore this)
    :param knee_class: any other stuff you need, including actuation forces, parameters and constants, etc.
    :return: a 1-D numpy array of the current state derivative
    """
    m = knee_class.mass
    f1 = knee_class.force
    f2 = knee_class.force2

    if state[0] < state[2]:
        f1 = 0.

    if state[2] < state[4]:
        f2 = 0.

    dstate_dt = np.zeros(STATE_SHAPE)
    # a = useful_class.stuff
    # useful_class.pdcontrol()
    # des_vel=useful_class.desired_velocity
    # des_vel2=useful_class.desired_velocity
    input_disturbance = 25 * np.sin(2 * np.pi * knee.t)
    knee.disturb = input_disturbance
    dstate_dt[0] = state[1]  # dx/dt = xdot
    dstate_dt[1] = 0  # #input
    dstate_dt[2] = state[3]  # dx/dt of weight
    # dstate_dt[3] = (f1-f2+useful_class.check_rand())/m
    dstate_dt[3] = (f1 + f2 + input_disturbance) / m  # accel of weight
    dstate_dt[4] = state[5]  # vel oc phi2
    dstate_dt[5] = 0  # des_vel2
    return dstate_dt


if __name__ == "__main__":
    # State: phi, phi_dot, x, xdot, phi2, phi_dot2

    initial_state = [0., 0., 0., 0., 0., 0.]
    state = np.array(initial_state)
    t = 0.
    dt = 0.001
    knee = knee_object()
    knee.desired_force = 0
    knee._desired_force2 = 0
    time_history = [0.]
    force_history = [0.]
    force_history = [knee.force]
    force2_history = [0.]
    force2_history = [knee.force2]
    state_history = [initial_state]
    des_f_history = [0.]
    des_f2_history = [0.]

    while t < 10:
        for _ in range(10):
            times = np.array([t, t + dt])
            states = spi.odeint(ode, state, times, (knee,))
            state = states[-1]
            t += dt
        knee.t = t
        state_history.append([state[0], state[1], state[2], state[3], state[4], state[5]])
        time_history.append(t)
        knee.force = (state[0] - state[2]) * knee.stiff + (state[1] - state[3]) * knee.damp
        knee.force2 = -((state[2] - state[4]) * knee.stiff + (state[3] - state[5])) * knee.damp
        knee.observe_f()

        goal_pos = -.05
        if state[2] < goal_pos:
            des_force = 250 * (goal_pos - state[2])
            des_force2 = 0.
        else:
            des_force = 0.
            des_force2 = 250 * (goal_pos - state[2])

        knee.desired_force = des_force
        knee.desired_force2 = des_force2

        if state[0] < state[2]:
            force_history.append(0.)
        else:
            force_history.append(knee.force)

        if state[2] < state[4]:
            force2_history.append(0.)
        else:
            force2_history.append(knee.force2)

        des_f_history.append(des_force)
        des_f2_history.append(des_force2)
        # print "state at t = {:.2f}:  x = {:.3f}, {:.3f}, {:.3f}, f={:.3f}, {:.3f}".format(t, state[0], state[2], state[4],my_thing.force, my_thing.force2)

        # knee.pdcontrol()

        knee.pdcontrol_extend_active(state)

        knee.pdcontrol_flex_active(state)

        state[1] = knee._desired_velocity
        state[5] = knee._desired_velocity2

        if t > 3:
            z = 1
    # End SImulation while loop

    state_history = np.array(state_history)
    force_history = np.array(force_history)
    force2_history = np.array(force2_history)
    des_f_history = np.array(des_f_history)
    des_f2_history = np.array(des_f2_history)
    plt.subplot(3, 1, 1)
    x1_line, = plt.plot(time_history, state_history[:, 0], 'r-', linewidth=4.0, label="X1")
    x2_line, = plt.plot(time_history, state_history[:, 2], 'g-', linewidth=4.0, label="X2")
    x3_line, = plt.plot(time_history, state_history[:, 4], 'b-', linewidth=4.0, label="X3")
    #    plt.legend(handles=(x1_line, x2_line, x3_line))

    plt.ylabel('Positions (m)')

    plt.subplot(3, 1, 2)
    xdot1_line, = plt.plot(time_history, state_history[:, 1], 'r-', linewidth=4.0, label="X1_dot")
    xdot2_line, = plt.plot(time_history, state_history[:, 3], 'g-', linewidth=4.0, label="X2_dot")
    xdot3_line, = plt.plot(time_history, state_history[:, 5], 'b-', linewidth=4.0, label="X3_dot")

    plt.ylabel('Velocities (m/s)')

    # f_line, = plt.plot(time_history, force_history, 'g-', linewidth=4.0, label="f")


    plt.subplot(3, 1, 3)
    f1_line, = plt.plot(time_history, force_history, 'r-', linewidth=4.0, label="Force")
    f2_line, = plt.plot(time_history, force2_history, 'b-', linewidth=4.0, label="Force")
    f_input, = plt.plot(time_history, 25 * np.sin(2 * np.pi * np.array(time_history)), 'g--', linewidth=4.0,
                        label="desired")

    f_des_line = plt.plot(time_history, des_f_history, 'r--', linewidth=4.0, label="Des Force1")
    f2_des_line = plt.plot(time_history, des_f2_history, 'b--', linewidth=4.0, label="Des Force2")

    plt.ylabel('Force (N)')
    plt.xlabel('Time (s)')
    plt.show()
