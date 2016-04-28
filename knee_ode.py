import numpy as np
import scipy.integrate as spi
import matplotlib.pyplot as plt

# constants
STATE_SHAPE = (6,)
SOME_COEFFICIENT = 0.9
BOWDEN_STIFFNESS = 1000
BOWDEN_DAMPING=.1

class some_class(object):
    def __init__(self):
        self._stuff = SOME_COEFFICIENT
        self._stiff = BOWDEN_STIFFNESS
        self._damp = BOWDEN_DAMPING
        self._force = 0.
        self._force2 = 0.
        self._f_hist=[0,0,0]
        self._f2_hist=[0,0,0]
        self._desired_velocity = 0.
        self._desired_velocity2 = 0.
        self._desired_force = 0.
        self._desired_force2 = 0.
        self._kp = .04
        self._kd = 0.3
        self._df = 0.
        self._df2 = 0.


    def pcontrol(self):
        self._desired_velocity=self._kp*(self._desired_force-self._force)
        #self._desired_velocity2=self._kp*(self._desired_force2-self._force2)
        self._desired_velocity2=0
    def pdcontrol(self):
        self._desired_velocity=self._kp*(self._desired_force-self._force)-self._kd*self._df
        #self._desired_velocity2=self._kp*(self._desired_force2-self._force2)-self._kd*self._df2
        self._desired_velocity2=0
    def observe_f(self):
        self._df=(self._force-self._f_hist[0])/3
        self._df=(self._force2-self._f2_hist[0])/3
        self._f_hist.pop(0)
        self._f2_hist.pop(0)
        self._f_hist.append(self._force)
        self._f2_hist.append(self._force2)




    @property
    def stiff(self):
        return self._stiff

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

    @property
    def df(self):
        return self.df
    @kd.setter
    def df(self, df_in):
        self._df=df_in

def ode(state, t, useful_class):
    """
    Function that you feed into ODE integrationst
    :param state: 1-D numpy array of the current state
    :param t: current time (time independent ODE's ignore this)
    :param useful_class: any other stuff you need, including actuation forces, parameters and constants, etc.
    :return: a 1-D numpy array of the current state derivative
    """
    m=100
    f1=useful_class.force
    f2=state[2]*useful_class.stiff+state[3]*useful_class.damp

    if state[0]<state[2]:
        f1=0


    dstate_dt = np.zeros(STATE_SHAPE)
    a = useful_class.stuff
    useful_class.pdcontrol()
    #des_vel=useful_class.desired_velocity
    #des_vel2=useful_class.desired_velocity
    dstate_dt[0] = state[1]  # dx/dt = xdot
    dstate_dt[1] = 0 # #input
    dstate_dt[2] = state[3]
    dstate_dt[3] = (f1-f2)/m
    dstate_dt[4] = state[5]
    dstate_dt[5] = 0 #des_vel2
    return dstate_dt


if __name__ == "__main__":
    #State: phi, phi_dot, x, xdot
    initial_state = [0., 0., 0., 0., 0., 0.]
    state = np.array(initial_state)
    t = 0.
    dt = 0.01
    my_thing = some_class()
    my_thing.desired_force=0
    my_thing._desired_force2=1
    time_history = [0.]
    force_history=[0.]
    force_history=[my_thing.force]
    force2_history=[0.]
    state_history = [initial_state]
    des_f_history=[0.]
    des_f2_history=[0.]

    while t < 10:
        times = np.array([t, t + dt])
        states = spi.odeint(ode, state, times, (my_thing,))
        state = states[-1]
        state_history.append([state[0], state[1], state[2], state[3], state[4], state[5]])
        t += dt
        time_history.append(t)
        my_thing.force=(state[0]-state[2])*my_thing.stiff+(state[1]-state[3])*my_thing.damp
        my_thing.observe_f()
        des_force=5*np.sin(2*np.pi*t)
        des_force2=0
        my_thing._desired_force=des_force
        if state[0]<state[2]:
            force_history.append(0)
        else:
            force_history.append(my_thing.force)
        des_f_history.append(des_force)
        des_f2_history.append(des_force2)
        force2_history.append(-(state[2]*my_thing.stiff+state[3]*my_thing.damp))
        print "state at t = {:.2f}:  x = {:.3f}, {:.3f}, f={:.3f}, {:.3f}".format(t, state[0], state[1],my_thing.force, my_thing.force2)
        my_thing.pdcontrol()
        state[1]=my_thing._desired_velocity
        state[3]=my_thing._desired_velocity2



    state_history = np.array(state_history)
    force_history=np.array(force_history)
    force2_history=np.array(force2_history)
    des_f_history=np.array(des_f_history)
    des_f2_history=np.array(des_f2_history)
    plt.subplot(3,1,1)
    x1_line, = plt.plot(time_history, state_history[:, 0], 'r-', linewidth=4.0, label="X1")
    x2_line, = plt.plot(time_history, state_history[:, 2], 'g-', linewidth=4.0, label="X2")
    x3_line, = plt.plot(time_history, state_history[:, 4], 'b-', linewidth=4.0, label="X3")

    #plt.legend(handles=(x1_line, x2_line))
    plt.ylabel('Positions (m)')

    plt.subplot(3,1,2)
    xdot1_line, = plt.plot(time_history, state_history[:, 1], 'r-', linewidth=4.0, label="X1_dot")
    xdot2_line, = plt.plot(time_history, state_history[:, 3], 'g-', linewidth=4.0, label="X2_dot")
    xdot3_line, = plt.plot(time_history, state_history[:, 5], 'b-', linewidth=4.0, label="X3_dot")

    plt.ylabel('Velocities (m/s)')

   # f_line, = plt.plot(time_history, force_history, 'g-', linewidth=4.0, label="f")


    plt.subplot(3,1,3)
    f1_line, = plt.plot(time_history, force_history, 'r-', linewidth=4.0, label="Force")
    f2_line, = plt.plot(time_history, force2_history, 'b-', linewidth=4.0, label="Force")


    f_des_line= plt.plot(time_history,des_f_history, '--', linewidth=4.0, label="Des Force")
    plt.ylabel('Force (N)')
    plt.xlabel('Time (s)')
    plt.show()