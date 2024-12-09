import numpy as np

from cpsim import Attack
from cpsim.models.nonlinear.continuous_stirred_tank_reactor import CSTR, cstr_imath


class cstr_bias:
    name = 'cstr_bias'  # benchmark name
    max_index = 180  # simulation time period
    ref = [np.array([0.98189, 300.00013])] * (max_index + 1)  # reference state
    dt = 0.1  # sampling time
    noise = {
        'process': {  # process/sensor disturbance
            'type': 'box_uniform',
            'param': {'lo': np.array([0, 0]), 'up': np.array([0.15, 0.15])}  # the intensity of disturbance
        }
    }
    # noise = None
    model = CSTR(name, dt, max_index, noise=noise)  # benchmark model  (defined in simulators folder)
    ode_imath = cstr_imath  # model for interval arithmetic

    attack_start_index = 90  # index when attack starts
    recovery_index = 100  # index when recovery starts
    bias = np.array([0, -30])  # bias sensor attack intensity
    unsafe_states_onehot = [0, 1]  # indicate which state is unsafe / affected by attack

    attack = Attack('bias', bias, attack_start_index)  # attack type and intensity

    output_index = 1  # index in state to be plotted
    ref_index = 1  # index in state to be plotted

    safe_set_lo = np.array([-5, 250])  # safe set lower bound
    safe_set_up = np.array([5, 360])  # safe set upper bound
    target_set_lo = np.array([-5, 299])  # target set lower bound
    target_set_up = np.array([5, 301])  # target set upper bound
    control_lo = np.array([200])  # control lower bound
    control_up = np.array([300])  # control upper bound
    recovery_ref = np.array([0.98189, 300.00013])  # recovery target state

    # Q = np.diag([1, 1])
    # QN = np.diag([1, 1])
    Q = np.diag([1, 1000])  # state cost for LQR or MPC
    QN = np.diag([1, 1000])  # final state cost for LQR or MPC
    R = np.diag([1])  # control cost for LQR or MPC

    MPC_freq = 1  # optimization frequency
    nx = 2  # state dimension
    nu = 1  # control dimension

    # plot
    y_lim = (280, 360)  # y axis range to be plotted
    x_lim = (8, dt * 200)  # x axis range to be plotted
    strip = (target_set_lo[output_index], target_set_up[output_index])  # strip to be plotted
    y_label = 'Temperature [K]'  # y axis label

    # for linearizations for baselines, find equilibrium point and use below
    u_ss = np.array([274.57786])  # equilibrium control  for baseline only
    x_ss = np.array([0.98472896, 300.00335862])  # equilibrium state for baseline only