# Ref: Data-Driven Science and Engineering: Machine Learning, Dynamical Systems, and Control (Session 8.7, Page 300)

import numpy as np

from cpsim import Simulator
from cpsim.controllers.LQR import LQR

# parameters
m = 1  # mass of rob
M = 5  # mass of cart
L = 2  # length of rob
g = -10
d = 1  # dumping (friction)
b = 1  # pendulum up (b=1)


def inverted_pendulum(t, x, u, use_imath=False):
    u=u[0]
    if use_imath:
        from interval import imath
        Sx = imath.sin(x[2])
        Cx = imath.cos(x[2])
        D = m * L * L * (M + m * (1 - Cx * Cx))

        dx = [0,0,0,0]
        dx[0] = x[1]
        dx[1] = (1 / D) * (-m * m * L * L * g * Cx * Sx + m * L * L * (m * L * x[3] * x[3] * Sx - d * x[1])) + m * L * L * (
                1 / D) * u
        dx[2] = x[3]
        dx[3] = (1 / D) * ((m + M) * m * g * L * Sx - m * L * Cx * (m * L * x[3] * x[3] * Sx - d * x[1])) - m * L * Cx * (
                1 / D) * u
    else:
        Sx = np.sin(x[2])
        Cx = np.cos(x[2])
        D = m * L * L * (M + m * (1 - Cx * Cx))

        dx = np.zeros((4,))
        dx[0] = x[1]
        dx[1] = (1 / D) * (-m * m * L * L * g * Cx * Sx + m * L * L * (m * L * x[3] * x[3] * Sx - d * x[1])) + m * L * L * (
                1 / D) * u
        dx[2] = x[3]
        dx[3] = (1 / D) * ((m + M) * m * g * L * Sx - m * L * Cx * (m * L * x[3] * x[3] * Sx - d * x[1])) - m * L * Cx * (
                1 / D) * u
    return dx

def inverted_pendulum_imath(t, x, u):
    return inverted_pendulum(t=t,x=x,u=u, use_imath=True)

x_0 = np.array([-1, 0, np.pi + 0.1, 0])
control_limit = {
    'lo': np.array([-50]),
    'up': np.array([50])
}

# utils parameters
A = np.array([[0, 1, 0, 0],
              [0, -d / M, b * m * g / M, 0],
              [0, 0, 0, 1],
              [0, -b * d / (M * L), -b * (m + M) * g / (M * L), 0]])
B = np.array([[0], [1 / M], [0], [b * 1 / (M * L)]])

R = np.array([[0.0001]])
Q = np.eye(4)*10
Q[2,2] = 1000

class Controller:
    def __init__(self, dt, control_limit=None):
        self.lqr = LQR(A, B, Q, R)
        self.set_control_limit(control_limit['lo'], control_limit['up'])

    def update(self, ref: np.ndarray, feedback_value: np.ndarray, current_time) -> np.ndarray:
        self.lqr.set_reference(ref)
        cin = self.lqr.update(feedback_value, current_time)
        return cin
    def set_control_limit(self, control_lo, control_up):
        self.control_lo = control_lo
        self.control_up = control_up
        self.lqr.set_control_limit(self.control_lo[0], self.control_up[0])
    def clear(self):
        return


class InvertedPendulum(Simulator):
    """
    States: (4,)
        x[0]: location of cart
        x[1]: dx[0]
        x[2]: pendulum angle  (down:0, up:pi)
        x[3]: dx[1]
    Control Input: (1,)  [control_limit]
        u[0]: force on the cart
    Output: (4,)
        State Feedback
    Controller: LQR
    """

    def __init__(self, name, dt, max_index, noise=None):
        super().__init__('Inverted Pendulum ' + name, dt, max_index)
        self.nonlinear(ode=inverted_pendulum, n=4, m=1, p=4)
        controller = Controller(dt, control_limit)
        settings = {
            'init_state': x_0,
            'feedback_type': 'state',
            'controller': controller
        }
        if noise:
            settings['noise'] = noise
        self.sim_init(settings)


if __name__ == "__main__":
    max_index = 800
    dt = 0.02
    ref = [np.array([1, 0, np.pi, 0])] * (max_index+1)
    # bias attack example
    from cpsim.actuator_attack import ActuatorAttack

    params={'start': 300, 'end': 600}

    bias_attack = ActuatorAttack('replay', params, 80)

    ip = InvertedPendulum('test', dt, max_index)

    attack = True
    for i in range(0, max_index + 1):
        if (i > params['start'] and i < params['end']) and attack==True:
            assert ip.cur_index == i
            ip.update_current_ref(ref[i])
            ip.cur_feedback = bias_attack.launch(ip.cur_feedback, ip.cur_index, ip.states)
            ip.evolve()
        else:
            assert ip.cur_index == i
            ip.update_current_ref(ref[i])
            ip.evolve()

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(6,7))
    ax1, ax2, ax3 = ax
    
    t_arr = np.linspace(0, 800, max_index + 1)
    ref1 = [x[0] for x in ip.refs[:max_index + 1]]
    y1_arr = [x[0] for x in ip.outputs[:max_index + 1]]
    ref2 = [x[2] for x in ip.refs[:max_index + 1]]
    y2_arr = [x[2] for x in ip.outputs[:max_index + 1]]
    u_arr = [x[0] for x in ip.inputs[:max_index + 1]]

# ------------------------------------------------------------------------

    ax1.plot(t_arr, y1_arr, label="Cart Position (x)")
    ax1.plot(t_arr, ref1, label="Reference Position", color='red', linestyle='dashed')
    if attack:
        ax1.plot(t_arr[params['start']], y1_arr[params['start']], 'o', label='Replay Start', color='red')
        ax1.plot(t_arr[params['end']], y1_arr[params['end']], 'o', label='Replay End', color='orange')
        ax1.set_title('Cart Position with Replay Attack')
    else:
        ax1.set_title('Cart Position')
    ax1.grid()
    ax1.legend()
    ax1.set_ylabel('Position (m)')

# ------------------------------------------------------------------------

    ax2.plot(t_arr, y2_arr, label='Pendulum Angle')
    ax2.plot(t_arr, ref2, label='Pendulum Angle', color='red', linestyle='dashed')
    if attack:
        ax2.plot(t_arr[params['start']], y2_arr[params['start']], 'o', label='Replay Start', color='red')
        ax2.plot(t_arr[params['end']], y2_arr[params['end']], 'o', label='Replay End', color='orange')
        ax2.set_title('Pendulum Angle with Replay Attack')
    else:
        ax2.set_title('Pendulum Angle')
    ax2.set_ylabel('Angle (degrees)')
    ax2.grid()
    ax2.legend()
    
# ------------------------------------------------------------------------

    ax3.plot(t_arr, u_arr, label='Control Input')
    if attack:
        ax3.plot(t_arr[params['start']], u_arr[params['start']], 'o', label='Replay Start', color='red')
        ax3.plot(t_arr[params['end']], u_arr[params['end']], 'o', label='Replay End', color='orange')
        ax3.set_title('Control Input (u-Force) with Replay Attack')
    else:
        ax3.set_title('Control Input (u-Force)')
    ax3.set_xlabel('Times (s)')
    ax3.set_ylabel('Force (N)')
    ax3.grid()
    ax3.legend()
    plt.tight_layout()
    if attack:
        plt.savefig('./figs/nonlinear/inverted_pendulum_replay.png', dpi=800)
    else:
        plt.savefig('./figs/nonlinear/inverted_pendulum_replay (no attack).png', dpi=800)
    plt.show()