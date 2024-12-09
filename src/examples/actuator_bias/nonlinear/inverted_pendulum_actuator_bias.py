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

    # actuator bias attack set
    bias_magnitude = np.array([0]) 
    bias_start = 300  
    bias_end = 600

    import sys
    import os
    # Dynamically add cpsim path to sys.path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    cpsim_dir = os.path.abspath(os.path.join(current_dir, '../../../cpsim'))
    sys.path.insert(0, cpsim_dir)

    from actuator_attack import ActuatorAttack
    actuator_bias_attack = ActuatorAttack('bias', 
                                          bias_magnitude, 
                                          bias_start, 
                                          bias_end)

    ip = InvertedPendulum('test', dt, max_index)

    for i in range(0, max_index + 1):
        assert ip.cur_index == i
        ip.update_current_ref(ref[i])

        u0 = ip.controller.update(
            ip.cur_ref, 
            ip.cur_feedback, 
            ip.dt * ip.cur_index
        )

        # attack here
        u1 = actuator_bias_attack.launch(u0, ip.cur_index, ip.states)        

        ip.evolve(u=u1)





    # print results
    import matplotlib.pyplot as plt

    import os
    current_dir = os.path.dirname(__file__)
    results_dir = os.path.join(current_dir, '..', 'results')
    os.makedirs(results_dir, exist_ok=True)

    t_arr = np.linspace(0, max_index * dt, max_index + 1)
    x_arr = [x[0] for x in ip.outputs[:max_index + 1]]  # Cart Position
    theta_arr = [x[2] for x in ip.outputs[:max_index + 1]]  # Pendulum Angle
    u_arr = [x[0] for x in ip.inputs[:max_index + 1]]  # Control Input


    fig, axes = plt.subplots(3, 1, figsize=(5,6), sharex=True)

    # axes[0].set_title(f'Cart Position with Bias Attack (Bias = {bias_magnitude[0]})')
    axes[0].set_title(f'Cart Position')

    axes[0].plot(t_arr, x_arr, label='Cart Position (x)')
    axes[0].axhline(ref[0][0], color='red', linestyle='--', label='Reference Position')
    
    # axes[0].scatter(t_arr[bias_start], x_arr[bias_start], color='red', label='Bias Start', zorder=5)
    # axes[0].scatter(t_arr[bias_end], x_arr[bias_end], color='orange', label='Bias End', zorder=5)
    
    axes[0].set_ylabel('Position')
    axes[0].legend()
    axes[0].grid()

    # axes[1].set_title(f'Pendulum Angle with Bias Attack (Bias = {bias_magnitude[0]})')
    axes[1].set_title(f'Pendulum Angle')

    axes[1].plot(t_arr, np.rad2deg(theta_arr), label='Pendulum Angle')
    axes[1].axhline(np.rad2deg(ref[0][2]), color='red', linestyle='--', label='Reference Angle')
    
    # axes[1].scatter(t_arr[bias_start], np.rad2deg(theta_arr[bias_start]), color='red', label='Bias Start', zorder=5)
    # axes[1].scatter(t_arr[bias_end], np.rad2deg(theta_arr[bias_end]), color='orange', label='Bias End', zorder=5)
    
    axes[1].set_ylabel('Angle')
    axes[1].legend()
    axes[1].grid()

    # axes[2].set_title(f'Control Input (u-Force) with Bias Attack (Bias = {bias_magnitude[0]})')
    axes[2].set_title(f'Control Input (u-Force)')

    axes[2].plot(t_arr, u_arr, label='Control Input')
    
    # axes[2].scatter(t_arr[bias_start], u_arr[bias_start], color='red', label='Bias Start', zorder=5)
    # axes[2].scatter(t_arr[bias_end], u_arr[bias_end], color='orange', label='Bias End', zorder=5)
    
    axes[2].set_xlabel('Time steps')
    axes[2].set_ylabel('u Force')
    axes[2].legend()
    axes[2].grid()

    plt.tight_layout()

    plt.savefig(os.path.join(results_dir, 'pendulum_combined_bias_attack.png'))

    plt.show()




    # fig, ax = plt.subplots(3, 1)
    # ax1, ax2, ax3 = ax
    # t_arr = np.linspace(0, 10, max_index + 1)

    # ref1 = [x[0] for x in ip.refs[:max_index + 1]]
    # y1_arr = [x[0] for x in ip.outputs[:max_index + 1]]
    # ax1.set_title('x0-location')
    # ax1.plot(t_arr, y1_arr, t_arr, ref1)

    # ref2 = [x[2] for x in ip.refs[:max_index + 1]]
    # y2_arr = [x[2] for x in ip.outputs[:max_index + 1]]
    # ax2.set_title('x2-angle')
    # ax2.plot(t_arr, y2_arr, t_arr, ref2)

    # u_arr = [x[0] for x in ip.inputs[:max_index + 1]]
    # ax3.set_title('u-force')
    # ax3.plot(t_arr, u_arr)

    # print(u_arr)

    # plt.show()
