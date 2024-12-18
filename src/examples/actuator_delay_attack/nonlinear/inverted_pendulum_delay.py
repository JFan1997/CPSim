# Ref: Data-Driven Science and Engineering: Machine Learning, Dynamical Systems, and Control (Session 8.7, Page 300)

import numpy as np
import matplotlib.pyplot as plt
from cpsim import Simulator
from cpsim.actuator_attack import ActuatorAttack
from cpsim.controllers.LQR import LQR

# parameters
m = 1  # mass of rob
M = 5  # mass of cart
L = 2  # length of rob
g = -10
d = 1  # dumping (friction)
b = 1  # pendulum up (b=1)


def inverted_pendulum(t, x, u, use_imath=False):
    u = u[0]
    if use_imath:
        from interval import imath
        Sx = imath.sin(x[2])
        Cx = imath.cos(x[2])
        D = m * L * L * (M + m * (1 - Cx * Cx))

        dx = [0, 0, 0, 0]
        dx[0] = x[1]
        dx[1] = (1 / D) * (
                -m * m * L * L * g * Cx * Sx + m * L * L * (m * L * x[3] * x[3] * Sx - d * x[1])) + m * L * L * (
                        1 / D) * u
        dx[2] = x[3]
        dx[3] = (1 / D) * (
                (m + M) * m * g * L * Sx - m * L * Cx * (m * L * x[3] * x[3] * Sx - d * x[1])) - m * L * Cx * (
                        1 / D) * u
    else:
        Sx = np.sin(x[2])
        Cx = np.cos(x[2])
        D = m * L * L * (M + m * (1 - Cx * Cx))

        dx = np.zeros((4,))
        dx[0] = x[1]
        dx[1] = (1 / D) * (
                -m * m * L * L * g * Cx * Sx + m * L * L * (m * L * x[3] * x[3] * Sx - d * x[1])) + m * L * L * (
                        1 / D) * u
        dx[2] = x[3]
        dx[3] = (1 / D) * (
                (m + M) * m * g * L * Sx - m * L * Cx * (m * L * x[3] * x[3] * Sx - d * x[1])) - m * L * Cx * (
                        1 / D) * u
    return dx


def inverted_pendulum_imath(t, x, u):
    return inverted_pendulum(t=t, x=x, u=u, use_imath=True)


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
Q = np.eye(4) * 10
Q[2, 2] = 1000


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

    # ip = InvertedPendulum('test', dt, max_index)
    # for i in range(0, max_index + 1):
    #     assert ip.cur_index == i
    #     ip.update_current_ref(ref[i])
    #     # attack here
    #     # ip.cur_feedback = bias_attack.launch(ip.cur_feedback, ip.cur_index, ip.states)
    #     ip.evolve()
if __name__ == "__main__":
    max_index = 800
    dt = 0.02
    ref = [np.array([1, 0, np.pi, 0])] * (max_index + 1)
    # bias attack example
    sim = InvertedPendulum('test', dt, max_index)
    attack = False
    start_index = 150
    delay_attack = ActuatorAttack('delay', 10, start_index)
    for i in range(0, max_index + 1):
        assert sim.cur_index == i
        sim.update_current_ref(ref[i])
        if attack:
            u = delay_attack.launch(sim.cur_u, sim.cur_index, sim.inputs)
            print('this is u', u)
            sim.evolve(u=u)
        else:
            sim.evolve()

    ############## Generate figure ##################


    step_arr = np.arange(0, max_index + 1)

    ################################
    # Figure 1: x0-location
    ################################
    ref1 = [x[0] for x in sim.refs[:max_index + 1]]
    y1_arr = [x[0] for x in sim.outputs[:max_index + 1]]

    plt.figure()
    plt.plot(step_arr, y1_arr, label="Output x0")
    plt.plot(step_arr, ref1, label="Reference x0")
    plt.axvline(x=start_index, color='red', linestyle='--', label=f"Step {start_index}")
    plt.scatter(start_index, y1_arr[start_index], color='red', label="Marked Point")
    plt.legend()
    plt.title("x0-location")
    plt.xlabel("Step")
    plt.ylabel("Value")
    plt.grid()
    plt.tight_layout()
    plt.savefig(
        '../results/nonlinear/Pendulum/x0_{}.png'.format(
            'no_attack' if not attack else 'attack'))
    plt.show()

    ################################
    # Figure 2: x2-angle
    ################################
    ref2 = [x[2] for x in sim.refs[:max_index + 1]]
    y2_arr = [x[2] for x in sim.outputs[:max_index + 1]]

    plt.figure()
    plt.plot(step_arr, y2_arr, label="Output x2")
    plt.plot(step_arr, ref2, label="Reference x2")
    plt.axvline(x=start_index, color='red', linestyle='--', label=f"Step {start_index}")
    plt.scatter(start_index, y2_arr[start_index], color='red', label="Marked Point")
    plt.legend()
    plt.title("x2-angle")
    plt.xlabel("Step")
    plt.ylabel("Value")
    plt.grid()
    plt.tight_layout()
    plt.savefig(
        '../results/nonlinear/Pendulum/x2_{}.png'.format(
            'no_attack' if not attack else 'attack'))
    plt.show()

    ################################
    # Figure 3: u-force (input)
    ################################
    u_arr = [x[0] for x in sim.inputs[:max_index + 1]]
    plt.figure()
    plt.plot(step_arr, u_arr, label="Input u")
    plt.axvline(x=start_index, color='red', linestyle='--', label=f"Step {start_index}")
    plt.scatter(start_index, u_arr[start_index], color='red', label="Marked Point")
    plt.legend()
    plt.title("u-force")
    plt.xlabel("Step")
    plt.ylabel("Value")
    plt.grid()
    plt.tight_layout()
    plt.savefig(
        '../results/nonlinear/Pendulum/u_{}.png'.format(
            'no_attack' if not attack else 'attack'))
    plt.show()
