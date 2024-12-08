# Ref:https://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
import numpy as np

from cpsim import Simulator
from cpsim.actuator_attack import ActuatorAttack
from cpsim.controllers.PID import PID

# system dynamics
g = 9.81
m = 0.468
A = [[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, -g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
B = [[0], [0], [0], [0], [0], [0], [0], [0], [1 / m], [0], [0], [0]]
C = [[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]
D = [[0], [0], [0], [0], [0], [0]]

x_0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# utils parameters
control_limit = {
    'lo': np.array([-50]),
    'up': np.array([50])
}

KP = 15
KI = 0
KD = -4


class Controller:
    def __init__(self, dt):
        self.dt = dt
        self.pid = PID(KP, KI, KD, current_time=-dt)
        self.pid.setWindup(100)
        self.pid.setSampleTime(dt)
        self.set_control_limit(control_limit['lo'], control_limit['up'])

    def update(self, ref: np.ndarray, feedback_value: np.ndarray, current_time) -> np.ndarray:
        self.pid.set_reference(ref[0])
        cin = self.pid.update(feedback_value[5], current_time)
        return np.array([cin])

    def set_control_limit(self, control_lo, control_up):
        self.control_lo = control_lo
        self.control_up = control_up
        self.pid.set_control_limit(self.control_lo[0], self.control_up[0])

    def clear(self):
        self.pid.clear(current_time=-self.dt)


class Quadrotor(Simulator):
    """
               States: (12,)
                    x[0], x[1], x[2]: Roll, Pitch, Yaw
                    x[9], x[10], x[11]: relative position
                    x[11]: altitude
               Control Input: (4,)
                   u[0]: vertical thrust
                   u[1], u[2], u[3]:  one for each of the angular motions
               Output:  (6,)
                   y[5]: the altitude of the quadrotor
                   Output Feedback
               Controller: PID
               """

    def __init__(self, name, dt, max_index, noise=None):
        super().__init__('Quadrotor ' + name, dt, max_index)
        self.linear(A, B, C)
        controller = Controller(dt)
        settings = {
            'init_state': x_0,
            'feedback_type': 'output',
            'controller': controller
        }
        if noise:
            settings['noise'] = noise
        self.sim_init(settings)


# class quadrotor(Simulator):
#     """
#     States: (4,)
#         x[0]: location of cart
#         x[1]: dx[0]
#         x[2]: pendulum angle  (down:0, up:pi)
#         x[3]: dx[1]
#     Control Input: (1,)  [control_limit]
#         u[0]: force on the cart
#     Output: (4,)
#         State Feedback
#     Controller: LQR
#     """
#
#     def __init__(self, name, dt, max_index, noise=None):
#         super().__init__('Quadrotor ' + name, dt, max_index)
#         self.nonlinear(ode=quad, n=12, m=1, p=12)
#         controller = Controller(dt)
#         settings = {
#             'init_state': x_0,
#             'feedback_type': 'state',
#             'controller': controller
#         }
#         if noise:
#             settings['noise'] = noise
#         self.sim_init(settings)


if __name__ == "__main__":
    max_index = 600
    dt = 0.02
    ref = [np.array([2])] * 201 + [np.array([4])] * 200 + [np.array([2])] * 200
    noise = {
        'process': {
            'type': 'white',
            'param': {'C': np.eye(12) * 0.001}
        }
    }
    attack = True
    sim = Quadrotor('test', dt, max_index, None)
    start_index = 300
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

    import matplotlib.pyplot as plt

    # t_arr = np.linspace(0, 10, max_index + 1)
    step_arr = np.arange(0, max_index + 1)
    ref = [x[0] for x in sim.refs[:max_index + 1]]
    y_arr = [x[5] for x in sim.outputs[:max_index + 1]]

    # figure 1 plot output and reference
    # 绘制输出值和参考值
    plt.figure()
    plt.plot(step_arr, y_arr, label="Output")
    plt.plot(step_arr, ref, label="Reference")

    # 在第300个步长添加标记

    plt.axvline(x=start_index, color='red', linestyle='--', label=f"Step {start_index}")
    plt.scatter(start_index, y_arr[start_index], color='red', label="Marked Point")

    # 图例和标题
    plt.legend()
    plt.title("Output and Reference with Step Marked")
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.grid()
    plt.savefig('/Users/fjl2401/CPSim/src/examples/actuator_delay_attack/results/linear/quadrotor/state_{}.png'.format(
        'no_attack' if not attack else 'attack'))
    plt.show()

    # figure 2 plot input
    u_arr = [x[0] for x in sim.inputs[:max_index + 1]]
    plt.figure()
    plt.plot(step_arr, u_arr, label="Input")

    # 在第300个步长添加标记
    plt.axvline(x=start_index, color='red', linestyle='--', label=f"Step {start_index}")
    plt.scatter(start_index, u_arr[start_index], color='red', label="Marked Point")

    # 图例和标题
    plt.legend()
    plt.title("Input with Step Marked")
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.grid()
    plt.savefig('/Users/fjl2401/CPSim/src/examples/actuator_delay_attack/results/linear/quadrotor/input_{}.png'.format(
        'no_attack' if not attack else 'attack'))
    plt.show()

    # t_arr = np.linspace(0, 10, max_index + 1)
    # ref = [x[0] for x in quadrotor.refs[:max_index + 1]]
    # y_arr = [x[5] for x in quadrotor.outputs[:max_index + 1]]

    # plt.plot(t_arr, y_arr, t_arr, ref)
    # plt.show()

    # print results
    # import matplotlib.pyplot as plt
    #
    # t_arr = np.linspace(0, 10, max_index + 1)
    # ref = [x[0] for x in sim.refs[:max_index + 1]]
    # y_arr = [x[5] for x in sim.outputs[:max_index + 1]]
    #
    # plt.plot(t_arr, y_arr, t_arr, ref)
    # plt.show()
