# Ref: https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=ControlPID

import numpy as np

from cpsim import Simulator
from cpsim.controllers.PID import PID

# system dynamics
J = 0.01  # moment of inertia of the rotor
b = 0.1  # motor viscous friction constant
Ke = 0.01  # electromotive force constant
Kt = 0.01  # motor torque constant
R = 1  # electric resistance
L = 0.5  # electric inductance

A = [[-b / J, Kt / J], [-Ke / L, -R / L]]
B = [[0], [1 / L]]
C = [[1, 0]]

x_0 = np.array([0.0, 0.0])

# utils parameters
P = 19
I = 37
D = 0.1

control_limit = {
    'lo': np.array([0]),
    'up': np.array([60])
}


def envolve_sim(sim, ref, max_index, attack=False):
    if attack:
        for i in range(0, max_index + 1):
            assert sim.cur_index == i
            sim.update_current_ref(ref[i])
            u = delay_attack.launch(sim.cur_u, sim.cur_index, sim.inputs)
            sim.evolve(u=u)
        # print results
    else:
        for i in range(0, max_index + 1):
            assert sim.cur_index == i
            sim.update_current_ref(ref[i])
            sim.evolve()


class Controller:
    def __init__(self, dt):
        self.dt = dt
        self.pid = PID(P, I, D, current_time=-dt)
        self.pid.setWindup(100)
        self.pid.setSampleTime(dt)
        self.set_control_limit(control_limit['lo'], control_limit['up'])

    def update(self, ref: np.ndarray, feedback_value: np.ndarray, current_time) -> np.ndarray:
        self.pid.set_reference(ref[0])
        cin = self.pid.update(feedback_value[0], current_time)
        return np.array([cin])

    def set_control_limit(self, control_lo, control_up):
        self.control_lo = control_lo
        self.control_up = control_up
        self.pid.set_control_limit(self.control_lo[0], self.control_up[0])

    def clear(self):
        self.pid.clear(current_time=-self.dt)


class MotorSpeed(Simulator):
    """
            States: (2,)
                x[0]: the rotational speed of the shaft
                x[1]: electric current
            Control Input: (1,)
                u[0]: voltage source
            Output:  (1,)
                y[0]: the rotational speed of the shaft
                Output Feedback
            Controller: PID
            """

    def __init__(self, name, dt, max_index, noise=None):
        super().__init__('Motor Speed ' + name, dt, max_index)
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


if __name__ == "__main__":
    max_index = 500
    dt = 0.02
    ref = [np.array([5])] * 201 + [np.array([4])] * 200 + [np.array([5])] * 100
    noise = {
        'process': {
            'type': 'white',
            'param': {'C': np.eye(2) * 0.1}
        }
    }

    from cpsim.actuator_attack import ActuatorAttack

    sim = MotorSpeed('test', dt, max_index, noise)
    delay_attack = ActuatorAttack('delay', 10, 300, simulator=sim)
    attack = False
    for i in range(0, max_index + 1):
        assert sim.cur_index == i
        sim.update_current_ref(ref[i])
        if attack:
            u = delay_attack.launch(sim.cur_u, sim.cur_index, sim.inputs)
            print('this is u', u)
            sim.evolve(u)
        else:
            sim.evolve()

    import matplotlib.pyplot as plt

    # t_arr = np.linspace(0, 10, max_index + 1)
    step_arr = np.arange(0, max_index + 1)
    ref = [x[0] for x in sim.refs[:max_index + 1]]
    y_arr = [x[0] for x in sim.outputs[:max_index + 1]]

    # figure 1 plot output and reference
    # 绘制输出值和参考值
    plt.figure()
    plt.plot(step_arr, y_arr, label="Output")
    plt.plot(step_arr, ref, label="Reference")

    # 在第300个步长添加标记
    step_index = 300
    plt.axvline(x=step_index, color='red', linestyle='--', label=f"Step {step_index}")
    plt.scatter(step_index, y_arr[step_index], color='red', label="Marked Point")

    # 图例和标题
    plt.legend()
    plt.title("Output and Reference with Step Marked")
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.grid()
    plt.savefig('/Users/fjl2401/CPSim/src/examples/actuator_delay_attack/results/linear/motor/state_{}.png'.format(
        'no_attack' if not attack else 'attack'))
    plt.show()

    # figure 2 plot input
    u_arr = [x[0] for x in sim.inputs[:max_index + 1]]
    plt.figure()
    plt.plot(step_arr, u_arr, label="Input")

    # 在第300个步长添加标记
    plt.axvline(x=step_index, color='red', linestyle='--', label=f"Step {step_index}")
    plt.scatter(step_index, u_arr[step_index], color='red', label="Marked Point")

    # 图例和标题
    plt.legend()
    plt.title("Input with Step Marked")
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.grid()
    plt.savefig('/Users/fjl2401/CPSim/src/examples/actuator_delay_attack/results/linear/motor/input_{}.png'.format(
        'no_attack' if not attack else 'attack'))
    plt.show()
