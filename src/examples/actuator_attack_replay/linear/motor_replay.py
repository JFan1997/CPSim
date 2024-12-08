# Ref: https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=ControlPID

import numpy as np
import matplotlib.pyplot as plt
from cpsim import Simulator
from cpsim.controllers.PID import PID
from cpsim.actuator_attack import ActuatorAttack
    
# system dynamics
J = 0.01   # moment of inertia of the rotor
b = 0.1    # motor viscous friction constant
Ke = 0.01  # electromotive force constant
Kt = 0.01  # motor torque constant
R = 1      # electric resistance
L = 0.5    # electric inductance

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
    params={'start': 250, 'end': 350, 'bias': 0.1, 'step': 1}
    bias_attack = ActuatorAttack('replay', params, max_index)
    ip = MotorSpeed('test', dt, max_index, noise)


    attack = True
    for i in range(0, max_index + 1):
        if (i > params['start'] and i < params['end']) and attack==True:
            assert ip.cur_index == i
            ip.update_current_ref(ref[i])
            # attack here
            u = bias_attack.launch(ip.cur_u, ip.cur_index, ip.inputs)
            ip.evolve(u=u)
        else:
            assert ip.cur_index == i
            ip.update_current_ref(ref[i])
            # attack here
            ip.evolve()
    # print results

    # # 定义数据
    t_arr = np.linspace(0, 500, max_index + 1)
    ref = [x[0] for x in ip.refs[:max_index + 1]]
    y_arr = [x[0] for x in ip.outputs[:max_index + 1]]
    u_arr = [x[0] for x in ip.inputs[:max_index + 1]]



    # 第一个图
    plt.figure()
    plt.plot(t_arr, y_arr, label="Output")
    plt.plot(t_arr, ref, label="Reference Speed", linestyle='dashed')
    if attack:
        plt.title('Motor Speed with Replay Attack')
        plt.plot(t_arr[params['start']], y_arr[params['start']], 'o', label='Replay Start', color='red')
        plt.plot(t_arr[params['end']], y_arr[params['end']], 'o', label='Replay End', color='orange')

    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.grid()
    if attack:
        plt.savefig('./figs/linear/motor.png', dpi=800)
    else:
        plt.savefig('./figs/linear/motor (no attack).png', dpi=800)
    plt.show()

    # ------------------------------------------------------------

    # 第二个图
    plt.figure()
    plt.plot(t_arr, u_arr, label="Control Input (Armature Voltage)")
    if attack:
        plt.title('Motor Control Input with Replay Attack')
        plt.plot(t_arr[params['start']], u_arr[params['start']], 'o', label='Replay Start', color='red')
        plt.plot(t_arr[params['end']], u_arr[params['end']], 'o', label='Replay End', color='orange')

    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Voltage')
    plt.grid()
    if attack:
        plt.savefig('./figs/linear/motor-uforce.png', dpi=800)
    else:
        plt.savefig('./figs/linear/motor-uforce (no attack).png', dpi=800)
    plt.show()