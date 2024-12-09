# Ref: https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=ControlPID

import numpy as np

from cpsim import Simulator
from cpsim.controllers.PID import PID

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
    

    # actuator bias attack set
    bias_magnitude = 50 
    bias_start = 250     
    bias_end = 350       
    bias = np.array([bias_magnitude])
    

    import sys
    import os
    # Dynamically add cpsim path to sys.path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    cpsim_dir = os.path.abspath(os.path.join(current_dir, '../../../cpsim'))
    sys.path.insert(0, cpsim_dir)

    from actuator_attack import ActuatorAttack
    actuator_bias_attack = ActuatorAttack('bias', 
                                          bias, 
                                          bias_start, 
                                          bias_end)    
    
    noise = {
        'process': {
            'type': 'white',
            'param': {'C': np.eye(2) * 0.1}
        }
    }

    motor_speed = MotorSpeed('test', dt, max_index, noise)

    for i in range(0, max_index + 1):
        assert motor_speed.cur_index == i
        motor_speed.update_current_ref(ref[i])


        u0 = motor_speed.controller.update(
            motor_speed.cur_ref, 
            motor_speed.cur_feedback, 
            motor_speed.dt * motor_speed.cur_index
        )

        # attack here
        u1 = actuator_bias_attack.launch(u0, motor_speed.cur_index, motor_speed.states)     
        print("here")
        motor_speed.evolve(u=u1)


    # print results
    import matplotlib.pyplot as plt


    import os
    current_dir = os.path.dirname(__file__)
    results_dir = os.path.join(current_dir, '..', 'results')
    os.makedirs(results_dir, exist_ok=True)


    t_arr = np.linspace(0, max_index * dt, max_index + 1)
    ref_arr = [x[0] for x in motor_speed.refs[:max_index + 1]]
    y_arr = [x[0] for x in motor_speed.outputs[:max_index + 1]]


    u_arr = [x[0] for x in motor_speed.inputs[:max_index + 1]]

    # speed tracking
    plt.figure(figsize=(8, 5))

    plt.title(f'Motor Speed with Bias Attack (Bias = {bias_magnitude})')

    plt.plot(t_arr, y_arr, label='Rotor Speed')
    plt.plot(t_arr, ref_arr, label='Reference Speed', linestyle='--')

    plt.scatter(t_arr[bias_start], y_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], y_arr[bias_end], color='orange', label='Bias End', zorder=5)

    plt.xlabel('Time steps')
    plt.ylabel('Speed')
    plt.legend()
    plt.grid()

    plt.savefig(os.path.join(results_dir, 'motor_speed_bias_attack.png'))

    plt.show()

    # control input
    plt.figure(figsize=(8, 5))
    plt.title(f'Motor Control input with Bias Attack (Bias = {bias_magnitude})')

    plt.plot(t_arr, u_arr, label='Control Input (Armature Voltage)')

    plt.scatter(t_arr[bias_start], u_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], u_arr[bias_end], color='orange', label='Bias End', zorder=5)

    plt.xlabel('Time steps')
    plt.ylabel('Voltage')
    plt.legend()
    plt.grid()


    plt.savefig(os.path.join(results_dir, 'motor_voltage_bias_attack.png'))

    plt.show()



    # plt.plot(t_arr, y_arr, t_arr, ref)
    # plt.show()

    # u_arr = [x[0] for x in motor_speed.inputs[:max_index + 1]]
    # plt.plot(t_arr, u_arr)
    # plt.show()