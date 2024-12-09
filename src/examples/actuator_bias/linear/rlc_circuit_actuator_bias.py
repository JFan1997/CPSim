import numpy as np

from cpsim import Simulator
from cpsim.controllers.PID import PID

# system dynamics
R = 10000
L = 0.5
C = 0.0001

A = [[0, 1 / C], [-1 / L, -R / L]]
B = [[0], [1 / L]]
C = [[1, 0]]
D = [[0]]

x_0 = np.array([0.0, 0.0])

# utils parameters
KP = 5
KI = 5
KD = 0
control_limit = {'lo': [-15], 'up': [15]}


class Controller:
    def __init__(self, dt):
        self.dt = dt
        self.pid = PID(KP, KI, KD, current_time=-dt)
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


class RlcCircuit(Simulator):
    def __init__(self, name, dt, max_index, noise=None):
        super().__init__('Aircraft Pitch ' + name, dt, max_index)
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
    ref = [np.array([2])] * 201 + [np.array([3])] * 200 + [np.array([2])] * 100
    
    # bias actuator attack set
    bias_magnitude = 4  
    bias_start = 275     
    bias_end = 325       
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
            'param': {'C': np.eye(2) * 0.01}
        }
    }
    rlc_circuit = RlcCircuit('test', dt, max_index, noise)
    for i in range(0, max_index + 1):
        assert rlc_circuit.cur_index == i
        rlc_circuit.update_current_ref(ref[i])

        u0 = rlc_circuit.controller.update(
            rlc_circuit.cur_ref, 
            rlc_circuit.cur_feedback, 
            rlc_circuit.dt * rlc_circuit.cur_index
        )

        # bias attack here
        u1 = actuator_bias_attack.launch(u0, rlc_circuit.cur_index, rlc_circuit.states)     
              
        
        rlc_circuit.evolve(u=u1)



    # print results
    import matplotlib.pyplot as plt

    import os
    current_dir = os.path.dirname(__file__)
    results_dir = os.path.join(current_dir, '..', 'results')
    os.makedirs(results_dir, exist_ok=True)


    t_arr = np.linspace(0, 10, max_index + 1)
    ref = [x[0] for x in rlc_circuit.refs[:max_index + 1]]
    y_arr = [x[0] for x in rlc_circuit.outputs[:max_index + 1]]

    u_arr = [x[0] for x in rlc_circuit.inputs[:max_index + 1]]

    # Voltage tracking
    plt.figure(figsize=(10, 5))
    plt.title(f'RLC Circuit Voltage with Bias Attack (Bias = {bias_magnitude})')
    plt.plot(t_arr, y_arr, label='Capacitor Voltage')
    plt.plot(t_arr, ref, label='Reference Voltage', linestyle='--')
    
    plt.scatter(t_arr[bias_start], y_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], y_arr[bias_end], color='orange', label='Bias End', zorder=5)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Voltage (V)')
    plt.legend()
    plt.grid()

    plt.savefig(os.path.join(results_dir, 'rlc_voltage_bias_attack.png'))
    
    plt.show()

    # control input
    plt.figure(figsize=(10, 5))
    plt.title(f'RLC Circuit Control Input with Bias Attack (Bias = {bias_magnitude})')
    plt.plot(t_arr, u_arr, label='Control Input')

    plt.scatter(t_arr[bias_start], u_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], u_arr[bias_end], color='orange', label='Bias End', zorder=5)

    plt.xlabel('Time (s)')
    plt.ylabel('Input Voltage (V)')
    plt.legend()
    plt.grid()

    plt.savefig(os.path.join(results_dir, 'rlc_control_input_bias_attack.png'))
    
    plt.show()


    # plt.plot(t_arr, y_arr, t_arr, ref)
    # plt.show()
