#Ref:https://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
import numpy as np

from cpsim import Simulator
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

    # actuator bias attack set
    bias_magnitude = 20
    bias = np.array([bias_magnitude]) 
    bias_start = 275
    bias_end = 325

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




    quadrotor = Quadrotor('test', dt, max_index, None)

    for i in range(0, max_index + 1):
        assert quadrotor.cur_index == i
        quadrotor.update_current_ref(ref[i])

        # origin control input
        u0 = quadrotor.controller.update(quadrotor.cur_ref, 
                                         quadrotor.cur_feedback, 
                                         quadrotor.dt * quadrotor.cur_index)
        
        # attack here
        u1 = actuator_bias_attack.launch(u0, quadrotor.cur_index, quadrotor.states)

        quadrotor.evolve(u = u1)
    
    
    # print results
    import matplotlib.pyplot as plt
    import os
    current_dir = os.path.dirname(__file__)
    results_dir = os.path.join(current_dir, '..', 'results')
    os.makedirs(results_dir, exist_ok=True)


    t_arr = np.linspace(0, 10, max_index + 1)
    ref = [x[0] for x in quadrotor.refs[:max_index + 1]]
    y_arr = [x[5] for x in quadrotor.outputs[:max_index + 1]]


    u_arr = [x[0] for x in quadrotor.inputs[:max_index + 1]]


    # height
    plt.figure(figsize=(7, 5))
    
    # plt.title(f'Quadrotor Altitude with Bias Attack (Bias = {bias_magnitude})')
    
    plt.plot(t_arr, y_arr, label='Actual Altitude')
    plt.plot(t_arr, ref, label='Reference Altitude', linestyle='--')
    
    plt.scatter(t_arr[bias_start], y_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], y_arr[bias_end], color='orange', label='Bias End', zorder=5)

    plt.xlabel('Time')
    plt.ylabel('Altitude')
    plt.legend()
    plt.grid()
    
    plt.savefig(os.path.join(results_dir, 'quadrotor_altitude_bias_attack.png'))
    
    plt.show()

    # control input
    plt.figure(figsize=(7, 5))
    
    plt.title(f'Quadrotor Control Input with Bias Attack (Bias = {bias_magnitude})')
    
    plt.plot(t_arr, u_arr, label='Control Input')
    
    plt.scatter(t_arr[bias_start], u_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], u_arr[bias_end], color='orange', label='Bias End', zorder=5)

    plt.xlabel('Time')
    plt.ylabel('Thrust Input')
    plt.legend()
    plt.grid()

    plt.savefig(os.path.join(results_dir, 'quadrotor_control_input_bias_attack.png'))
    
    plt.show()


    # plt.plot(t_arr, y_arr, t_arr, ref)
    # plt.show()
