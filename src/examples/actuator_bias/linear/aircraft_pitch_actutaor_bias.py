#Ref: https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=ControlPID
import numpy as np

from cpsim import Simulator
from cpsim.controllers.PID import PID

# system dynamics
A = [[-0.313, 56.7, 0],
     [-0.0139, -0.426, 0],
     [0, 56.7, 0]]
B = [[0.232], [0.0203], [0]]
C = [[0, 0, 1]]
D = [[0]]

x_0 = np.array([0.0, 0.0, 0.0])

# utils parameters
KP = 1.13
KI = 0.0253
KD = 0.0
control_limit = {'lo': [-20], 'up': [20]}

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


class AircraftPitch(Simulator):
    """
      States: (3,)
          x[0]: Angle of attack
          x[1]: Pitch rate
          x[2]: Pitch angle
      Control Input: (1,)
          u[0]: the elevator deflection angle
      Output:  (1,)
          y[0]: the pitch angle of the aircraft
          Output Feedback
      Controller: PID
      """
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
    max_index = 1500
    dt = 0.02
    ref = [np.array([0.2])] * 1501

    # actuator bias attack set
    bias_magnitude = 0.2  
    bias_start = 500     
    bias_end = 1000  
    bias = np.array([bias_magnitude])

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


    noise = {
        'process': {
            'type': 'white',
            'param': {'C': np.eye(3) * 0.00001}
        }
    }

    aircraft_pitch = AircraftPitch('test', dt, max_index, noise)
    
    for i in range(0, max_index + 1):
        assert aircraft_pitch.cur_index == i
        aircraft_pitch.update_current_ref(ref[i])


        u0 = aircraft_pitch.controller.update(
            aircraft_pitch.cur_ref, 
            aircraft_pitch.cur_feedback, 
            aircraft_pitch.dt * aircraft_pitch.cur_index
        )

        # attack here
        u1 = actuator_bias_attack.launch(u0, aircraft_pitch.cur_index, aircraft_pitch.states)      

        aircraft_pitch.evolve(u=u1)



    # print results
    import matplotlib.pyplot as plt
    import os
    current_dir = os.path.dirname(__file__)
    results_dir = os.path.join(current_dir, '..', 'results')
    os.makedirs(results_dir, exist_ok=True)


    t_arr = np.linspace(0, max_index * dt, max_index + 1)
    ref_arr = [x[0] for x in aircraft_pitch.refs[:max_index + 1]]
    y_arr = [x[0] for x in aircraft_pitch.outputs[:max_index + 1]]

    u_arr = [x[0] for x in aircraft_pitch.inputs[:max_index + 1]]

    plt.figure(figsize=(10, 5))
    plt.title(f'Aircraft Pitch Angle with Bias Attack (Bias = {bias_magnitude})')
    plt.plot(t_arr, y_arr, label='Actual Pitch Angle')
    plt.plot(t_arr, ref_arr, label='Reference Pitch Angle', linestyle='--')
    plt.scatter(t_arr[bias_start], y_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], y_arr[bias_end], color='orange', label='Bias End', zorder=5)
    plt.xlabel('Time')
    plt.ylabel('Pitch Angle')
    plt.legend()
    plt.grid()

    plt.savefig(os.path.join(results_dir, 'aircraft_pitch_angle_bias_attack.png'))
    
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.title(f'Aircraft Elevator Deflection with Bias Attack (Bias = {bias_magnitude})')
    plt.plot(t_arr, u_arr, label='Control Input (Elevator Deflection)')
    plt.scatter(t_arr[bias_start], u_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], u_arr[bias_end], color='orange', label='Bias End', zorder=5)
    plt.xlabel('Time')
    plt.ylabel('Elevator Deflection')
    plt.legend()
    plt.grid()
    
    plt.savefig(os.path.join(results_dir, 'aircraft_elevator_bias_attack.png'))

    plt.show()


    # t_arr = np.linspace(0, 10, max_index + 1)
    # ref = [x[0] for x in aircraft_pitch.refs[:max_index + 1]]
    # y_arr = [x[0] for x in aircraft_pitch.outputs[:max_index + 1]]



    # plt.plot(t_arr, y_arr, t_arr, ref)
    # plt.show()

    # u_arr = [x[0] for x in aircraft_pitch.inputs[:max_index + 1]]
    # plt.plot(t_arr, u_arr)
