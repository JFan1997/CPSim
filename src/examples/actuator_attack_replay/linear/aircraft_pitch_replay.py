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
    from cpsim.actuator_attack import ActuatorAttack
    max_index = 1500
    dt = 0.02
    ref = [np.array([0.2])] * 1501
    noise = {
        'process': {
            'type': 'white',
            'param': {'C': np.eye(3) * 0.00001}
        }
    }
    params={'start': 1000, 'end': 1500, 'bias': 0.1, 'step': 1}

    aircraft_pitch = AircraftPitch('test', dt, max_index, noise)

    bias_attack = ActuatorAttack('replay', params, max_index)

    for i in range(0, max_index + 1):
        if i > params['start'] and i < params['end']:
            assert aircraft_pitch.cur_index == i
            aircraft_pitch.update_current_ref(ref[i])
            # attack here
            u = bias_attack.launch(aircraft_pitch.cur_u, aircraft_pitch.cur_index, aircraft_pitch.inputs)
            aircraft_pitch.evolve(u=u)
        else:
            assert aircraft_pitch.cur_index == i
            aircraft_pitch.update_current_ref(ref[i])
            # attack here
            aircraft_pitch.evolve()


    # print results
    import matplotlib.pyplot as plt

    t_arr = np.linspace(0, 10, max_index + 1)
    ref = [x[0] for x in aircraft_pitch.refs[:max_index + 1]]
    y_arr = [x[0] for x in aircraft_pitch.outputs[:max_index + 1]]
    plt.figure()
    plt.plot(t_arr, y_arr, label="Output")
    plt.plot(t_arr, ref, label="Reference", linestyle='dashed')
    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Output and Reference with Step Marked')
    plt.grid()
    plt.savefig(f'./figs/linear/aircraft.png')
    plt.show()

    u_arr = [x[0] for x in aircraft_pitch.inputs[:max_index + 1]]
    plt.figure()
    plt.plot(t_arr, u_arr, label="Input")
    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Input with Step Marked')
    plt.grid()
    plt.savefig(f'./figs/linear/aircraft-force.png')
    plt.show()