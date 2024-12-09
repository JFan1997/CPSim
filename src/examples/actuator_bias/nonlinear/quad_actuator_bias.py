import math
import numpy as np
from cpsim import Simulator
from cpsim.controllers.PID import PID


def quad(t, x, u, use_imath=False):
    Ix = 5 * 0.001
    Iy = 5 * 0.001
    Iz = 5 * 0.001
    mass = 1
    g = 9.81

    phi, theta, psi, w_phi, w_theta, w_psi, x, y, z, v_x, v_y, v_z = x
    U_t = u[0]
    U_phi = 0
    U_theta = 0
    U_psi = 0

    xdot = np.array([
        w_phi,
        w_theta,
        w_psi,
        U_phi / Ix + w_theta * w_psi * (Iy - Iz) / Ix,
        U_theta / Iy + w_phi * w_psi * (Iz - Iz) / Iy,
        U_psi / Iz + w_phi * w_theta * (Ix - Iy) / Iz,
        v_x,
        v_y,
        v_z,
        U_t / mass * (math.cos(phi) * math.sin(theta) * math.cos(psi) + math.sin(phi) * math.sin(psi)),
        U_t / mass * (math.cos(phi) * math.sin(theta) * math.sin(psi) - math.sin(phi) * math.cos(psi)),
        U_t / mass * math.cos(phi) * math.cos(theta) - g,
    ])
    return xdot


x_0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
control_limit = {
    'lo': np.array([-10]),
    'up': np.array([100])
}

# utils parameters


KP =  100
KI = 0
KD = -19
class Controller:
    def __init__(self, dt):
        self.dt = dt
        self.pid = PID(KP, KI, KD, current_time=-dt)
        self.pid.setWindup(0)
        self.pid.setSampleTime(dt)
        self.set_control_limit(control_limit['lo'], control_limit['up'])

    def update(self, ref: np.ndarray, feedback_value: np.ndarray, current_time) -> np.ndarray:
        self.pid.set_reference(ref[-4])
        cin = self.pid.update(feedback_value[-4], current_time)
        return np.array([cin])

    def set_control_limit(self, control_lo, control_up):
        self.control_lo = control_lo
        self.control_up = control_up
        self.pid.set_control_limit(self.control_lo[0], self.control_up[0])

    def clear(self):
        self.pid.clear(current_time=-self.dt)



class quadrotor(Simulator):
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
        super().__init__('Quadrotor ' + name, dt, max_index)
        self.nonlinear(ode=quad, n=12, m=1, p=12)
        controller = Controller(dt)
        settings = {
            'init_state': x_0,
            'feedback_type': 'state',
            'controller': controller
        }
        if noise:
            settings['noise'] = noise
        self.sim_init(settings)


if __name__ == "__main__":
    max_index = 3000
    dt = 0.01
    ref = [np.array([0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0])] * (max_index + 1)


    # actuator bias attack set
    bias_magnitude = 80
    bias = np.array([bias_magnitude]) 
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
                                          bias, 
                                          bias_start, 
                                          bias_end)


    sim = quadrotor('test', dt, max_index)
    for i in range(0, max_index + 1):
        assert sim.cur_index == i
        sim.update_current_ref(ref[i])


        u0 = sim.controller.update(
            sim.cur_ref, 
            sim.cur_feedback, 
            sim.dt * sim.cur_index)
        
        # print('this is action', u0)

        u1 = actuator_bias_attack.launch(u0, sim.cur_index, sim.states)

        sim.evolve(u = u1)


    import matplotlib.pyplot as plt

    import os
    current_dir = os.path.dirname(__file__)
    results_dir = os.path.join(current_dir, '..', 'results')
    os.makedirs(results_dir, exist_ok=True)

    # t_arr = np.linspace(0, max_index * dt, max_index + 1)
    t_arr = np.linspace(0, 3, max_index + 1)

    ref2 = [x[-4] for x in sim.refs[:max_index + 1]]
    y2_arr = [x[-4] for x in sim.outputs[:max_index + 1]]
    u_arr = [x[0] for x in sim.inputs[:max_index + 1]]


    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(4, 8))  

    # height change
    ax1.set_title(f'Quad Altitude with bias Attack = {bias_magnitude}')
    # ax1.set_title(f'Quad Altitude')

    ax1.plot(t_arr, y2_arr, label='y2_arr')
    ax1.plot(t_arr, ref2, label='ref2')

    ax1.scatter(t_arr[bias_start], y2_arr[bias_start], color='red', label='Bias Start', zorder=5)
    ax1.scatter(t_arr[bias_end], y2_arr[bias_end], color='orange', label='Bias End', zorder=5)  
    
    ax1.legend()
    ax1.set_xlabel('index steps')
    ax1.set_ylabel('Altitude')

    # control input
    ax2.set_title(f'Quad u-force with bias Attack = {bias_magnitude}')
    # ax2.set_title(f'Quad u-force ')

    ax2.plot(t_arr, u_arr, label='u_arr')
    
    ax2.scatter(t_arr[bias_start], u_arr[bias_start], color='red', label='Bias Start', zorder=5)
    ax2.scatter(t_arr[bias_end], u_arr[bias_end], color='orange', label='Bias End', zorder=5)

    ax2.axhline(y=9.81, color='red', linestyle='--', label='y=9.81')
    ax2.annotate('9.81', xy=(0, 9.81), xytext=(-10, 0),
                textcoords='offset points', color='red', ha='right', va='center')
    ax2.legend()
    ax2.set_xlabel('index steps')
    ax2.set_ylabel('Control Input')

    plt.tight_layout()

    plt.savefig(os.path.join(results_dir, 'quad altitude & u-force bias.png'))
    
    plt.show()

