# Ref: Data-Driven Science and Engineering: Machine Learning, Dynamical Systems, and Control (Session 8.7, Page 300)

import math

import numpy as np

from cpsim import Simulator
from cpsim.controllers.PID import PID


def quad(t, x, u, use_imath=False):
    # Control Inputs (4):
    # Only U_t is non-zero

    # More Parameters:
    Ix = 5 * 0.001
    Iy = 5 * 0.001
    Iz = 5 * 0.001  # values from https://repository.upenn.edu/cgi/viewcontent.cgi?article=1705&context=edissertations
    mass = 1
    g = 9.81

    # States (12):
    phi, theta, psi, w_phi, w_theta, w_psi, x, y, z, v_x, v_y, v_z = x
    U_t = u[0]
    U_phi = 0
    U_theta = 0
    U_psi = 0
    if use_imath:
        # from intervals.interval import imath
        from interval import imath
        xdot = [
            w_phi,
            w_theta,
            w_psi,
            U_phi / Ix + w_theta * w_psi * (Iy - Iz) / Ix,  # 0
            U_theta / Iy + w_phi * w_psi * (Iz - Iz) / Iy,  # 0
            U_psi / Iz + w_phi * w_theta * (Ix - Iy) / Iz,  # 0
            v_x,
            v_y,
            v_z,
            # Replaced theta with phi on RHS terms below based on https://arxiv.org/pdf/1602.02622.pdf
            U_t / mass * (imath.cos(phi) * imath.sin(theta) * imath.cos(psi) + imath.sin(phi) * imath.sin(psi)),
            U_t / mass * (imath.cos(phi) * imath.sin(theta) * imath.sin(psi) - imath.sin(phi) * imath.cos(psi)),
            U_t / mass * imath.cos(phi) * imath.cos(theta) - g
        ]
    else:
        xdot = np.array([
            w_phi,
            w_theta,
            w_psi,
            U_phi / Ix + w_theta * w_psi * (Iy - Iz) / Ix,  # 0
            U_theta / Iy + w_phi * w_psi * (Iz - Iz) / Iy,  # 0
            U_psi / Iz + w_phi * w_theta * (Ix - Iy) / Iz,  # 0
            v_x,
            v_y,
            v_z,
            # Replaced theta with phi on RHS terms below based on https://arxiv.org/pdf/1602.02622.pdf
            U_t / mass * (math.cos(phi) * math.sin(theta) * math.cos(psi) + math.sin(phi) * math.sin(psi)),
            U_t / mass * (math.cos(phi) * math.sin(theta) * math.sin(psi) - math.sin(phi) * math.cos(psi)),
            U_t / mass * math.cos(phi) * math.cos(theta) - g
        ])
    # xdot = [
    #     0,
    #     0,
    #     0,
    #     0, # 0
    #     0, # 0
    #     0, # 0
    #     0,
    #     0,
    #     v_z,
    #     # Replaced theta with phi on RHS terms below based on https://arxiv.org/pdf/1602.02622.pdf
    #     0,
    #     0,
    #     U_t/mass - g
    # ]
    # Return xdot:
    return xdot


def quad_imath(t, x, u):
    return quad(t=t, x=x, u=u, use_imath=True)


def quad_jfx(x, u, dt):
    # More Parameters:
    Ix = 5 * 0.001
    Iy = 5 * 0.001
    Iz = 5 * 0.001  # values from https://repository.upenn.edu/cgi/viewcontent.cgi?article=1705&context=edissertations
    mass = 1
    g = 9.81

    # States (12):
    phi, theta, psi, w_phi, w_theta, w_psi, x, y, z, v_x, v_y, v_z = x
    U_t = u[0]
    U_phi = 0
    U_theta = 0
    U_psi = 0

    Jacobian = np.array([
        [1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, dt * w_psi * (Iy - Iz) / Ix, dt * w_theta * (Iy - Iz) / Ix, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, dt * w_theta * (Ix - Iy) / Iz, dt * w_phi * (Ix - Iy) / Iz, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt],
        [U_t * dt * (-math.sin(phi) * math.sin(theta) * math.cos(psi) + math.sin(psi) * math.cos(phi)) / mass,
         U_t * dt * math.cos(phi) * math.cos(psi) * math.cos(theta) / mass,
         U_t * dt * (math.sin(phi) * math.cos(psi) - math.sin(psi) * math.sin(theta) * math.cos(phi)) / mass, 0, 0, 0,
         0, 0, 0, 1, 0, 0],
        [U_t * dt * (-math.sin(phi) * math.sin(psi) * math.sin(theta) - math.cos(phi) * math.cos(psi)) / mass,
         U_t * dt * math.sin(psi) * math.cos(phi) * math.cos(theta) / mass,
         U_t * dt * (math.sin(phi) * math.sin(psi) + math.sin(theta) * math.cos(phi) * math.cos(psi)) / mass, 0, 0, 0,
         0, 0, 0, 0, 1, 0],
        [-U_t * dt * math.sin(phi) * math.cos(theta) / mass, -U_t * dt * math.sin(theta) * math.cos(phi) / mass, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1]
    ])

    return Jacobian


x_0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
control_limit = {
    'lo': np.array([-10]),
    'up': np.array([100])
}

# utils parameters


KP = 100
KI = 20
KD = -40


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
    from cpsim.actuator_attack import ActuatorAttack

    # bias = np.array([0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0.2, 0.1])
    params={'start': 200, 'end': 600}

    bias_attack = ActuatorAttack('replay', params, 3000)
    
    ip = quadrotor('test', dt, max_index)
    
    attack = False
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

    # 定义数据
    t_arr = np.linspace(0, 3000, max_index + 1)
    ref = [x[-4] for x in ip.refs[:max_index + 1]]
    y_arr = [x[-4] for x in ip.outputs[:max_index + 1]]
    u_arr = [x[0] for x in ip.inputs[:max_index + 1]]

    import matplotlib.pyplot as plt
    import numpy as np

    # 第一个图
    plt.figure()
    plt.plot(t_arr, y_arr, label="Output")
    plt.plot(t_arr, ref, label="Reference", linestyle='dashed')
    if attack:
        plt.plot(t_arr[params['start']], y_arr[params['start']], 'o', label='Replay Start', color='red')
        plt.plot(t_arr[params['end']], y_arr[params['end']], 'o', label='Replay End', color='orange')
        plt.title(f'Quad Altitude with Replay Attack')
    else:
        plt.title(f'Quad Altitude')

    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.grid()
    if attack:
        plt.savefig('./figs/nonlinear/quadrotor.png', dpi=800)
    else:
        plt.savefig('./figs/nonlinear/quadrotor (no attack).png', dpi=800)
    plt.show()

    # ----------------------------------------------------------------------------------------

    # 第二个图
    plt.figure()
    plt.plot(t_arr, u_arr, label='Input')
    if attack:
        plt.plot(t_arr[params['start']], u_arr[params['start']], 'o', label='Replay Start', color='red')
        plt.plot(t_arr[params['end']], u_arr[params['end']], 'o', label='Replay End', color='orange')
        plt.title(f'Quad u-force with Replay Attack')
    else:
        plt.title(f'Quad u-force')

    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.axhline(9.80656, color='red', label='gravity')
    plt.grid()
    plt.legend()
    if attack:
        plt.savefig('./figs/nonlinear/quadrotor-uforce.png', dpi=800)
    else:
        plt.savefig('./figs/nonlinear/quadrotor-uforce (no attack).png', dpi=800)
    plt.show()
