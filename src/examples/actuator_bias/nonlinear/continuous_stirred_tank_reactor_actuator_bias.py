import numpy as np
from numpy import exp

from cpsim import Simulator
from cpsim.controllers.PID_incremental import PID

# Parameters:
# Volumetric Flowrate (m^3/sec)
q = 100
# Volume of CSTR (m^3)
V = 100
# Density of A-B Mixture (kg/m^3)
rho = 1000
# Heat capacity of A-B Mixture (J/kg-K)
Cp = 0.239
# Heat of reaction for A->B (J/mol)
mdelH = 5e4
# E - Activation energy in the Arrhenius Equation (J/mol)
# R - Universal Gas Constant = 8.31451 J/mol-K
EoverR = 8750
# Pre-exponential factor (1/sec)
k0 = 7.2e10
# U - Overall Heat Transfer Coefficient (W/m^2-K)
# A - Area - this value is specific for the U calculation (m^2)
UA = 5e4


def cstr(t, x, u, Tf=350, Caf=1, use_imath=False):
    # Inputs (3):
    # Temperature of cooling jacket (K)
    Tc = u[0]
    # Tf = Feed Temperature (K)
    # Caf = Feed Concentration (mol/m^3)

    # States (2):
    # Concentration of A in CSTR (mol/m^3)
    Ca = x[0]
    # Temperature in CSTR (K)
    T = x[1]

    # reaction rate
    if use_imath:
        from interval import imath
        rA = k0 * imath.exp(-EoverR / T) * Ca
    else:
        rA = k0 * np.exp(-EoverR / T) * Ca

    # Calculate concentration derivative
    dCadt = q / V * (Caf - Ca) - rA
    # Calculate temperature derivative
    dTdt = q / V * (Tf - T) \
           + mdelH / (rho * Cp) * rA \
           + UA / V / rho / Cp * (Tc - T)

    # Return xdot:
    if use_imath:
        return [dCadt, dTdt]
    else:
        xdot = np.zeros(2)
        xdot[0] = dCadt
        xdot[1] = dTdt
        return xdot

def cstr_imath(t, x, u, Tf=350, Caf=1):
    return cstr(t=t, x=x, u=u, Tf=Tf, Caf=Caf, use_imath=True)


def f(x, u, dt):
    dx = cstr(None, x, u)
    return x + dt * dx

def jfx(x, u, dt):
    Ca = x[0]
    T = x[1]
    Tc = u[0]
    Ad = np.array([
        [dt * (-k0 * exp(-EoverR / T) - q / V) + 1, -Ca * EoverR * dt * k0 * exp(-EoverR / T) / T ** 2],
        [dt * k0 * mdelH * exp(-EoverR / T) / (Cp * rho),
         dt * (Ca * EoverR * k0 * mdelH * exp(-EoverR / T) / (Cp * T ** 2 * rho) - q / V - UA / (Cp * V * rho)) + 1]])
    return Ad

def jfu(x, u, dt):
    Ca = x[0]
    T = x[1]
    Tc = u[0]
    Bd = np.array([
        [0],
        [UA * dt / (Cp * V * rho)]])
    return Bd


# initial states
x_0 = np.array([0.98, 280])

# utils parameters
KP = 0.5 * 1.0
KI =  KP / (3 / 8.0)
KD =  - KP * 0.1
u_0 = 274.57786
control_limit = {
    'lo': np.array([250]),
    'up': np.array([350])
}


class Controller:
    def __init__(self, dt):
        self.dt = dt
        self.pid = PID(u_0, KP, KI, KD, current_time=-dt)
        self.pid.setWindup(100)
        self.pid.setSampleTime(dt)
        self.set_control_limit(control_limit['lo'], control_limit['up'])

    def update(self, ref: np.ndarray, feedback_value: np.ndarray, current_time) -> np.ndarray:
        self.pid.set_reference(ref[1])             # only care about the 2nd state here
        cin = self.pid.update(feedback_value[1], current_time)  # only use the 2nd state here
        return np.array([cin])

    def set_control_limit(self, control_lo, control_up):
        self.control_lo = control_lo
        self.control_up = control_up
        self.pid.set_control_limit(self.control_lo[0], self.control_up[0])

    def clear(self):
        self.pid.clear(current_time=-self.dt)


class CSTR(Simulator):
    """
        States: (2,)
            x[0]: Concentration of A in CSTR (mol/m^3)
            x[1]: Temperature in CSTR (K)
        Control Input: (1,)
            u[0]: Temperature of cooling jacket (K)
        Output:  (2,)
            State Feedback
        Controller: PID
    """

    def __init__(self, name, dt, max_index, noise=None):
        super().__init__('CSTR ' + name, dt, max_index)
        self.nonlinear(ode=cstr, n=2, m=1, p=2)  # number of states, utils inputs, outputs
        controller = Controller(dt)
        settings = {
            'init_state': x_0,
            'feedback_type': 'state',  # 'state' or 'output',  you must define C if 'output'
            'controller': controller
        }
        if noise:
            settings['noise'] = noise
        self.sim_init(settings)


if __name__ == "__main__":
    max_index = 1000
    dt = 0.02
    ref = [np.array([0, 300])] * (max_index+1)

    # noise = {
    #     'process': {
    #         'type': 'box_uniform',
    #         'param': {'lo': np.array([-0.000001, -0.001]), 
    #                   'up': np.array([0.001, 0.001])}
    #     }
    # }
    noise = None

    # actuator bias attack set
    bias_magnitude = 80  
    bias_start = 500     
    bias_end = 800       
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

    cstr_model = CSTR('test', dt, max_index, noise)


    for i in range(0, max_index + 1):
        assert cstr_model.cur_index == i
        cstr_model.update_current_ref(ref[i])
        # attack here

        # x = A x + B u
        # x, u - 1-D array
        # cstr_model.cur_x   #  ground truth
        # cstr_model.cur_u   #  utils input of last step
        # cstr_model.cur_y   #
        # cstr_model.cur_feedback   # attack on this


        u0 = cstr_model.controller.update(
            cstr_model.cur_ref, 
            cstr_model.cur_feedback, 
            cstr_model.dt * cstr_model.cur_index
        )

        u1 = actuator_bias_attack.launch(u0, cstr_model.cur_index, cstr_model.states)

        cstr_model.evolve(u=u1)



    # print results
    import matplotlib.pyplot as plt


    import os
    current_dir = os.path.dirname(__file__)
    results_dir = os.path.join(current_dir, '..', 'results')
    os.makedirs(results_dir, exist_ok=True)


    t_arr = np.linspace(0, max_index * dt, max_index + 1)
    T_arr = [x[1] for x in cstr_model.outputs[:max_index + 1]]
    T_c_arr = [x[0] for x in cstr_model.inputs[:max_index + 1]]
    ref_T = [x[1] for x in cstr_model.refs[:max_index + 1]]



    plt.figure()
    plt.title(f'CSTR Temperature with Bias Attack (Bias = {bias_magnitude} K)')
    plt.plot(t_arr, T_arr, label='Reactor Temperature (T)')
    plt.plot(t_arr, ref_T, label='Reference Temperature', linestyle='--')


    plt.scatter(t_arr[bias_start], T_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], T_arr[bias_end], color='orange', label='Bias End', zorder=5)

    plt.xlabel('Time steps')
    plt.ylabel('Temperature (K)')
    plt.legend()
    plt.grid()

    plt.savefig(os.path.join(results_dir, 'cstr_temperature_bias_attack.png'))
    
    plt.show()

    # control input
    plt.figure()
    plt.title(f'CSTR Cooling Jacket Temperature with Bias Attack (Bias = {bias_magnitude} K)')
    plt.plot(t_arr, T_c_arr, label='Cooling Jacket Temperature (Tc)')

    plt.scatter(t_arr[bias_start], T_c_arr[bias_start], color='red', label='Bias Start', zorder=5)
    plt.scatter(t_arr[bias_end], T_c_arr[bias_end], color='orange', label='Bias End', zorder=5)

    plt.xlabel('Time steps')
    plt.ylabel('Temperature (K)')
    plt.legend()
    plt.grid()

    plt.savefig(os.path.join(results_dir, 'cstr_cooling_jacket_bias_attack.png'))
    
    plt.show()



    # t_arr = np.linspace(0, 10, max_index + 1)
    # ref = [x[1] for x in cstr_model.refs[:max_index + 1]]
    # y_arr = [x[1] for x in cstr_model.outputs[:max_index + 1]]

    # plt.plot(t_arr, y_arr, t_arr, ref)
    # plt.show()

    # u_arr = [x[0] for x in cstr_model.inputs[:max_index + 1]]
    # plt.plot(t_arr, u_arr)
    # plt.show()
