'''
This file implements the simulation of real-time recovery linear quadratic regulator (RTR-LQR)
Virtual sensors (VS), Optimal probabilistic recovery in open loop (OPR-OL) and closed loop (OPR-CL)
'''

from copy import deepcopy
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 15})
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import numpy as np
import logging
import sys
import os
import csv
import time
from time import perf_counter

os.environ["RANDOM_SEED"] = '0'   # for reproducibility
sys.path.append(sys.path[0] + '/../../')

from cpsim.formal.gaussian_distribution import GaussianDistribution
from cpsim.formal.reachability import ReachableSet
from cpsim.formal.zonotope import Zonotope
from cpsim.observers.kalman_filter import KalmanFilter
from cpsim.observers.full_state_bound import Estimator
from cpsim.controllers.LP_cvxpy import LP
from cpsim.controllers.MPC_cvxpy import MPC

# from cpsim.performance.performance_metrics import distance_to_strip_center, in_strip

def in_strip(l, x, a, b):
    if l @ x >= a and l @ x <= b:
        success = 1
    else:
        success = 0
    return success

colors = {'none': 'red', 'lqr': 'C1', 'ssr': 'C2', 'oprp-close': 'C4', 'oprp-open': 'C0'}
labels = {'none': 'None', 'lqr': 'RTR-LQR', 'ssr': 'VS', 'oprp-close': 'OPR-PCL', 'oprp-open': 'OPR'}

# logger
logging.basicConfig(
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("debug.log"),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)






def save_exp_data(exp, exp_rst, bl, profiling_data = None):
    '''method to store data'''
    exp_rst[bl]['states'] = deepcopy(exp.model.states)
    exp_rst[bl]['outputs'] = deepcopy(exp.model.outputs)
    exp_rst[bl]['inputs'] = deepcopy(exp.model.inputs)
    exp_rst[bl]['time'] = {}

    exp_rst[bl]['recovery_steps'] = profiling_data['recovery_steps']
    exp_rst[bl]['recovery_complete_index'] = profiling_data['recovery_complete_index']
    
        

# ---------  attack + no recovery  -------------
def simulate_no_recovery(exp, bl):
    '''
    Simulation with attack and no recovery
    exp: class with the experiment characteristics
    '''
    exp.model.reset()
    exp_name = f" {bl} {exp.name} "
    logger.info(f"{exp_name:=^40}")
    for i in range(0, exp.max_index + 1):
        assert exp.model.cur_index == i
        exp.model.update_current_ref(exp.ref[i])
        # attack here
        exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)
        exp.model.evolve()

    profiling_data = {}
    profiling_data["recovery_complete_index"] = exp.max_index + 1
    profiling_data["recovery_steps"] = exp.max_index - exp.attack_start_index 
    
    return profiling_data


# ---------  LQR  -------------
def simulate_LQR(exp, bl):
    '''
    Simulate the RTR-LQR
    '''
    exp_name = f" {bl} {exp.name} "
    logger.info(f"{exp_name:=^40}")
    # required objects
    A = exp.model.sysd.A
    B = exp.model.sysd.B
    est = Estimator(A, B, max_k=150, epsilon=1e-7)
    maintain_time = 3
    exp.model.reset()
    # init variables
    recovery_complete_index = exp.max_index
    rec_u = None

    elapsed_times = []
    # Main loop 
    for i in range(0, exp.max_index + 1):
        assert exp.model.cur_index == i
        # Obtains reference
        exp.model.update_current_ref(exp.ref[i])
        # Launch attack
        exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)
        if i == exp.attack_start_index - 1:
            logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            pass

        
        # Check if the recovery begins
        if i == exp.recovery_index:
            logger.debug(f'recovery_index={i}, recovery_start_state={exp.model.cur_x}')

            # State reconstruction
            us = exp.model.inputs[exp.attack_start_index - 1:exp.recovery_index]
            x_0 = exp.model.states[exp.attack_start_index - 1]
            x_cur_lo, x_cur_up, x_cur = est.estimate(x_0, us)
            logger.debug(f'reconstructed state={x_cur}')

            # deadline estimate
            safe_set_lo = exp.safe_set_lo
            safe_set_up = exp.safe_set_up
            control = exp.model.inputs[i - 1]
            k = est.get_deadline(x_cur, safe_set_lo, safe_set_up, control, 100)
            recovery_complete_index = exp.recovery_index + k
            logger.debug(f'deadline={k}')
            # maintainable time compute


            # get recovery control sequence
            lqr_settings = {
                'Ad': A, 'Bd': B,
                'Q': exp.Q, 'QN': exp.QN, 'R': exp.R,
                'N': k + 3,
                'ddl': k, 'target_lo': exp.target_set_lo, 'target_up': exp.target_set_up,
                'safe_lo': exp.safe_set_lo, 'safe_up': exp.safe_set_up,
                'control_lo': exp.control_lo, 'control_up': exp.control_up,
                'ref': exp.recovery_ref
            }
            lqr = MPC(lqr_settings)
            _ = lqr.update(feedback_value=x_cur)
            rec_u = lqr.get_full_ctrl()
            rec_x = lqr.get_last_x()
            logger.debug(f'expected recovery state={rec_x}')
        # Implements the RTR-LQR recovery
        if i == recovery_complete_index + maintain_time:
            logger.debug(f'state after recovery={exp.model.cur_x}')
            step = recovery_complete_index + maintain_time - exp.recovery_index
            logger.debug(f'use {step} steps to recover.')
        # Check we need to use the recovery
        if exp.recovery_index <= i < recovery_complete_index + maintain_time:
            rec_u_index = i - exp.recovery_index
            u = rec_u[rec_u_index]
            exp.model.evolve(u)
        else: # If not using the recovery, run one step
            exp.model.evolve()
    # Store profiling data
    profiling_data = {}
    profiling_data["recovery_complete_index"] = recovery_complete_index
    profiling_data["recovery_steps"] = step
    return profiling_data


# ---------  attack + virtual sensors  -------------
def simulate_ssr(exp, bl):
    '''
    Implements the software sensors
    '''
    # required objects
    recovery_complete_index = exp.max_index - 1
    A = exp.model.sysd.A
    B = exp.model.sysd.B
    est = Estimator(A, B, max_k=150, epsilon=1e-7)
    logger.info(f"{bl} {exp.name:=^40}")
    for i in range(0, exp.max_index + 1):
        assert exp.model.cur_index == i
        exp.model.update_current_ref(exp.ref[i])
        # attack here
        exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)

        if i == exp.attack_start_index - 1:
            logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            pass
        if i == exp.recovery_index:
            logger.debug(f'recovery_index={i}, recovery_start_state={exp.model.cur_x}')

            # State reconstruction
            us = exp.model.inputs[exp.attack_start_index - 1:exp.recovery_index]
            x_0 = exp.model.states[exp.attack_start_index - 1]
            x_cur = est.estimate_wo_bound(x_0, us)
            logger.debug(f'one before reconstructed state={x_cur}')
            last_predicted_state = deepcopy(x_cur)

        if exp.recovery_index <= i < recovery_complete_index:
            # check if it is in target set
            if in_strip(exp.s.l, last_predicted_state, exp.s.a, exp.s.b):
                recovery_complete_index = i
                logger.debug('state after recovery={exp.model.cur_x}')
                step = recovery_complete_index - exp.recovery_index
                logger.debug(f'use {step} steps to recover.')
            
            # Estimate the state using the virtual sensors
            us = [exp.model.inputs[i - 1]]
            x_0 = last_predicted_state
            x_cur = est.estimate_wo_bound(x_0, us)

            # Set the feedback to the state estimated by the virtual sensors
            exp.model.cur_feedback = exp.model.sysd.C @ x_cur
            last_predicted_state = deepcopy(x_cur)
            print(f'{exp.model.cur_u}')


        exp.model.evolve()
    
    profiling_data = {}
    profiling_data["recovery_complete_index"] = recovery_complete_index
    profiling_data["recovery_steps"] = recovery_complete_index - exp.recovery_index 
    return profiling_data


# ---------  attack + OPR-CL  -------------
def simulate_oprp_cl(exp, bl):
    '''
    Simulate the OPR-CL
    exp: class with the experiments characteristics.
    '''
    # required objects
    A = exp.model.sysd.A
    B = exp.model.sysd.B
    # Initialize the Kalman filter
    kf_C = exp.kf_C
    C = exp.model.sysd.C
    D = exp.model.sysd.D
    kf_Q = exp.model.p_noise_dist.sigma if exp.model.p_noise_dist is not None else np.zeros_like(A)
    kf_R = exp.kf_R
    kf = KalmanFilter(A, B, kf_C, D, kf_Q, kf_R)
    # Initialize the zonotopes
    U = Zonotope.from_box(exp.control_lo, exp.control_up)
    W = exp.model.p_noise_dist
    reach = ReachableSet(A, B, U, W, max_step=exp.max_recovery_step )

    # init variables
    recovery_complete_index = exp.max_index
    x_cur_update = None

    
    
    logger.info(f" OPR-CL {exp.name} ")
    for i in range(0, exp.max_index + 1):
        assert exp.model.cur_index == i
        exp.model.update_current_ref(exp.ref[i])
        # attack here
        exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)

        if i == exp.attack_start_index - 1:
            logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            pass

        # state reconstruct
        if i == exp.recovery_index:
            logger.debug(f'recovery_index={i}, recovery_start_state={exp.model.cur_x}')
            # Obtains the inputs between instant of the attack beginning and the recovery beginning
            us = exp.model.inputs[exp.attack_start_index-1:exp.recovery_index]
            # Obtains the measurements.
            ys = (kf_C @ exp.model.states[exp.attack_start_index:exp.recovery_index + 1].T).T
            x_0 = exp.model.states[exp.attack_start_index-1]
            # Use a Kalman filter to estimate the state at the reconfiguration beginning
            x_res, P_res = kf.multi_steps(x_0, np.zeros_like(A), us, ys)
            x_cur_update = GaussianDistribution(x_res[-1], P_res[-1])
            logger.debug(f"reconstructed state={x_cur_update.miu=}, ground_truth={exp.model.cur_x}")
        # OPR-OL recovery begins
        # Estimate the probability distribution of the system state
        if exp.recovery_index < i < recovery_complete_index:
            x_cur_predict = GaussianDistribution(*kf.predict(x_cur_update.miu, x_cur_update.sigma, exp.model.cur_u))
            y = kf_C @ exp.model.cur_x
            x_cur_update = GaussianDistribution(*kf.update(x_cur_predict.miu, x_cur_predict.sigma, y))
            logger.debug(f"reconstructed state={x_cur_update.miu=}, ground_truth={exp.model.cur_x}")

        if i == recovery_complete_index:
            logger.debug(f'state after recovery={exp.model.cur_x}')
            pass
        # Use the OPR-OL to implement the recovery
        if exp.recovery_index <= i < recovery_complete_index:
            # Compute the reachable means
            reach.init(x_cur_update, exp.s)
            # Compute the recovery sequence
            k, X_k, D_k, z_star, alpha, P, arrive = reach.given_k(max_k=exp.max_recovery_step)
            print(f"{k=}, {z_star=}, {P=}")
            recovery_control_sequence = U.alpha_to_control(alpha)
            recovery_complete_index = i+k
            exp.model.evolve(recovery_control_sequence[0])
            print(f"{i=}, {recovery_control_sequence[0]=}")
        else:
            exp.model.evolve()
    
    profiling_data = {}
    profiling_data["recovery_complete_index"] = recovery_complete_index
    profiling_data["recovery_steps"] = recovery_complete_index - exp.recovery_index 
    return profiling_data

# ---------  attack + OPR_OL  -------------
def simulate_oprp_ol(exp, bl):
    '''
    function to simulate the OPR-OL strategy.
    exp: the experiment settings class 
    '''
    # required objects
    A = exp.model.sysd.A
    B = exp.model.sysd.B
    C = exp.model.sysd.C
    U = Zonotope.from_box(exp.control_lo, exp.control_up)
    W = exp.model.p_noise_dist
    reach = ReachableSet(A, B, U, W, max_step=exp.max_recovery_step )

    # init variables
    recovery_complete_index = exp.max_index
    x_cur_update = None
    exp_name = f" OPR-OL {exp.name} "
    logger.info(f"{exp_name:=^40}")
    for i in range(0, exp.max_index + 1):
        assert exp.model.cur_index == i
        exp.model.update_current_ref(exp.ref[i])
        # attack here
        exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)
        if i == exp.attack_start_index - 1:
            logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            pass

        # state reconstruct
        if i == exp.recovery_index:
            us = exp.model.inputs[exp.attack_start_index - 1:exp.recovery_index]
            x_0 = exp.model.states[exp.attack_start_index - 1]
            x_0 = GaussianDistribution(x_0, np.zeros((exp.model.n, exp.model.n)))
            reach.init(x_0, exp.s)
            x_res_point = reach.state_reconstruction(us)
            print('x_0=', x_res_point)

            reach.init(x_res_point, exp.s)
            k, X_k, D_k, z_star, alpha, P, arrive = reach.given_k(max_k=exp.k_given)
            recovery_complete_index = exp.recovery_index + k
            rec_u = U.alpha_to_control(alpha)
            
            print('k=', k, 'P=', P, 'z_star=', z_star, 'arrive=', arrive)
            print('D_k=', D_k)
            print('recovery_complete_index=', recovery_complete_index)
            print(rec_u)

        if exp.recovery_index <= i < recovery_complete_index:
            rec_u_index = i - exp.recovery_index
            u = rec_u[rec_u_index]
            exp.model.evolve(u)
        else:
            exp.model.evolve()
    
    profiling_data = {}
    profiling_data["recovery_complete_index"] = recovery_complete_index
    profiling_data["recovery_steps"] = recovery_complete_index - exp.recovery_index 

    return profiling_data

def plot_ts(strategies, exp, exp_rst):
    plt.figure(figsize=(5,3))
    max_t = 0
    for strategy in strategies:
        # 
        states = exp_rst[strategy]["states"]
        t = np.arange(0, len(states)) * exp.dt

        states = states[0:exp_rst[strategy]["recovery_complete_index"] + 1]
        t = t[0:exp_rst[strategy]["recovery_complete_index"] + 1]
        max_t = np.maximum(t[-1], max_t)
        #
        plt.plot(t, states[:, 0], color=colors[strategy], label=labels[strategy], linewidth=2)
        plt.plot(t[-1], states[-1, 0], color=colors[strategy], marker='*', linewidth=8)
    # Plot limits in x
    axes = plt.gca()
    x_lim = axes.get_xlim()
    plt.xlim(exp.x_lim[0], max_t + 0.5)

    # Plot limits in y
    plt.ylim(exp.y_lim)
    plt.vlines(exp.attack_start_index * exp.dt, exp.y_lim[0], exp.y_lim[1], color='red', linestyle='dashed', linewidth=2)
    plt.vlines(exp.recovery_index * exp.dt, exp.y_lim[0], exp.y_lim[1], colors='green', linestyle='dotted', linewidth=2)

    plt.fill_between(t + 0.5, t * 0 - exp.s.a, t * 0 - exp.s.b, facecolor='green', alpha=0.3)
    plt.ylabel(exp.y_label)
    plt.xlabel("Time [s]")
    plt.legend(ncol=2)


def sim_strategies(exps, plot_time_series):
    
    for exp in exps:

        strategies = {}
        # Declare strategies we want to simulate
        strategies = {  # 'none': simulate_no_recovery,
                        'ssr': simulate_ssr,
                        'oprp-close': simulate_oprp_cl,
                        'oprp-open': simulate_oprp_ol,
                        'lqr': simulate_LQR}
        
        
        result = {}
        result[exp.name] = {}
        exp_rst = result[exp.name]
        for strategy in strategies:
            exp.model.reset()
            exp_rst[strategy] = {}
            simulate = strategies[strategy]
            # Simulate strategy
            profiling_data = simulate(exp, strategy)
            # Store data
            save_exp_data(exp, exp_rst, strategy, profiling_data)
            recovery_complete_index = exp_rst[strategy]['recovery_complete_index']
            final_state = exp_rst[strategy]['states'][recovery_complete_index].tolist()
            

            
        # Begin plots 
        if plot_time_series:
            str_plot = ['ssr', 'oprp-open', 'lqr']
            plot_ts(str_plot, exp, exp_rst)
            str_plot = ['oprp-open', 'oprp-close']
            plot_ts(str_plot, exp, exp_rst)
            plt.show()
            


# Main function
def main():
    plot_time_series = True
    # Import the system and simulation settings. See the settings.py file
    # This classes have the simulation parameters such as the safe regions
    from settings import motor_speed_bias, quadruple_tank_bias, f16_bias, aircraft_pitch_bias, boeing747_bias, quadrotor_bias, rlc_circuit_bias
    exps = [motor_speed_bias]
    sim_strategies(exps, plot_time_series)

    
if __name__ == "__main__":
    main()
    
        