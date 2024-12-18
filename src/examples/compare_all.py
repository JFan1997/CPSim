import logging
import os
import sys
from copy import deepcopy
# from black import main

import matplotlib.pyplot as plt
import numpy as np
import cvxpy as cp
import time
import json

os.environ["RANDOM_SEED"] = '0'  # for reproducibility
from settings import cstr_bias, quad_bias, vessel_bias
from cpsim.observers.full_state_bound import Estimator
from cpsim.controllers.LP_cvxpy import LP
from cpsim.controllers.MPC_cvxpy import MPC
from cpsim.observers.full_state_bound_nonlinear import NonlinearEstimator
from cpsim.utils.linearizer import Linearizer

# from utils.observers.full_state_bound import Estimator
# from utils.controllers.LP_cvxpy import LP
# from utils.controllers.MPC_cvxpy import MPC
# from utils.observers.full_state_bound_nonlinear import NonlinearEstimator
# from utils.control.linearizer import Linearizer, analytical_linearize_cstr

# parse experiment simulator
import argparse

parser = argparse.ArgumentParser(description='Parse simulator.')
parser.add_argument('--sim', default='cstr_bias', help='cstr_bias, quad_bias, or vessel_bias')
args = parser.parse_args()
if args.sim == 'cstr_bias':
    exps = [cstr_bias]
elif args.sim == 'quad_bias':
    exps = [quad_bias]
elif args.sim == 'vessel_bias':
    exps = [vessel_bias]

baselines = ['none', 'lp', 'lqr', 'ssr', 'mpc']  # For LP, use cp.ECOS solver for cstr but cp.OSQP solver for quadrotor
# baselines = ['none', 'lqr', 'ssr', 'mpc'] # For LP, use cp.ECOS solver for cstr but cp.OSQP solver for quadrotor
if 'mpc' not in baselines:
    deadline_for_all_methods = 100
colors = {'none': 'red', 'lp': 'cyan', 'lqr': 'green', 'ssr': 'orange', 'mpc': 'blue'}
result = {}
overhead_dict = {}

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

for exp in exps:
    result[exp.name] = {}
    exp_rst = result[exp.name]

    #  =================  no_recovery  ===================
    # if 'none' in baselines:
    if True:
        bl = 'none'
        exp_name = f" {bl} {exp.name} "
        logger.info(f"{exp_name:=^40}")
        starttime = time.time()
        for i in range(0, exp.max_index + 1):
            assert exp.model.cur_index == i
            exp.model.update_current_ref(exp.ref[i])
            # attack here
            exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)
            if i == exp.attack_start_index - 1:
                logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            if i == exp.recovery_index:
                logger.debug(f'recovery_index={i}, recovery_start_state={exp.model.cur_x}')
            exp.model.evolve()

        overhead_dict[bl] = time.time() - starttime
        exp_rst[bl] = {}
        exp_rst[bl]['refs'] = deepcopy(exp.model.refs)
        exp_rst[bl]['states'] = deepcopy(exp.model.states)
        exp_rst[bl]['outputs'] = deepcopy(exp.model.outputs)
        exp_rst[bl]['inputs'] = deepcopy(exp.model.inputs)
        exp_rst[bl]['time'] = {}
        exp_rst[bl]['time']['recovery_complete'] = exp.max_index - 1

    #  =================  MPC_recovery  ===================
    # did not add maintainable time estimation, let it to be 3
    maintain_time = 3
    exp.model.reset()

    # init variables
    recovery_complete_index = np.inf
    rec_u = None
    linearize = Linearizer(ode=exp.model.ode, nx=exp.nx, nu=exp.nu, dt=exp.dt)
    non_est = NonlinearEstimator(exp.ode_imath, exp.dt)

    if 'mpc' in baselines:
        bl = 'mpc'
        exp_name = f" {bl} {exp.name} "
        logger.info(f"{exp_name:=^40}")
        starttime = time.time()
        for i in range(0, exp.max_index + 1):
            assert exp.model.cur_index == i
            exp.model.update_current_ref(exp.ref[i])
            # attack here
            exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)
            if i == exp.attack_start_index - 1:
                logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            if exp.recovery_index <= i < recovery_complete_index:
                if (i - exp.recovery_index) % exp.MPC_freq == 0:
                    logger.debug(f'recovery_index={i}, recovery_start_state={exp.model.cur_x}')

                    # State reconstruction
                    us = exp.model.inputs[exp.attack_start_index - 1:i]
                    xs = exp.model.states[exp.attack_start_index - 1:i + 1]
                    x_0 = exp.model.states[exp.attack_start_index - 1]
                    x_cur_lo, x_cur_up, x_cur = non_est.estimate(x_0, us, xs, exp.unsafe_states_onehot)
                    logger.debug(f'reconstructed state={x_cur}')

                    # deadline estimate only once
                    if i == exp.recovery_index:
                        safe_set_lo = exp.safe_set_lo
                        safe_set_up = exp.safe_set_up
                        control = exp.model.inputs[i - 1]
                        k = non_est.get_deadline(x_cur, safe_set_lo, safe_set_up, control, max_k=100)
                        deadline_for_all_methods = k
                        recovery_complete_index = exp.recovery_index + k
                        logger.debug(f'deadline={k}')
                    # maintainable time compute

                    # Linearize and Discretize
                    # Ad, Bd, cd = analytical_linearize_cstr(x_cur, exp.model.inputs[i-1], exp.dt)  # (2, 2) (2, 1) (2,)
                    Ad, Bd, cd = linearize.at(x_cur, exp.model.inputs[i - 1])

                    # get recovery control sequence
                    mpc_settings = {
                        'Ad': Ad, 'Bd': Bd, 'c_nonlinear': cd,
                        'Q': exp.Q, 'QN': exp.QN, 'R': exp.R,
                        'N': k + maintain_time + 1 - (i - exp.recovery_index),
                        # horizon (N) keeps decreasing in receding horizon MPC
                        'ddl': k - (i - exp.recovery_index), 'target_lo': exp.target_set_lo,
                        'target_up': exp.target_set_up,
                        'safe_lo': exp.safe_set_lo, 'safe_up': exp.safe_set_up,
                        'control_lo': exp.control_lo, 'control_up': exp.control_up,
                        'ref': exp.recovery_ref
                    }
                    mpc = MPC(mpc_settings)
                    _ = mpc.update(feedback_value=x_cur)
                    rec_u = mpc.get_full_ctrl()
                    rec_x = mpc.get_last_x()
                    logger.debug(f'expected recovery state={rec_x}')

                u = rec_u[(i - exp.recovery_index) % exp.MPC_freq]
                exp.model.evolve(u)
                print(f'after evolve - {exp.model.cur_x=}')
            elif recovery_complete_index <= i < recovery_complete_index + maintain_time + 1:
                if i == recovery_complete_index:
                    logger.debug(f'state after recovery={exp.model.cur_x}')
                    step = recovery_complete_index - exp.recovery_index
                    logger.debug(f'use {step} steps to recover.')

                u = rec_u[i - recovery_complete_index + 1]
                exp.model.evolve(u)
                print(f'after evolve - {exp.model.cur_x=}')
            else:
                exp.model.evolve()

        overhead_dict[bl] = time.time() - starttime
        exp_rst[bl] = {}
        exp_rst[bl]['states'] = deepcopy(exp.model.states)
        exp_rst[bl]['outputs'] = deepcopy(exp.model.outputs)
        exp_rst[bl]['inputs'] = deepcopy(exp.model.inputs)
        exp_rst[bl]['time'] = {}
        exp_rst[bl]['time']['recovery_complete'] = recovery_complete_index + maintain_time

    #  =================  LP_recovery  ===================
    exp.model.reset()

    # required objects
    ## Linearize about steady state and Linear estimator (may be required for staqte reconstruction comparison)
    linearize = Linearizer(ode=exp.model.ode, nx=exp.nx, nu=exp.nu, dt=exp.dt)
    A, B, c = linearize.at(exp.x_ss, exp.u_ss)
    # est = Estimator(A, B, max_k=100, epsilon=1e-7)

    # init variables
    recovery_complete_index = np.inf
    rec_u = None

    if 'lp' in baselines:
        bl = 'lp'
        exp_name = f" {bl} {exp.name} "
        logger.info(f"{exp_name:=^40}")
        starttime = time.time()
        for i in range(0, exp.max_index + 1):
            assert exp.model.cur_index == i
            exp.model.update_current_ref(exp.ref[i])
            # attack here
            exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)
            if i == exp.attack_start_index - 1:
                logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            if i == exp.recovery_index:
                logger.debug(f'recovery_index={i}, recovery_start_state={exp.model.cur_x}')

                # State reconstruction
                # us = exp.model.inputs[exp.attack_start_index - 1:exp.recovery_index]
                # x_0 = exp.model.states[exp.attack_start_index - 1]
                # x_cur_lo, x_cur_up, x_cur = est.estimate(x_0, us)
                us = exp.model.inputs[exp.attack_start_index - 1:i]
                xs = exp.model.states[exp.attack_start_index - 1:i + 1]
                x_0 = exp.model.states[exp.attack_start_index - 1]
                x_cur_lo, x_cur_up, x_cur = non_est.estimate(x_0, us, xs, exp.unsafe_states_onehot)
                logger.debug(f'reconstructed state={x_cur}')

                # # deadline estimate
                # safe_set_lo = exp.safe_set_lo
                # safe_set_up = exp.safe_set_up
                # control = exp.model.inputs[i - 1]
                # k = est.get_deadline(x_cur, safe_set_lo, safe_set_up, control, 100)
                k = deadline_for_all_methods
                recovery_complete_index = exp.recovery_index + k
                logger.debug(f'deadline={k}')

                # get recovery control sequence
                lp_settings = {
                    'Ad': A, 'Bd': B, 'c_nonlinear': c,
                    'N': k + maintain_time,
                    'ddl': k, 'target_lo': exp.target_set_lo, 'target_up': exp.target_set_up,
                    'safe_lo': exp.safe_set_lo, 'safe_up': exp.safe_set_up,
                    'control_lo': exp.control_lo, 'control_up': exp.control_up,
                    'ref': exp.recovery_ref,
                    'solver': cp.OSQP if exp.name == 'quad_bias' else cp.ECOS,
                }
                lp = LP(lp_settings)
                _ = lp.update(feedback_value=x_cur)
                rec_u = lp.get_full_ctrl()
                rec_x = lp.get_last_x()
                logger.debug(f'expected recovery state={rec_x}')

            if i == recovery_complete_index:
                logger.debug(f'state after recovery={exp.model.cur_x}')
                step = recovery_complete_index - exp.recovery_index
                logger.debug(f'use {step} steps to recover.')

            if exp.recovery_index <= i < recovery_complete_index + maintain_time:
                rec_u_index = i - exp.recovery_index
                u = rec_u[rec_u_index]
                exp.model.evolve(u)
            else:
                exp.model.evolve()

        overhead_dict[bl] = time.time() - starttime
        exp_rst[bl] = {}
        exp_rst[bl]['states'] = deepcopy(exp.model.states)
        exp_rst[bl]['outputs'] = deepcopy(exp.model.outputs)
        exp_rst[bl]['inputs'] = deepcopy(exp.model.inputs)
        exp_rst[bl]['time'] = {}
        exp_rst[bl]['time']['recovery_complete'] = recovery_complete_index + maintain_time

    #  =================  LQR_recovery  ===================
    # did not add maintainable time estimation, let it to be 3
    maintain_time = 3
    exp.model.reset()

    # init variables
    recovery_complete_index = np.inf
    rec_u = None

    if 'lqr' in baselines:
        bl = 'lqr'
        exp_name = f" {bl} {exp.name} "
        logger.info(f"{exp_name:=^40}")
        starttime = time.time()
        for i in range(0, exp.max_index + 1):
            assert exp.model.cur_index == i
            exp.model.update_current_ref(exp.ref[i])
            # attack here
            exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)
            if i == exp.attack_start_index - 1:
                logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            if i == exp.recovery_index:
                logger.debug(f'recovery_index={i}, recovery_start_state={exp.model.cur_x}')

                # State reconstruction
                # us = exp.model.inputs[exp.attack_start_index - 1:exp.recovery_index]
                # x_0 = exp.model.states[exp.attack_start_index - 1]
                # x_cur_lo, x_cur_up, x_cur = est.estimate(x_0, us)
                us = exp.model.inputs[exp.attack_start_index - 1:i]
                xs = exp.model.states[exp.attack_start_index - 1:i + 1]
                x_0 = exp.model.states[exp.attack_start_index - 1]
                x_cur_lo, x_cur_up, x_cur = non_est.estimate(x_0, us, xs, exp.unsafe_states_onehot)
                logger.debug(f'reconstructed state={x_cur}')

                # deadline estimate
                # safe_set_lo = exp.safe_set_lo
                # safe_set_up = exp.safe_set_up
                # control = exp.model.inputs[i - 1]
                # k = est.get_deadline(x_cur, safe_set_lo, safe_set_up, control, 100)
                k = deadline_for_all_methods
                recovery_complete_index = exp.recovery_index + k
                logger.debug(f'deadline={k}')
                # maintainable time compute

                # get recovery control sequence
                Q_lqr = np.diag([1] * exp.nx)
                QN_lqr = np.diag([1] * exp.nx)
                R_lqr = np.diag([1])
                # R_lqr = np.diag([1]*exp.nu)
                lqr_settings = {
                    'Ad': A, 'Bd': B, 'c_nonlinear': c,
                    'Q': Q_lqr, 'QN': QN_lqr, 'R': R_lqr,
                    # 'Q': exp.Q, 'QN': exp.QN, 'R': exp.R,
                    'N': k + maintain_time,
                    'ddl': k, 'target_lo': exp.target_set_lo, 'target_up': exp.target_set_up,
                    'safe_lo': exp.safe_set_lo, 'safe_up': exp.safe_set_up,
                    'control_lo': exp.control_lo, 'control_up': exp.control_up,
                    'ref': exp.recovery_ref
                }
                lqr = MPC(lqr_settings)
                _ = lqr.update(feedback_value=x_cur)
                rec_u_lqr = lqr.get_full_ctrl()
                rec_x = lqr.get_last_x()
                logger.debug(f'expected recovery state={rec_x}')

            if i == recovery_complete_index:
                logger.debug(f'state after recovery={exp.model.cur_x}')
                step = recovery_complete_index - exp.recovery_index
                logger.debug(f'use {step} steps to recover.')

            if exp.recovery_index <= i < recovery_complete_index + maintain_time:
                rec_u_index = i - exp.recovery_index
                u = rec_u_lqr[rec_u_index]
                exp.model.evolve(u)
            else:
                exp.model.evolve()

        overhead_dict[bl] = time.time() - starttime
        exp_rst[bl] = {}
        exp_rst[bl]['states'] = deepcopy(exp.model.states)
        exp_rst[bl]['outputs'] = deepcopy(exp.model.outputs)
        exp_rst[bl]['inputs'] = deepcopy(exp.model.inputs)
        exp_rst[bl]['time'] = {}
        exp_rst[bl]['time']['recovery_complete'] = recovery_complete_index + maintain_time

    #  =================  Software_sensor_recovery  ===================
    exp.model.reset()


    # required objects
    def in_target_set(target_lo, target_hi, x_cur):
        res = True
        for i in range(len(x_cur)):
            if target_lo[i] > x_cur[i] or target_hi[i] < x_cur[i]:
                res = False
                break
        return res


    est = Estimator(A, B, max_k=100, epsilon=1e-7, c=c)

    # init variables
    recovery_complete_index = np.inf
    last_predicted_state = None

    if 'ssr' in baselines:
        bl = 'ssr'
        exp_name = f" {bl} {exp.name} "
        logger.info(f"{exp_name:=^40}")
        starttime = time.time()
        for i in range(0, exp.max_index + 1):
            assert exp.model.cur_index == i
            exp.model.update_current_ref(exp.ref[i])
            # attack here
            exp.model.cur_feedback = exp.attack.launch(exp.model.cur_feedback, i, exp.model.states)
            if i == exp.attack_start_index - 1:
                logger.debug(f'trustworthy_index={i}, trustworthy_state={exp.model.cur_x}')
            if i == exp.recovery_index:
                logger.debug(f'recovery_index={i}, recovery_start_state={exp.model.cur_x}')

                # State reconstruction
                # us = exp.model.inputs[exp.attack_start_index - 1:exp.recovery_index-1]
                # x_0 = exp.model.states[exp.attack_start_index - 1]
                # x_cur = est.estimate_wo_bound(x_0, us)
                us = exp.model.inputs[exp.attack_start_index - 1:i]
                xs = exp.model.states[exp.attack_start_index - 1:i + 1]
                x_0 = exp.model.states[exp.attack_start_index - 1]
                x_cur_lo, x_cur_up, x_cur = non_est.estimate(x_0, us, xs, exp.unsafe_states_onehot)
                logger.debug(f'one before reconstructed state={x_cur}')
                last_predicted_state = deepcopy(x_cur)

            if exp.recovery_index <= i <= recovery_complete_index:
                # check if it is in target set
                # if in_target_set(exp.target_set_lo, exp.target_set_up, last_predicted_state):
                #     recovery_complete_index = i
                #     logger.debug('state after recovery={exp.model.cur_x}')
                #     step = recovery_complete_index - exp.recovery_index
                #     logger.debug(f'use {step} steps to recover.')

                us = np.array([exp.model.inputs[i - 1]])
                xs = exp.model.states[i - 1:i + 1]
                x_0 = last_predicted_state
                # x_cur = est.estimate_wo_bound(x_0, us)
                # us = exp.model.inputs[exp.attack_start_index - 1:i]
                # xs = exp.model.states[exp.attack_start_index - 1:i+1]
                # x_0 = exp.model.states[exp.attack_start_index - 1]
                # x_cur_lo, x_cur_up, x_cur = non_est.estimate(x_0, us, xs, exp.unsafe_states_onehot)
                x_cur = est.estimate_wo_bound(x_0, us)
                exp.model.cur_feedback = x_cur  # exp.model.sysd.C @ x_cur
                last_predicted_state = deepcopy(x_cur)
                # print(f'{exp.model.cur_u}')
            exp.model.evolve()

        overhead_dict[bl] = time.time() - starttime
        exp_rst[bl] = {}
        exp_rst[bl]['states'] = deepcopy(exp.model.states)
        exp_rst[bl]['outputs'] = deepcopy(exp.model.outputs)
        exp_rst[bl]['inputs'] = deepcopy(exp.model.inputs)
        exp_rst[bl]['time'] = {}
        exp_rst[bl]['time']['recovery_complete'] = exp.max_index - 1
        # print(f'{recovery_complete_index}')

    # ==================== plot =============================
    plt.rcParams.update({'font.size': 24})  # front size
    fig = plt.figure(figsize=(10, 5))

    # plot reference
    t_arr = np.linspace(0, exp.dt * exp.max_index, exp.max_index + 1)[:exp.max_index]
    ref = [x[exp.ref_index] for x in exp_rst['none']['refs'][:exp.max_index]]
    plt.plot(t_arr, ref, color='grey', linestyle='dashed')
    # plot common part (before recovery)
    t_arr_common = t_arr[:exp.recovery_index + 1]
    output = [x[exp.output_index] for x in exp_rst['none']['outputs'][:exp.recovery_index + 1]]
    plt.plot(t_arr_common, output, color='black')
    # plot attack / recovery
    if exp.y_lim:
        plt.vlines(exp.attack_start_index * exp.dt, exp.y_lim[0], exp.y_lim[1], colors='red', linestyle='dashed',
                   linewidth=2)
        plt.vlines(exp.recovery_index * exp.dt, exp.y_lim[0], exp.y_lim[1], colors='green', linestyle='dotted',
                   linewidth=2)

        recovery_complete_index = exp.recovery_index + deadline_for_all_methods
        # print(recovery_complete_index)
        plt.vlines((recovery_complete_index) * exp.dt, exp.y_lim[0], exp.y_lim[1], colors='blue', linestyle='dotted',
                   linewidth=2)
        plt.vlines((recovery_complete_index + maintain_time) * exp.dt, exp.y_lim[0], exp.y_lim[1], colors='black',
                   linestyle='dotted', linewidth=2)
    # strip
    cnt = len(t_arr)
    y1 = [exp.strip[0]] * cnt
    y2 = [exp.strip[1]] * cnt
    plt.fill_between(t_arr, y1, y2, facecolor='green', alpha=0.1)

    for bl in baselines:
        end_time = exp_rst[bl]['time']['recovery_complete']
        t_arr_tmp = t_arr[exp.recovery_index:end_time + 1]
        output = [x[exp.output_index] for x in exp_rst[bl]['outputs'][exp.recovery_index:end_time + 1]]
        # output = [x[exp.output_index] for x in exp_rst[bl]['states'][exp.recovery_index:end_time + 1]]
        plt.plot(t_arr_tmp, output, color=colors[bl], label=bl)

    if exp.y_lim:
        plt.ylim(exp.y_lim)
    if exp.x_lim:
        updated_x_lim = (exp.x_lim[0], exp.dt * (recovery_complete_index + maintain_time))
        plt.xlim(updated_x_lim)

    # plt.legend()
    plt.ylabel(exp.y_label)
    plt.xlabel('Time [sec]', loc='right', labelpad=-55)
    plt.legend()
    os.makedirs(f'fig', exist_ok=True)
    plt.savefig(f'fig/{exp.name}_all.png', format='png', bbox_inches='tight')

    with open("fig/overhead.txt", "w") as file:
        file.write(json.dumps(overhead_dict))
    # plt.show()
