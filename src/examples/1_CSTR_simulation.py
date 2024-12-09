import numpy as np

# import built-in CSTR system
from cpsim.models.nonlinear.continuous_stirred_tank_reactor import CSTR

# simulation settings
max_index = 1000  # simulation steps
dt = 0.02  # sampling time
ref = [np.array([0, 300])] * (max_index + 1)  # reference/target state
noise = None  # noise settings
cstr_model = CSTR('test', dt, max_index, noise)  # simulation object

# simulation control loop
for i in range(0, max_index + 1):
    assert cstr_model.cur_index == i
    cstr_model.update_current_ref(ref[i])  # update reference/target state
    cstr_model.evolve()  # evolve the system

# print results
import matplotlib.pyplot as plt

# how to get the simulation results
t_arr = np.linspace(0, 10, max_index + 1)
ref = [x[1] for x in cstr_model.refs[:max_index + 1]]
y_arr = [x[1] for x in cstr_model.outputs[:max_index + 1]]
u_arr = [x[0] for x in cstr_model.inputs[:max_index + 1]]
# plot the state x[1]
plt.title('CSTR x[1]')
plt.plot(t_arr, y_arr, t_arr, ref)
plt.show()
