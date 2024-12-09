Custom Components
=================


Add New Models
--------------
We define system models in the ``cpsim.models`` subpackage.
Here are steps to add a new model:

1. Define system dynamics (state-space model for linear systems, ODE for nonlinear systems).

.. code-block:: python

    def system_name(t, x, u):
        # ODE here
        return dx   # dx is the derivative of x, shape (n,)


2. Define a controller for the system, and we can use built-in controllers.

.. code-block:: python

    class Controller:
        def __init__(self, dt):
            self.dt = dt
            # init controller here
        def update(self, ref: np.ndarray, feedback_value: np.ndarray, current_time) -> np.ndarray:
            # calculate one step of control input here
            return u   # shape: (m,)
        def set_control_limit(self, control_lo, control_up):
            # set control limits here, the maximum range of control input
            pass
        def clear(self):
            # reset controller here if needed
            pass


3. Define the simulation class integrating the dynamics and controller

.. code-block:: python

    class SystemName(Simulator):
        def __init__(self, name, dt, max_index, noise=None):
            super().__init__('SystemName ' + name, dt, max_index)
            self.nonlinear(ode=system_name, n=2, m=1, p=2)  # num of states,control inputs,outputs
            controller = Controller(dt)
            settings = {
                'init_state': x_0,  # initial state
                'feedback_type': 'state',  # 'state'or'output', you must define C if 'output'
                'controller': controller
            }
            if noise:
                settings['noise'] = noise  # noise settings
            self.sim_init(settings)


For complete examples, please refer to the source code of the built-in system models.

* Linear systems, such as `DC Motor Speed <https://github.com/lion-zhang/CPSim/blob/main/src/cpsim/models/linear/motor_speed.py>`_.
* Nonlinear systems, such as `Inverted Pendulum <https://github.com/lion-zhang/CPSim/blob/main/src/cpsim/models/nonlinear/inverted_pendulum.py>`_.


Add New Controllers
-------------------
We can define a new controller by inheriting the ``Controller`` class.

.. literalinclude:: ../../src/cpsim/controllers/controller_base.py
   :language: python
   :linenos:
   :caption: cpsim/controllers/controller_base.py

Please refer to the source code of the built-in controllers, such as `LQR controller <https://github.com/lion-zhang/CPSim/blob/main/src/cpsim/controllers/LQR.py>`_, for more details.
