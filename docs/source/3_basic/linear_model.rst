+------------------------------------------+------------+--------------------+-------------+
| System                                   |  # States  |  # control inputs  |  # outputs  |
+==========================================+============+====================+=============+
| `Motor Speed`_                           |      2     |           1        |       1     |
+------------------------------------------+------------+--------------------+-------------+
| `RLC Circuit`_                           |      2     |           1        |       1     |
+------------------------------------------+------------+--------------------+-------------+
| `Aircraft Pitch`_                        |      3     |           1        |       1     |
+------------------------------------------+------------+--------------------+-------------+
| `Quadrotor`_                             |      12    |           4        |       1     |
+------------------------------------------+------------+--------------------+-------------+
| `RLC Circuit`_                           |      2     |           1        |       1     |
+------------------------------------------+------------+--------------------+-------------+
| `RLC Circuit`_                           |      2     |           1        |       1     |
+------------------------------------------+------------+--------------------+-------------+

.. _Motor Speed:

1. Motor Speed [1]_
~~~~~~~~~~~~~~~~~~~~
A common actuator in control systems is the DC motor. It directly provides rotary motion and, coupled with wheels or drums and cables, can provide translational motion. The electric equivalent circuit of the armature and the free-body diagram of the rotor are shown in the following figure.

.. image:: images/3_basic/motor.png
   :width: 300 px
   :align: center
   :alt: DC motor

State-space model:

.. math::

    \begin{gathered}
    \frac{d}{d t}\left[\begin{array}{c}
    \dot{\theta} \\
    i
    \end{array}\right]=\left[\begin{array}{cc}
    -\frac{b}{J} & \frac{K}{J} \\
    -\frac{K}{L} & -\frac{R}{L}
    \end{array}\right]\left[\begin{array}{l}
    \dot{\theta} \\
    i
    \end{array}\right]+\left[\begin{array}{c}
    0 \\
    \frac{1}{L}
    \end{array}\right] V \\
    y=\left[\begin{array}{ll}
    1 & 0
    \end{array}\right]\left[\begin{array}{l}
    \dot{\theta} \\
    i
    \end{array}\right]
    \end{gathered}


where :math:`\theta` is the angular position of the motor shaft, :math:`i` is the current through the motor, :math:`V` is the voltage applied to the motor, :math:`J=0.01 kg.m^2` is the moment of inertia of the motor, :math:`b=0.1 N.m.s` is the motor viscous friction constant, :math:`K=0.01 N.m/Amp` is the motor torque constant, and :math:`R=1 Ohm and :math:`L0.5 H` are the electrical resistance and inductance of the motor, respectively.


.. _RLC Circuit:

2. RLC Circuit [2]_
~~~~~~~~~~~~~~~~~~~~
A basic RLC circuit contains a resistor, an inductor, and a capacitor connected in series. An adjustable voltage source is connected to form a closed loop circuit. The system dynamics are modeled by the right equation such that state :math:`x_1`
denotes the voltage across the capacitor and state :math:`x_2` denotes the electric current in the loop. The
control input :math:`u` is considered the voltage of the voltage source.

State-space model:

.. math::

   \begin{gathered}
   \left[\begin{array}{c}
   \dot{x}_1 \\
   \dot{x}_2
   \end{array}\right]=\left[\begin{array}{cc}
   0 & \frac{1}{C} \\
   -\frac{1}{L} & -\frac{R}{L}
   \end{array}\right]\left[\begin{array}{l}
   x_1 \\
   x_2
   \end{array}\right]+\left[\begin{array}{c}
   0 \\
   \frac{1}{L}
   \end{array}\right] u
   \end{gathered}

.. _Aircraft Pitch:

3. Aircraft Pitch [2]_
~~~~~~~~~~~~~~~~~~~~~~
The system ODE describes the longitudinal dynamics of motion for the aircraft. The :math:`x_1` denotes the angle of attack,
:math:`x_2` denotes the pitch rate, and :math:`x_3` denotes the pitch angle. The control input :math:`u` is the elevator deflection angle.

State-space model:

.. math::
   \begin{gathered}
   \left[\begin{array}{l}
   \dot{x}_1 \\
   \dot{x}_2 \\
   \dot{x}_3
   \end{array}\right]=\left[\begin{array}{ccc}
   -0.313 & 56.7 & 0 \\
   -0.0139 & -0.426 & 0 \\
   0 & 56.7 & 0
   \end{array}\right]\left[\begin{array}{l}
   x_1 \\
   x_2 \\
   x_3
   \end{array}\right]+\left[\begin{array}{c}
   0.232 \\
   0.0203 \\
   0
   \end{array}\right] u
   \end{gathered}


.. _Quadrotor:

4. Quadrotor [3]_
~~~~~~~~~~~~~~~~~
We consider a linear quadrotor model described in [3]_. The system consists of 12 state variables: :math:`(x, y, z)`
denotes the (relative) position, :math:`(φ, θ, ψ)` denotes the angles of pitch, yaw and roll respectively, :math:`(u, v, w)` and :math:`(p, q, r)` are the velocity and angular velocity of the quadrotor. The controller
produces 4 inputs which are ft: total thrust, and :math:`[τ_x, τ_y, τ_z]` control torques caused by differences of rotor speeds.


References:

.. [1] `Control Tutorials for MATLAB and Simulink - Motor Speed: System Modeling. (n.d.). <https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed§ion=SystemModeling>`_
.. [2] `Reference 2`_
.. [3] `Quadrotor control:modeling,nonlinear controldesign,and simulation. <https://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf>`_