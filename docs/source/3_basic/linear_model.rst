+------------------------------------------+------------+--------------------+-------------+
| System                                   |  # States  |  # control inputs  |  # outputs  |
+==========================================+============+====================+=============+
| `Motor Speed`_                           |      2     |           1        |       1     |
+------------------------------------------+------------+--------------------+-------------+
| `Next  Model`_                           |      2     |           1        |       1     |
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


.. _Next Model:
2. Next Model [2]_
~~~~~~~~~~~~~~~~~~~~
intro here


References:

.. [1] `Control Tutorials for MATLAB and Simulink - Motor Speed: System Modeling. (n.d.). <https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed§ion=SystemModeling>`_
.. [2] Reference 2