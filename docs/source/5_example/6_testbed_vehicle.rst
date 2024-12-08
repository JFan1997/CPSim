Robotic Vehicle Testbed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Autonomous vehicles are a type of CPS that rely on sensor information to perform tasks such as path tracking and lane keeping. We built scaled autonomous vehicles, measuring 24 cm in length and 19 cm in width, as testbeds to evaluate the proposed attack detection and recovery methods.

.. image:: images/5_example/a2_testbed.png
   :width: 400 px
   :align: center
   :alt: Robotic Vehicle Testbed

Autonomous vehicles sense states and environments, make decisions, and control mobility. Our robotic vehicle testbeds, whose hardware architecture is shown in the following figure, simulate these functions through the following stages:

(i) Perception: Our testbed is equipped with an Inertial Measurement Unit (IMU), Ultra-wideband (UWB), and encoder sensors that measure attitude, position, and velocity, respectively. We can also use cameras and LiDAR to collect additional environmental data. However, these sensors are vulnerable to sensor attacks.

(ii) Decision Making: A Raspberry Pi with Robot Operating System (ROS2) serves as the main controller. It collects sensor data, estimates vehicle states, and generates control signals. The system uses different controllers for longitudinal and lateral control. For cruise control, a PID controller stabilizes the testbed's velocity based on encoder feedback. For lane keeping, a Stanley Controller uses the front axle as its reference point, considering both heading error and cross-track error. We can also deploy the proposed attack detection and recovery algorithms on this system.

(iii) Control: An STM32 microcontroller with FreeRTOS system receives control signals from the Raspberry Pi through a Universal Asynchronous Receiver/Transmitter (UART) protocol. Running a real-time operating system, it performs time-sensitive tasks such as generating Pulse Width Modulation (PWM) signals to drive actuators.

(iv) Actuator: The actuator stage includes components such as motors and servos that execute vehicle movements according to control signals.


.. image:: images/5_example/hardware_arch.png
   :width: 400 px
   :align: center
   :alt: Hardware Architecture of Robotic Vehicle Testbed


To demonstrate the toolbox usage, we show a real-time attack recovery on SVL high-fidelity simulator.
We implement the LQR-based attack recovery method [2]_ on the robotic vehicle testbed, and the design is illustrated in this figure:

.. image:: images/5_example/testbed_recovery_arch.png
   :width: 550 px
   :align: center
   :alt: Real-time Attack Recovery Implementation on Robotic Vehicle Testbed

Autonomous vehicles perform lateral control to track paths provided by path planning modules. High-fidelity models of vehicle dynamics are complex, non-linear, and discontinuous. However, to reduce computational complexity, path tracking controllers typically consider a simplified lateral dynamics model of the vehicle (as shown in the following equation). This model approximates dynamic effects to improve tracking performance.

.. math::
   :nowrap:

    \begin{equation}\label{equ:lanekeeping}
        \frac{d}{d t} \left[\begin{array}{c}
    y \\
    \dot{y} \\
    \psi \\
    \dot{\psi}
    \end{array}\right]=\left[\begin{array}{cccc}
    0 & 1 & 0 & 0 \\
    0 & \frac{-\left(c_f+c_r\right)}{m v_x} & 0 & \frac{\left(l_r c_r-l_f c_f\right)}{m v_x}-v_x \\
    0 & 0 & 0 & 1 \\
    0 & \frac{l_r c_r-l_f c_f}{I_z v_x} & 0 & \frac{-\left(\ell_f^2 c_f+\ell_r^2 c_r\right)}{I_z v_x}
    \end{array}\right]\left[\begin{array}{c}
    y \\
    \dot{y} \\
    \psi \\
    \dot{\psi}
    \end{array}\right]+\left[\begin{array}{c}
    0 \\
    \frac{c_f}{m} \\
    0 \\
    \frac{l_f c_f}{I_z}
    \end{array}\right] \delta
    \end{equation}


Here, :math:`c_f` and :math:`c_r` represent cornering stiffness for the front and rear tires; :math:`l_f` and :math:`l_r` are the distances from the front and rear tires to the vehicle's center of gravity;
:math:`I_z` is the vehicle's moment of inertia; and :math:`v_x` is the longitudinal velocity. The system states are lateral distance from the path (:math:`y`), lateral error rate (:math:`\dot{y}`),
yaw error (:math:`\psi`), and yaw error rate (:math:`\dot{\psi}`). The control input is the front wheel steering angle ( :math:`\delta`).

To achieve path tracking, we implement the Stanley lateral controller in ROS. The control law is expressed as :math:`\delta(t)=\psi(t)+\tan ^{-1}\left(\frac{k y(t)}{v_x(t)}\right)`.
The yaw error (:math:`\psi`) is obtained from the IMU sensor, and the lateral distance from the path (:math:`y`) is obtained from the UWB sensor for indoor use. In the absence of sensor attacks, the controller can perform path tracking tasks with good performance.
The attacker launches an attack on the IMU sensor, reducing the value of :math:`\psi` by 0.60 radians from the start of the attack, with a detection delay of 60 control steps. Figure (a) shows the attack result as detected by the detector.
Subsequently, our proposed method begins controlling the vehicle back to the safe zone, as shown in Figure (c).

.. image:: images/5_example/testbed_result.jpg
   :width: 700 px
   :align: center
   :alt: Recovery demonstration from our testbed

Finally, we show the demonstration video here.

.. raw:: html

    <p align="center"><iframe width="700" height="350" src="https://www.youtube.com/embed/OW5P37aairs?rel=0" title="Demonstration on testbed" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe></p>
