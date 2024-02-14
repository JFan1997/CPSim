1. Quadrotor
~~~~~~~~~~~~
The nonlinear dynamics of the quadrotor relate its attitude and position with 12 states same as the linear quadrotor model above.

.. math::

   \begin{gathered}
    \begin{array}{ll}
    \dot{\phi}=w_\phi, & \dot{w}_\phi=\frac{\tau_\phi}{I_x}+\dot{\theta} \dot{\psi}\left(\frac{I_y-I_z}{I_x}\right) \\
    \dot{\theta}=w_\theta, & \dot{w}_\theta=\frac{\tau_\theta}{I_y}+\dot{\phi} \dot{\psi}\left(\frac{I_z-I_x}{I_y}\right) \\
    \dot{\psi}=w_\psi, & \dot{w}_\psi=\frac{\tau_\psi}{I_z}+\dot{\phi} \dot{\theta}\left(\frac{I_x-I_y}{I_z}\right) \\
    \dot{x}=v_x, & \dot{v}_x=\frac{f_t}{m}(\cos (\phi) \sin (\theta) \cos (\psi)+\sin (\phi) \sin (\psi) \\
    \dot{y}=v_y, & \dot{v}_y=\frac{f_t}{m}(\cos (\phi) \sin (\theta) \sin (\psi)-\sin (\phi) \cos (\psi)) \\
    \dot{z}=v_z, & \dot{v}_z=\frac{f_t}{m} \cos (\phi) \cos (\theta)-g
    \end{array}
   \end{gathered}



2. Vessel
~~~~~~~~~
The vessel dynamics characterize a standard ship with 3 degrees of freedom. The state-space model is derived from the Marine Cybernetics lecture notes, and the high-fidelity simulator is based on the NTNU AUR-lab's development, incorporating Matlab scripts from the VESSELS catalog of the MSS toolbox. The simulator replicates the vessel with essential ship components such as the engine, propeller, and rudder, treating the entire model as a system. The system comprises 8 states, including east and north positions, yaw, their velocities, the angular shaft speed of the propeller, and the DC motor's current.

.. image:: images/3_basic/vessel_cybership.png
   :width: 300 px
   :align: center
   :alt: vessel

References:

.. [1] `Marine cybernetics, towards autonomous marine operations and systems. <https://folk.ntnu.no/assor/Public/2018-08-20%20marcyb.pdf>`_
.. [2] `Handbook of marine craft hydrodynamics and motion control. <https://onlinelibrary.wiley.com/doi/book/10.1002/9781119994138>`_

3. Pendulum
~~~~~~~~~~~
The dynamics of the inverted pendulum on a cart is described by a nonlinear model. The objective is to stabilize the inverted pendulum towards the upright position by applying force to the cart.
The cart position is denoted by x, velocity by v, pendulum angle by θ, angular velocity by ω, pendulum mass by m, cart mass by M, pendulum arm by L, gravitational acceleration by g, friction damping on the cart by δ, and control force applied to the cart by u.

.. math::
    \begin{gathered}
     \begin{array}{ll}
     \dot{x}=v, \\
     \dot{v}=\frac{-m^2 L^2 g \cos (\theta) \sin (\theta)+m L^2\left(m L \omega^2 \sin (\theta)-\delta v\right)+m L^2 u}{m L^2\left(M+m\left(1-\cos (\theta)^2\right)\right)} \\
     \dot{\theta}=\omega  \\
     \dot{\omega}=\frac{(m+M) m g L \sin (\theta)-m L \cos (\theta)\left(m L \omega^2 \sin (\theta)-\delta v\right)-m L \cos (\theta) u}{m L^2\left(M+m\left(1-\cos (\theta)^2\right)\right)}
     \end{array}
   \end{gathered}

.. image:: images/3_basic/pendulum.png
   :width: 300 px
   :align: center
   :alt: pendulum

4. CSTR
~~~~~~~~~~~
In the dynamics of a CSTR, the exothermic
reaction of species A → B is analyzed using the concentration
of A (:math:`C_A`) and the reactor temperature (:math:`T`) as states. The
control input is determined by the temperature of the cooling jacket
(:math:`T_C`). The precise dynamics, considering different system parameters are outlined below.

.. math::
    \begin{gathered}
     \begin{array}{ll}
     V\dot{C}_A & =q\left(C_{a f}-C_A\right)-k_0 \exp \left(\frac{-E}{R T}\right) V C_A \\
     \rho C_p V \dot{T} & =\rho C_p q\left(T_f-T\right)+\Delta H k_0 \exp \left(\frac{-E}{R T}\right) V C_A \\
     & +U A\left(T_C-T\right)
     \end{array}
    \end{gathered}

.. image:: images/5_example/cstr.png
   :width: 400 px
   :align: center
   :alt: CSTR

References:

.. [1] `A nonlinear model library for dynamics and control. <https://d1wqtxts1xzle7.cloudfront.net/36872378/nonlinearmodellibrary-libre.pdf?1425595138=&response-content-disposition=inline%3B+filename%3DA_Nonlinear_Model_Library_for_Dynamics_a.pdf&Expires=1701210799&Signature=HIVd2ORsI4t3TogTf0ihSfQ~eHxnSZC2zxiAqqiJU5-t8E1AcS-D7IP2qnTSg9uONV~fBWmwhVwr5YHq3PrzZTchEMWIllVNI3fcMeTMRFU~x-2~yq5q1-TqxQmb0D-sGmVZefapQzDRLWsePOUWFK3rz9kcBljqboInK3Z0qFHiCFK2QPtTyL8hY1aMnkaNuQZkn2lbdQ7wb0vt9lA24~GqBq6yQ9-RbpkRx~Bmd9JDAvRTsb4x1zNEyTx4LwuBDtSgF16cn1hFq1rApw3OhPJOFFQOvvez7LkGqizBBowWsUFxlVMviiTan1a4K0vh1vNrckgPe~kheLB9j7ZhHQ__&Key-Pair-Id=APKAJLOHF5GGSLRBV4ZA>`_

