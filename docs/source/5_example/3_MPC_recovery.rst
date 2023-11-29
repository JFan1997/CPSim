Numerical Simulators
~~~~~~~~~~~~~~~~~~~~


.. First, we aim to evaluate the recovery performance of different baseline recovery controllers. We only require modifying the configuration file rather than writing simulation code.
In this file, we choose a benchmark plant, CSTR shown in the following figure, controlled by a PID controller.
Also, we define the bias sensor attack that subtracts 25K from the temperature sensor feedback starting from the ninth second.
The detector identifies the attack at the 10th second, and triggers the recovery controllers.
Baseline recovery controllers include (i) no recovery method (none), (ii) software-sensor-based recovery (ssr [1]_), (iii) linear-quadratic-regulator-based recovery (lqr [2]_), and (vi) data-predictive recovery (mpc [3]_).

First, we aim to evaluate the recovery performance of different baseline recovery controllers. We only require modifying the configuration file rather than writing simulation code.
In this file, we use three benchmark plants, CSTR, quadrotor and Vessel, controlled by a PID controller.
Also, we define the bias sensor attack.
The detector identifies the attack after the attack happends and triggers the recovery controllers.
Baseline recovery controllers include (i) no recovery method (none), (ii) software-sensor-based recovery (ssr [1]_), (iii) linear-quadratic-regulator-based recovery (lqr [2]_), and (vi) data-predictive recovery (mpc [3]_).



The following figure plots the results from the simulator. From the curve, we can intuitively analyze the recovery performance of each baseline recovery controller.

.. image:: images/5_example/performance.png
   :width: 650 px
   :align: center
   :alt: Attack Recovery Performance for Baselines

We conduct a sensitivity analysis by varying bias values and noise on the CSTR benchmark. The findings are displayed in the two figures below.
.. image:: images/5_example/sensitivity_bias_CSTR.png
   :width: 650 px
   :align: center
   :alt: Sensitivity analysis on CSTR benchmark

.. image:: images/5_example/sensitivity_noise_CSTR.png
   :width: 650 px
   :align: center
   :alt: Sensitivity analysis to noise on CSTR benchmark

We also provide the sensitivity analysis by the noise with varying upper bound on the quadrotor benchmark.
.. image:: images/5_example/sensitivity_noise_Quadrotor.png
   :width: 650 px
   :align: center
   :alt: Sensitivity analysis to noise on CSTR benchmark

We provide the source code of above figures.
.. literalinclude:: ../../../src/examples/CSTR_bias.py
    :language: python
    :name: CSTR_bias.py
    :caption: Bias attack on the CSTR benchmark
    :linenos:
Reference:

.. [1] `F. Kong, M. Xu, J. Weimer, O. Sokolsky, and I. Lee, "Cyber-physical system checkpointing and recovery," in 2018 ACM/IEEE 9th International Conference on Cyber-Physical Systems (ICCPS), IEEE, 2018, pp. 22-31.`
.. [2] `L. Zhang, P. Lu, F. Kong, X. Chen, O. Sokolsky, and I. Lee, "Real-time attack-recovery for cyber-physical systems using linear-quadratic regulator," ACM Trans. Embed. Comput. Syst., vol. 20, no. 5s, Sep. 2021, issn: 1539-9087. doi: 10.1145/3477010.[Online]. Available: https://doi.org/10.1145/3477010.`
.. [3] `L. Zhang, K. Sridhar, M. Liu, et al., "Real-time data-predictive attack-recovery for complex cyber-physical systems," in 2023 IEEE 29th Real-Time and Embedded Technology and Applications Symposium (RTAS), 2023.`