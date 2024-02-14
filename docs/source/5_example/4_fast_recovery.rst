Fast Attack Recovery for Stochastic Cyber-Physical Systems
~~~~~~~~~~~~~~~~~~~~

We present the simulations for the Fast Attack Recovery for Stochastic Cyber-Physical Systems paper [4]_. 
We propose the solution to the Optimal probabilistic recovery (OPR). 
We use the linear model of a motor as an example. 
For the experiments in the paper, we simulate a bias attack against the motor angular speed measurement by adding a bias of -1m/s.
The adversary deploys the attack at time 3s after beginning the simulation, and the detector identifies the attack one second later.
This alarm triggers the recovery strategies.


We compare the performance of our solution, called the open loop OPR (OPR-OL), with the LQR-based recovery [2]_, and virtual sensors.
We then obtain the next Figure:

.. image:: images/4_fast_recovery/timeseries_motor_speed_3_strats.jpg
   :width: 650 px
   :align: center
   :alt: Attack Recovery Performance for Baselines for the motor system

We also include the simulations to improve our solution by incorporating the non-compromised sensors into the recovery. 
We call this strategy the partially closed loop OPR (OPR-PCL).
We compare the OPR-OL and OPR-PCL:

.. image:: images/4_fast_recovery/timeseries_motor_speed_opr.jpg
   :width: 650 px
   :align: center
   :alt: Attack Recovery Performance for Baselines for the motor system

We provide the code to obtain previous figures.

.. literalinclude:: ../../src/examples/rtas/motor_example.py
    :language: python
    :linenos:
    :caption: Bias attack on the CSTR benchmark

An user can change the attack characteristics (e.g., detection delay).

.. literalinclude:: ../../src/examples/rtas/settings.py
    :language: python
    :linenos:
    :caption: Bias attack on the CSTR benchmark

Reference:

.. [1] `F. Kong, M. Xu, J. Weimer, O. Sokolsky, and I. Lee, "Cyber-physical system checkpointing and recovery," in 2018 ACM/IEEE 9th International Conference on Cyber-Physical Systems (ICCPS), IEEE, 2018, pp. 22-31.`
.. [2] `L. Zhang, P. Lu, F. Kong, X. Chen, O. Sokolsky, and I. Lee, "Real-time attack-recovery for cyber-physical systems using linear-quadratic regulator," ACM Trans. Embed. Comput. Syst., vol. 20, no. 5s, Sep. 2021, issn: 1539-9087. doi: 10.1145/3477010.[Online]. Available: https://doi.org/10.1145/3477010.`
.. [3] `L. Zhang, K. Sridhar, M. Liu, et al., "Real-time data-predictive attack-recovery for complex cyber-physical systems," in 2023 IEEE 29th Real-Time and Embedded Technology and Applications Symposium (RTAS), 2023.`
.. [4] `L. Zhang, L. Burbano, X. Chen, et al., "Fast Attack Recovery for Stochastic Cyber-Physical Systems," to appear in 2023 IEEE 30th Real-Time and Embedded Technology and Applications Symposium (RTAS), 2024.`