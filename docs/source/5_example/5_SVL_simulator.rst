High-fidelity Simulators
~~~~~~~~~~~~~~~~~~~~~~~~

Since CPSim is a python package, it can be used in conjunction with any simulator that can be called from python.
Especially, many high-fidelity simulators are controlled through python node of ROS, such as Gazebo, SVL, CARLA, and AirSim.

This example demonstrates how to use the toolbox to recover an autonomous vehicle in the SVL simulator.
The vehicle suffers from an IMU sensor attack, deviates from its own lane, and even enters the oncoming lane, as shown in Figure (a).
To apply the lqr recovery controller [2]_ after detecting the attack, we need to integrate the toolbox with the SVL simulator.
Since there is a ROS bridge communicating with the simulator, we load the toolbox in a ROS node, which is responsible for recovering the vehicle from the attack within a safety deadline.
Figure (b) shows that vehicle returns to its lane during the recovery process.
Figure (c) shows that the recovery controller steers the vehicle to a safe region, the road shoulder, to avoid a collision after recovery.

.. image:: images/5_example/svl_result.png
   :width: 650 px
   :align: center
   :alt: SVL simulation results

Finally, we show the demonstration video here.

.. raw:: html

    <p align="center"><iframe width="700" height="350" src="https://www.youtube.com/embed/OBKl84xG1r4?rel=0" title="Demonstration on SVL Simulator" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe></p>
