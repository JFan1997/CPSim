Get Started
=================

Installation
------------

We recommend installing CPSim in a virtual environment.
Please refer to `Anaconda Documentation <https://docs.anaconda.com/free/anaconda/install/index.html>`_ for more information.
After activating your virtual environment, install CPSim using the following command:

.. code:: bash

   pip install cpsim


(Optional) If you want to use interval arithmetic, install CPSim with the following command:

.. code:: bash

    pip install cpsim[interval]


This will install pyinterval dependency, but only works on Linux.


Minimum Simulation
------------------
After installing CPSim, you can write the following code to simulate the behaviours of a continuous stirred tank reactor.

.. literalinclude:: ../../src/examples/1_CSTR_simulation.py
    :language: python
    :linenos:

First, we import a built-in nonlinear CSTR model at Line 3.
Then, we configure some simulation parameters, such as sampling time :math:`dt`, at Lines 6-9.
At Line 10, we create a simulation object with the model and the parameters.

The main control loop is at Lines 12-16.
In each iteration of the loop, it updates the reference state, and evolves the system a sampling time advance.

After simulation, the reference state, output, system state, and control input are stored in the simulation object.
Here, we plot the reference state and the output at Lines 26-28.

Basic Concepts
--------------
Data Structures
~~~~~~~~~~~~~~~


Vector in Numpy
~~~~~~~~~~~~~~~