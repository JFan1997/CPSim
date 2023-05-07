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

.. literalinclude:: ../../src/examples/1_CSTR_simulation.py
    :language: python
    :linenos:


Basic Concepts
--------------
Data Structures
~~~~~~~~~~~~~~~


Vector in Numpy
~~~~~~~~~~~~~~~