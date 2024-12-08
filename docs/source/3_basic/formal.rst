Safety Analysis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When applying a sequence of control inputs :math:`\textbf{u}_0,\dots, \textbf{u}_{T-1} \in \mathcal{U}` to a physical system,
the system state evolves according to its dynamics :math:`\psi`, i.e., :math:`\textbf{x}_{t+1} = \psi (\textbf{x}_t,\textbf{u}_t)`.
The sequence of evolving states is called \emph{State Trajectory} :math:`\xi`, where :math:`\xi_i` denotes the :math:`i`-th state in the trajectory.
By applying all possible control sequences within :math:`T` control steps, we can have all possible state trajectories :math:`\Xi(\textbf{x}_0,T)` as
:math:`\Xi(\textbf{x}_0,T) = \{\xi:\xi_0=\textbf{x}_0, \xi_{t+1}=\psi (\xi_t,\textbf{u}_t)\}`,
where :math:`\textbf{u}_t \in \mathcal{U}`, :math:`t \in \{0,\dots, T-1\}`, and :math:`\textbf{x}_0` is an initial state.
Then, the reachable set :math:`\mathcal{R}` includes all possible system states in :math:`\Xi(\textbf{x}_0,T)`.

The **Unsafe State Set** :math:`\mathcal{F}` is a region within the state space, in which the physical system is unsafe and may cause serious consequences.
For example, the distance from a vehicle to an obstacle is less than zero where the unsafe state includes all negative distance values.
The complementary set of :math:`\mathcal{F}` is the safe state set :math:`\mathcal{S}`.
To keep the system safe, the reachable set of a system from a certain initial state :math:`\textbf{x}_0` is required not to intersect with the unsafe state set, i.e., :math:`\mathcal{R}\cap \mathcal{F}= \emptyset`.
Unfortunately, it is very expensive to compute the exact reachable set. 
Instead, we usually compute an over-approximation of the reachable set, denoted by :math:`\bar{\mathcal{R}}` and :math:`\bar{\mathcal{R}} \supseteq \mathcal{R}`.
If :math:`\bar{\mathcal{R}} \cap \mathcal{F}=\emptyset`, then we can guarantee that :math:`\mathcal{R} \cap \mathcal{F}= \emptyset`, i.e., the system is guaranteed to be safe.


Reachability Analysis for Linear Systems
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
By utilizing the Taylor model-based reachability computation on a
past state, we can obtain an overapproximate
estimation of the state at a subsequent time. We begin by taking the
most recent reliable state, :math:`x_w`, as the starting point.
Next, we analyze the historical control sequence utilized from the time :math:`t_w` to :math:`t_f` and
calculate an interval set (or box), :math:`X`. This set is guaranteed to contain the system state at :math:`t_f`.


State Estimate Reconstruction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
At time :math:`t_f` , the system state estimates
cannot reflect the actual system states because of sensor
attacks.  Therefore, the predictor must rebuild the current state
reachable set :math:`X_f` from a reliable :math:`x_w` state, as provided
by the checkpointer.

.. image:: images/3_basic/start_set_estimation.png
   :width: 500 px
   :align: center
   :alt: State Estimate Reconstruction


Deadline Estimation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
We conduct reachability analysis to determine a deadline, :math:`t_d`, at which point the system may touch the unsafe set.

.. image:: images/3_basic/deadline_estimation.png
   :width: 500 px
   :align: center
   :alt: Deadline Estimation

