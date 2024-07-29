<h1>Cao_OptimisticControl</h1>

This code supplements the Automatica submission "An Optimistic Approach to Cost-Aware Predictive Control" by Michael E. Cao, Matthieu Bloch, and Samuel Coogan.

<h2>Abstract</h2>
We consider continuous-time systems subject to a priori unknown state-dependent disturbance inputs. Given a target goal region, our first approach consists of a control scheme that avoids unsafe regions of the state space and observes the disturbance behavior until the goal is reachable with high probability. We leverage collected observations and the mixed monotonicity property of dynamical systems to efficiently obtain high-probability overapproximations of the system's reachable sets. These overapproximations improve as more observations are collected.
For our second approach, we consider the problem of minimizing a desired cost while navigating towards the goal region and modify our previous formulation to allow for the estimated confidence bounds on the disturbance to be adjusted based on what would result in a lower overall cost. We explicitly consider the additional cost incurred through exploration and develop a formulation wherein the amount of exploration performed can be directly tuned.
We show theoretical results confirming this strategy outperforms the previously developed strategy on a simplified system.
We demonstrate the first approach on an example of a motorboat navigating a river, then showcase a Monte Carlo simulation comparison of both approaches on a planar multirotor navigating towards a goal region through an unknown wind field.

<h2>Notes on Repository</h2>
The scripts contained within this repository can be used to reproduce the results from the paper.

For reference:

* The Boat on a River case study can be produced by running `boat_mpc_example.m` in the `boat/` directory
* The Optimistic Planar Multirotor case study can be produced by running `sixquad_mpc_example.m`, `sixquad_mpc_example_2.m`, and `sixquad_mpc_example_3.m` in the `multirotor/` directory. Additionally, the original results are provided in `multirotor/results/` and can be collated by running `collect_opt_results.m`
