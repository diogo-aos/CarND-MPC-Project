This repository contains the solution for the Model Predictive Control Project of the second term of Udacity's Self-Driving Car Engineer Nanodegree Program. This README explains the project and the solution.

![](https://latex.codecogs.com/svg.latex?\delta)
[this video](report/img/delta.svg)


# Vehicle Kinematic Model

The implemented controller uses a simple kinematic model without the inclusion of dynamic forces. The vehicle state is given by 4 variables: x and y position, orientation ![](report/img/psi.svg) and velocity (as seen in the figure below).

![Vehicle state variables](report/img/vehicle_state_model.png)
Vehicle state variables (adapted from lecture video).

These variables are actuated upon by the control inputs, which in our model are the throttle _a_ and the steering ![](report/img/delta.svg).

The state transition between consecutive time steps (with _dt_ as the interval between steps) is given by the following equations:

<!-- ![](https://latex.codecogs.com/svg.latex?x_{t+1} = x_t + v_t . cos \left ( \psi_t \right ) . dt
) -->

![](https://latex.codecogs.com/svg.latex?y_{t+1} = y_t + v_t . sin\psi_t . dt)

![](https://latex.codecogs.com/svg.latex?\psi_{t+1} = \psi_t + \frac{v_t}{L_f} . \delta_t . dt)

![](https://latex.codecogs.com/svg.latex?v_{t+1} = v_t + a . dt)


![](https://latex.codecogs.com/svg.latex?cte_{t+1} = cte_t + v_t . sin\psi_t . dt)
