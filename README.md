This repository contains the solution for the Model Predictive Control Project of the second term of Udacity's Self-Driving Car Engineer Nanodegree Program. This README explains the project and the solution.


# Vehicle Kinematic Model

The implemented controller uses a simple kinematic model without the inclusion of dynamic forces. The vehicle state is given by 4 variables: x and y position, orientation and velocity (as seen in the figure below).

![Vehicle state variables](report/img/vehicle_state_model.png)
Vehicle state variables.

These variables are actuated upon by the control inputs, which in our model are the throttle and the steering.

![](https://latex.codecogs.com/svg.latex?\delta)








[this video](report/particle_filter_project.mp4)
