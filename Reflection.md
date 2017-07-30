This project tries to predict and evaluate vehicle model, and control vehicle movement by changing vehicle steering and throttle.


#The Model


Student describes their model in detail. This includes the state, actuators and update equations.

Timestep Length and Elapsed Duration (N & dt)

Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Polynomial Fitting and MPC Preprocessing

A polynomial is fitted to waypoints.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

fit it

Model Predictive Control with Latency

The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

just calculate the position with 100 millisecond


In Use MPC model to control the vehicle.

Key tuning:

When dt is small, the car is not stable, it shakes left and right
N is too large, not considering the immediate discrepancy
Not to look to far, the accretion has no impact on that far


weight
