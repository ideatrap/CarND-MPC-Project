This project tries to predict and evaluate vehicle model, and control vehicle movement by changing vehicle steering and throttle.


## The Model

The vehicle model used in MPC read in vehicle information from simulator. The information include ptsx(x positions of waypoints), ptsy (y positions of waypoints), px (current x position), py (current y position), psi (current orientation angle), v (current velocity), delta (steering angle), a (current throttle applied). 

The program firstly transforms the positions from global coordinate to vehicle's coordinate. Then it fits a three degree polynomial to the waypoints using function polyfit().

In MPC model, the error factors include cross track error, orientation error, actuation change value error, actuation error, and the error of deviating from the target speed. The weighted sum of the cost are recorded in fg. The priority is to keep car running on the track, hence epsi has the biggest weight. Actuation change value error is to minimize the abrupt change on actuator value, and actuation error is to minimize the usage of actuator. These two error is to try to make the drive comfortable. 

The constraints are set based on the physical property of the track and vehicle. 

The cost function and constraint are updated with the latest state passed in the model. Then, the cost function and the constraints are fed into ipopt solver. The ipopt solver minimizes the cost and yield the optimal actuator value. 


## Latency

The model assumes 100ms latency in applying actuator. To account for the latency, the model calculates the vehicle model in 100ms assuming it maintains the current throttle, and steering. 


## Timestep Length and Elapsed Duration (N & dt)

When I tune N & dt, I discover that if N is too big, the model is slow to respond, and when N is too small, the model tends to be short sighted, hence wave around. 
When dt is too big, the car is too rigid, and it moves too slowly, and keeps pressing break. When dt, the car is short sighted, and it waves left and right. Through trial and error, I found good result by setting N = 9, and dt = 0.135.
