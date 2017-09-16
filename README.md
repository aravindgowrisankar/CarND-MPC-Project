# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## SETUP

Please refer to the [SETUP](./SETUP.md) file for requirements & installation instructions.

## Implementation

The file [MPC.cpp](./src/MPC.cpp) contains a very simple implementation of the MPC controller.

The file [main.cpp](./src/main.cpp) contains the driver code.


## Reflection

### Model
The equations are based  on the Vehicle Model lesson described in this course.
The equations are converted to constraints that can be provided to IPOpt.

#### Cost Function
The function [operator()](https://github.com/aravindgowrisankar/CarND-MPC-Project/blob/master/src/MPC.cpp#L48) generates the costs to be used by the optimization function.

The key steps in computing the costs:
* Penalize the y co-ord(CTE), steering and velocity using Mean squared error
* Penalize the actuators 
* Penalize the change in velocity and the actuators 

#### State Update Equations

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + cte_start + t] = cte1 - (y_ref-y0 + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + psi_start + t] = psi1 - (psi0 + (v0 * d0*dt/Lf));
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0-psi_des + (v0 * d0*dt/Lf));

#### Actuator Equations
The two actuators are throttle and steering wheel. 
The throttle and steering wheel have a range [-1,1]. 

We place an additional constraint on steering wheel to ensure that the simulation is real i.e the car doesn't take abrupt 90 degree turns.
The Actuator constraints are provided to the optimizer along with a cost function and the optimizer comes up with the next set of actuations.

### Values for N and dt
I selected the same values used in the Quiz. 

N=25

dt=0.05

### Waypoint Preprocessing
There are 2 key insights required to convert the waypoints from Global co-ordinates to car's coordinate system. 

* Rotate/transform the map co-ordinate axis by the vehicle's orientation to the Car
* Rotate/transform the map co-ordinate axis by the vehicle's orientation to the Waypoints
* Center the waypoints around the Car's co-ordinate by subtracting the Car's x,y co-ordiantes

This is done by the equations in [main.cpp](https://github.com/aravindgowrisankar/CarND-MPC-Project/blob/master/src/main.cpp#L110)


            x_rel[i]=(ptsx[i]*cos(psi))+(ptsy[i]*sin(psi))-car_x;

            y_rel[i]=(ptsy[i]*cos(psi))-(ptsx[i]*sin(psi))-car_y;

### Polynomial Fitting
Polynomial fitting is done using the code provided in the lesson.
This is done using function [polyfit](https://github.com/aravindgowrisankar/CarND-MPC-Project/blob/master/src/main.cpp#L47)

### Dealing with Latency
In this exercise, I did not do anything to do with latency.

However, one option is to simulate the car's motion during the latency window. i.e assume the same reference curve/waypoints but apply the motion model to determine the new px/py/v/phi and re-run the optimization. 






