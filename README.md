# Model Predictive Control
---
This project is an implementation of a non linear Model predictive controller for driving a self driving car around track in simulator. The simulator feed the system with stream data that contain car position, velocity and orientation and also the way-points of the trajectory reference that the car should to follow. The control system is designed based on car Kinematic model that ignore that ignore tire forces, gravity, and mass.

## Car state
```
state<<x,y,psi,v,cte,epsi;
```

x: position x <br />
y:position y <br />
psi: car orientation <br />
v: velocity <br />
cte: cross track error <br />
epsi: orientation error <br />
## control input

```
solution.x[delta_start],   solution.x[a_start]
```

delta_start: steering angle
a_start: throttle/break

## Transform from map coordinates to car coordinate (Global Kinematic model)

The simulator provides the car and the trajectory reference coordinates in global coordinate system and the transformation form global to local as below.

```

for (int p=0; p<ptsx.size(); p++){

        	  double shift_x= ptsx[p]-px;
        	  double shift_y= ptsy[p]-py;
        	  ptsx_vehicle.push_back(shift_x*cos(-psi)-shift_y*sin(-psi));
        	 ptsy_vehicle.push_back(shift_x*sin(-psi)+shift_y*cos(-psi)) ;

          }
          
```
## Vehicle model
 kinematic bicycle model is used for this project as it handle the non linearity of the heading change over time and the model used the following equations.
```
fg[1+x_start+t]= x1-(x0+v0*CppAD::cos(psi0)*dt);
		  fg[1+y_start+t]= y1-(y0+v0*CppAD::sin(psi0)*dt);
		  fg[1+psi_start+t]= psi1-(psi0-v0*delta0*dt/Lf);
		  fg[1+v_start+t]=v1-(v0+a0*dt);


		  fg[1 + cte_start + t] =cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
          ```

## Following trajectory
The reference trajectory is typically passed to the control block as a polynomial. This polynomial is usually 3rd order, since third order polynomials will fit trajectories for most roads. To practice this most common situation, we will learn how to fit 3rd order polynomials to waypoints (x, y). I Used polyfit to fit a 3rd order polynomial to the given x and y coordinates representing waypoints.

```
          double* ptr_x=&ptsx_vehicle[0];
          double* ptr_y= &ptsy_vehicle[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptr_x, ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptr_y, ptsy.size());
          auto coeffs= polyfit(ptsx_transform,ptsy_transform,3);
          double cte = polyeval(coeffs,0);
          double epsi = -atan(coeffs[1]) ;
```

## Constraints. 

here I defined the lower and upper limits constrains. the steering angle should minimize the `cte` and `psi` to zero. Also, I set the upper and lower steering values to -25 to 25 radiance. the actuator constrains defined by assuming throttle and break pedals one input control from -1 to 1 so that -1 is full break and 1 is to speed.

```
// Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  for( i=0;i<delta_start;i++){
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
  for ( i = delta_start; i < a_start; i++) {
      vars_lowerbound[i] = -0.436332;
      vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for ( i = a_start; i < n_vars; i++) {
      vars_lowerbound[i] = -1.0;
      vars_upperbound[i] = 1.0;
    }
    ```
by knowing the current state of the car and reference trajectory we optimize the input control at each step in order to minimize the cost of the predicted trajectory. In order to minimise the associated cost of maintaining  the trajectory with average speed I tuned the cost functions as below to ban the vehicle to oscillate. 

```
  fg[0]=0;
	  for (int t; t<N; t++){
		  fg[0]+=3000*CppAD::pow(vars[cte_start+t] -ref_cte,2);
		  fg[0]+=3000*CppAD::pow(vars[epsi_start+t] -ref_epsi,2);
		  fg[0]+= CppAD::pow(vars[v_start+t]-ref_v,2);
	  }


	  for (int t; t<N-1; t++){
		  fg[0]+=3000*CppAD::pow(vars[delta_start+t],2);
		  fg[0]+=300*CppAD::pow(vars[a_start+t],2);

	  }


	  for (int t; t<N-2; t++){
	      	fg[0]+=3000*CppAD::pow(vars[delta_start+t+1]-vars[delta_start+t],2);
	      	fg[0]+=300*CppAD::pow(vars[a_start+t+1]-vars[a_start+t],2);

	  }
      ```
## MPC Tuning
prediction horizon `N=10,dt=.15` are tuned so that make the vehicle keep the trajectory in reasonable future prediction duration `T=1.5 sec`. by decreasing the `dt` the car make high osculation either in low or high speed. the used values make the car drive conservatively in average speed. 

## Dealing with Latency
Latency of 0.1 between MPC loop and the actual actuation is considered by making the car drive more wisely by increased the penalty of the velocity and steering angle


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.




 
    


