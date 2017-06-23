# Model Predictive Controller
Project in Self-Driving Car Engineer Nanodegree Program

## Introduction
When we want to program our cars to drive in a certain track, we cannot directly let it change its coordinates like in video games, we need to adjust the steer and throttle together in order to make our car drive. The controller algorithm is designed for this which adjust the actuators to let the vehicle performances a good drive on the planned track.

This controller I implemented is called model predictive controller, which is a machine-learning-liked algorithm: minimize the cost function by updating the parameters. It would predict the vehicle's future states in current set of actuators, calculate the cost function according to the future states and planed track, then calculate the gradients of the actuator values which minimize the cost function, then update the actuators according to gradients.

## Model
The model I used to predict the vehicle's state is the kinetic model. There are 2 popular models for MPCs in self driving car: kinetic model and dynamic model. Dynamic model is more complex and computation expensive but more accurate and more robust to the physically complex situations. In this project, as the car is driven in a simulator which does not consider the complex newton forces so kinetic model is used.

In this model, 6 states and 2 actuators are included.

### States
- x: horizontal coordinate of the car in vehicle's coordinate system
- y: vertical coordinate of the car in vehicle's coordinate system
- psi: orientation of the car
- v: speed of the car
- cte: the cross track error about the track
- epsi: the orientation error about the track

### Actuators
- delta: the steering angle in radian
- a: the throttle value

The update function see below:
```
x_t+1 = x_t + v_t * cos(psi_t) * dt
y_t+1 = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t / Lf * delta * dt
v_t+1 = v_t + a_t * dt
cte_t+1 = cte_t + v_t * sin(psi_t) * dt
epsi_t+1 = epsi_t + v_t * delta * dt / Lf

```


## Implementation details
### N and dt: how much future states would be predicted and how frequent
The MPC would predict "N" future states seperated in "dt" amount of time, so choose appropriate values of N and dt is required. This time I set N to 20 and dt to 0.1s so future 2 seconds of the vehicle's state would be predicted.

0.1s is the latency of the simulater, so dt=0.1s is chosen. 3 values of N are tried, 15 20 and 25. No obvious differences observed between these values, then 20 is chosen.

### Delay of the control
In real self driving cases, there is a time delay between when the state inputed to the controller and when the actuators receive the output the the controller and make move. Therefore, the inputed state should be the predicted state at the time actuators make move instead of the current state.


## Cost function and parameter tuning
The object of MPC is to minimize the cost function, so an appropriate cost function is the most important part of this project.

Here is my cost function chosen in this project:
```
Cost = w1 * cte ^ 2 + w2 * epsi ^ 2 + w3 * (v - ref_v) ^ 2 + w4 * delta ^ 2 + w5 * a ^ 2 + w6 * ddelta ^ 2 + w7 * da ^ 2

```
Which "delta" and "a" are the steer and throttle used in order to make a smoother drive and "ddelta" and "da" are the difference of the steer and throttle between current state and previous state in order to prevent sudden accelerations and osillations.

To tune the parameters w1 to w7, first I tried "1 1 1 0 0 0 0" which only first 3 terms of the cost function are considered. In this case, the vehicle osillated violently and then fell off the track.

Constraints on steering should be used, then I tried "1 1 1 1 0 1 0" but the car still oscillated and went off the track.

Larger constraints should be used, then I tried "1 1 1 200 0 1 0" and this time it drives smoothly without big oscillations. However, the speed went up really fast ,then over the reference speed sat which is 40, then up to 80 which led the car rush out of the track.

Constraints on throttle should be used, then I tried "1 1 1 200 100 1 1" and this time it successfully drove arount the track.




---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from [here](https://github.com/coin-or/Ipopt/releases).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
