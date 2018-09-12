# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

---


# Writeup for this project by Michael Berner
##### Stuttgart, September 12th 2019

The task of this project was to program and optimize a PID controller, which is controlling the steering angle of simulated vehicle.

The project repository on GitHub is located [here](https://github.com/udacity/CarND-Controls-PID). The [project rubric](https://review.udacity.com/#!/rubrics/824/view) describes the requirements to pass the project.

### Effect of PID terms
Each letter of the PID represents one part of the controller.

"P" stands for proportional: the controller is acting proportionally to the deviation. This is the first and most simple type of controller. However, it is susceptible to systematic biases in a final system, such as friction or in case of a car wind forces.

Systematic biases can be coped with by using a term which is integrating the observed error. This is hence called the "I" term.

The dynamic behavior of the controller (overshooting, undershooting) is often related to the change of the observed error. A differential "D" term can be used for this, which is acting based on the change in error.

### Choosing initial hyperparameters for PID controller
I implemented the PID controller using the following equation:

<a href="https://www.codecogs.com/eqnedit.php?latex=\dpi{120}&space;SteerAngle&space;=&space;-K_p&space;\cdot&space;p_{err}&space;-&space;K_i&space;\cdot&space;i_{err}&space;-&space;K_d&space;\cdot&space;d_{err}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dpi{120}&space;SteerAngle&space;=&space;-K_p&space;\cdot&space;p_{err}&space;-&space;K_i&space;\cdot&space;i_{err}&space;-&space;K_d&space;\cdot&space;d_{err}" title="SteerAngle = -K_p \cdot p_{err} - K_i \cdot i_{err} - K_d \cdot d_{err}" /></a>

with

<a href="https://www.codecogs.com/eqnedit.php?latex=\dpi{120}&space;\newline&space;p_{err}&space;=&space;CTE&space;\newline&space;d_{err}&space;=&space;CTE&space;-&space;CTE_{last}&space;\newline&space;i_{err}&space;=&space;i_{err,last}&space;&plus;&space;CTE&space;\cdot&space;\Delta&space;t" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dpi{120}&space;\newline&space;p_{err}&space;=&space;CTE&space;\newline&space;d_{err}&space;=&space;CTE&space;-&space;CTE_{last}&space;\newline&space;i_{err}&space;=&space;i_{err,last}&space;&plus;&space;CTE&space;\cdot&space;\Delta&space;t" title="\newline p_{err} = CTE \newline d_{err} = CTE - CTE_{last} \newline i_{err} = i_{err,last} + CTE \cdot \Delta t" /></a>

CTE stands for Cross-Track-Error, meaning the deviation versus the center of the lane.

ALl three K parameters were then defined by starting with <img src="https://latex.codecogs.com/gif.latex?\inline&space;\dpi{120}&space;K_p" title="K_p" />, until a half-way working solution was found. However, dynamic behavior was not acceptable yet. This issue was tackled by improving <img src="https://latex.codecogs.com/gif.latex?\inline&space;\dpi{120}&space;K_d" title="K_d" /> in the next step. The last parameter was set to a rather small value, since for this simulated project, a systematic bias did not occur (simulation).

The following three parameters were identified to already work quite well:

    Kp_init = 0.31;
    Ki_init = 0.00223694;
    Kd_init = 5.1;

### Parameter optimization with Twiddle
It was obvious, that further optimization was possible. The Udacity online courses introduced an optimization algorithm called [https://www.youtube.com/watch?v=2uQ2BSzDvXs](Twiddle).

I implemented this algorithm as subfunctions `Twiddle()` and `Twiddle_Logic()` in the PID controller C++ file `\src\PID.cpp`.

Every time, the vehicle leaves the road (`abs(CTE) > 5`) or has finished one lap at 30mph (`delta_t > 80`), the simulator is automatically reset and the next set of PID parameters is being evaluated.

The Twiddle algorithm was started with the initial values stated above and `delta_K` start values of `0.25 * init` values. Optimization was stopped, when the abortion criteria `sum(delta_K) < 0.001` was reached.

An error term is necessary to perform an optimization. I calculated it by integrating the absolute value of the cross-track-error CTE over the entire iteration. At the end, it was divided by the total distance driven. To keep the code a little simpler, the distance was approximated by directly using simulation time.

The result from every iteration is shown in the console and written to a text file called `\build\Optimizer_results.txt` and looks like this:

    PID controller successfully initialized with [Kp,Ki,Kd] = [0.31, 0.00223694, 5.1] !
    Iteration 1 with [Kp,Ki,Kd] = [0.31,0.00223694,5.1] finished. Error: 54.3143 (curr) vs 1e+11 (best).
    Iteration 2 with [Kp,Ki,Kd] = [0.31,0.00279618,5.1] finished. Error: 21.1804 (curr) vs 54.3143 (best).
    Iteration 3 with [Kp,Ki,Kd] = [0.31,0.00279618,6.375] finished. Error: 19.5896 (curr) vs 21.1804 (best).
    Iteration 4 with [Kp,Ki,Kd] = [0.39525,0.00279618,6.375] finished. Error: 19.1183 (curr) vs 19.5896 (best).
    Iteration 5 with [Kp,Ki,Kd] = [0.39525,0.00341133,6.375] finished. Error: 18.4623 (curr) vs 19.1183 (best).
    Iteration 6 with [Kp,Ki,Kd] = [0.39525,0.00341133,7.7775] finished. Error: 16.7749 (curr) vs 18.4623 (best).
    Iteration 7 with [Kp,Ki,Kd] = [0.489025,0.00341133,7.7775] finished. Error: 18.4795 (curr) vs 16.7749 (best).
    Iteration 8 with [Kp,Ki,Kd] = [0.301475,0.00341133,7.7775] finished. Error: 16.9628 (curr) vs 16.7749 (best).
    ...

An example of this file is stored in in this repository in `\Optimizer_results (Example).txt`.

[This video](https://youtu.be/y4xi85PcBZs) shows how the simulation resets and the next optimizer iteration is starting.  

### Solution and final PID parameters
Eventually, the optimizer exited providing the following parameters for the PID controller:

    Kp_best = 0.402086;
    Ki_best = 0.00408801;
    Kd_best = 11.7821;

A video of the final solution using these parameters can be watched on [Youtube](https://youtu.be/Xx0yX1ZeNuk).

### Discussion
It was observed, that Twiddle is in fact sometimes being stuck in local minimas. When I started the algorithm several times with slightly different initialization values for K, a different "best" solution was obtained every single time. Potentially, it could be more suitable to define a large Design-of-experiments plan and look more into detail at areas indicating sweet spots.  

Additionally, it became obvious, that constant parameters over the entire speed range and for straights and curves are not an ideal solution.

Furthermore, it would be an option to have controller parameters dependent on drive speed, curve radius and - for real life situations - ambient conditions, road conditions etc. Especially, the wobbling / oscillating driving behavior would be not at all acceptable for passengers.

I'm looking forward to the next project with an MPC controller, which might solve these shortcomings altogether!


### Video material

[Twiddle optimizer](https://youtu.be/y4xi85PcBZs)

[Final: complete lap](https://youtu.be/Xx0yX1ZeNuk)
