#include "PID.h"
#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

// void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
void PID::Init() {
  // (Re-)Initialize error variables
  p_error = 0.;
  i_error = 0.;
  d_error = 0.;
  tot_error = 0.;

  if (optimizer_on) {
    // Check initialization status and print out parameters if necessary
    if (!is_initialized) {
      // Call twiddle
      Twiddle();
      cout << "PID controller successfully initialized with [Kp,Ki,Kd] = [";
      cout << Kp << ", " << Ki << ", " << Kd << "] !" << endl;
      // Set initialized flag to true
      is_initialized = true;
      return;
    }
    else {
      // Call twiddle to obtain parameters for next iteration
      Twiddle();
    }
  }

  // Twiddle optimization not desired or finished
  else {
    // Not desired: Hard-code parameters
    if (!is_initialized) {
      Kp = 0.3;
      Ki = 0.001;
      Kd = 0.2;
      is_initialized = true;
      cout << "PID controller started with hard-coded parameters ";
      cout << "[Kp,Ki,Kd] = [" <<Kp<<", "<<Ki<<", "<<Kd<< "] !" << endl;
    }
    // Twiddle is finished
    else {
      cout << "PID controller starts over with final parameters from ";
      cout << "PID optimization. ";
    }
  }

}

double PID::CalcSteerAngle(double cte,double speed, double angle) {
  double steer_value = -Kp*p_error -Ki*i_error - Kd*d_error;
  // cout << "Steer value = " << steer_value << " Kp: " << Kp << " CTE: " << cte << endl;
  return steer_value;
}

void PID::UpdateError(double cte) {
  // Differential term:
  // current error minus previous error. Previous error still in p_error
  d_error  = cte - p_error;
  // Integral term: integration of error over time
  double delta_t = 0.1;
  i_error += cte * delta_t;
  if (i_error * Ki > 1.0) {
    i_error = 1.0 / Ki;
  }
  if (i_error * Ki < -1.0) {
    i_error = -1.0 / Ki;
  }
  // Proportional term:
  p_error  = cte;
}

void PID::TotalError(double cte) {
  tot_error += cte;
}

void PID::Twiddle() {
  /*
  # Make this tolerance bigger if you are timing out!
  def twiddle(tol=0.0001):
    # Don't forget to call `make_robot` before every call of `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration ",it,", err ","%.2e" % best_err, ", params: ",
        "%.2f" % p[0],",%.2f" % p[1],",%.2f" % p[2])

          TWIDDLE_LOGIC()

        it += 1
    return p, best_err
  */  // Perform initialization step if necessary
  if (!is_initialized) {
    K = {0.01, 0.01, 0.01};
    dK = {1., 1., 1.};
    iteration = 0;
    twiddle_step = 0;
    tolerance = 0.001;
    Kp = K[0]; Ki = K[1]; Kd = K[2];
    // Exit twiddle function, because everything is prepared for first run
    return;
  }

  // Enter Twiddle optimizer
  if (optimizer_on && (dK[0] + dK[1] + dK[2] > tolerance)) {
    Twiddle_Logic();
    cout << "Iteration " << iteration << ", [Kp,Ki,Kd] = [";
    cout << K[0] << "," << K[1] << "," << K[2] << "]. " << endl;
  }
  // Optimization finished!
  else {
    cout << "-----------------------------------------------" << endl;
    cout << "Optimizer finished! Best error = "<< tot_error << endl;
    cout << "Obtained parameters:" << endl;
    cout << " Kp = " << Kp << endl;
    cout << " Ki = " << Kp << endl;
    cout << " Kd = " << Kp << endl;
    // TODO: exit program or run with best parameters
    optimizer_on = false;
    is_initialized = true;
  }
}

void PID::Twiddle_Logic() {
  /*
    for i in range(len(p)):
        p[i] += dp[i]
        robot = make_robot()
        x_trajectory, y_trajectory, err = run(robot, p)

        if err < best_err:
            best_err = err
            dp[i] *= 1.1
        else:
            p[i] -= 2 * dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] += dp[i]
                dp[i] *= 0.9
    it += 1
  */
  // for (unsigned int j = 0; j < 3; ++j) {
  //
  // }
  std::cout << "Twiddle logic function called" << std::endl;
}
