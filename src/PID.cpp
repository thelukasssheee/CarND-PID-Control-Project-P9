#include "PID.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  CSV.open ("Optimizer_results.txt");
}

PID::~PID() {
  CSV.close();
}

// void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
void PID::Init() {
  // (Re-)Initialize error variables
  p_error = 0.;
  i_error = 0.;
  d_error = 0.;

  if (optimizer_on) {
    // Check initialization status and print out parameters if necessary
    if (!is_initialized) {
      // Call twiddle
      Twiddle();
      cout << "PID controller successfully initialized with [Kp,Ki,Kd] = [";
      cout << Kp << ", " << Ki << ", " << Kd << "] !" << endl;
      CSV << "PID controller successfully initialized with [Kp,Ki,Kd] = [";
      CSV << Kp << ", " << Ki << ", " << Kd << "] !" << endl;
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
      Kp = Kp_init;
      Ki = Ki_init;
      Kd = Kd_init;
      is_initialized = true;
      cout << "PID controller started with hard-coded parameters ";
      cout << "[Kp,Ki,Kd] = [" <<Kp<<", "<<Ki<<", "<<Kd<< "] !" << endl;
    }
    // Twiddle is finished
    else {
      Kp = K_best[0];
      Ki = K_best[1];
      Kd = K_best[2];
      cout << "PID controller starts over with final parameters from ";
      cout << "PID optimization. ";
      cout << "[Kp,Ki,Kd] = [" <<Kp<<", "<<Ki<<", "<<Kd<< "] !" << endl;

    }
  }

}

double PID::CalcSteerAngle(double cte,double speed, double angle) {
  return -Kp*p_error -Ki*i_error - Kd*d_error;
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
  tot_error += abs(cte);
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
  */

  // Perform initialization step if necessary
  if (!is_initialized) {
    K = {Kp_init, Ki_init, Kd_init};
    dK = {0.25*Kp_init, 0.25*Ki_init, 0.25*Kd_init};
    iteration = 0;
    twiddle_step = 0;
    K_it = 0;
    tolerance = 0.001;
    tot_error_best = 99999999999.;
    Kp = K[0]; Ki = K[1]; Kd = K[2];
    // Exit twiddle function, because everything is prepared for first run
    return;
  }

  // Enter Twiddle optimizer
  if (optimizer_on && (dK[0] + dK[1] + dK[2] > tolerance)) {
    // Normalize total error
    tot_error = tot_error / t_delta;
    cout << "Iteration " << iteration << " with [Kp,Ki,Kd] = [";
    cout << K[0] << "," << K[1] << "," << K[2] << "] finished. ";
    cout << "Error: ";
    cout << tot_error << " (curr) vs " << tot_error_best << " (best)." << endl;
    CSV << "Iteration " << iteration << " with [Kp,Ki,Kd] = [";
    CSV << K[0] << "," << K[1] << "," << K[2] << "] finished. ";
    CSV << "Error: ";
    CSV << tot_error << " (curr) vs " << tot_error_best << " (best)." << endl;
    Twiddle_Logic();
    Kp = K[0]; Ki = K[1]; Kd = K[2];
    iteration += 1;
  }


  // Optimization finished!
  else {
    Kp = K_best[0]; Ki = K_best[1]; Kd = K_best[2];
    cout << "-----------------------------------------------" << endl;
    cout << "Optimizer finished! Total vs. best error = ";
    cout << tot_error << ":" << tot_error_best << "." << endl;
    cout << "Obtained parameters:" << endl;
    cout << " Kp = " << Kp << endl;
    cout << " Ki = " << Kp << endl;
    cout << " Kd = " << Kp << endl;
    CSV << "-----------------------------------------------" << endl;
    CSV << "Optimizer finished! Total vs. best error = ";
    CSV << tot_error << ":" << tot_error_best << "." << endl;
    CSV << "Obtained parameters:" << endl;
    CSV << " Kp = " << Kp << endl;
    CSV << " Ki = " << Kp << endl;
    CSV << " Kd = " << Kp << endl;
    CSV.close();

    // TODO: exit program or run with best parameters
    optimizer_on = false;
    is_initialized = true;
  }
}

void PID::Twiddle_Logic() {
  if (iteration == 0) {
    return;
  }

  // Apply twiddle logic
  if (twiddle_step == 0) {
    if (tot_error < tot_error_best) {
      twiddle_step = 1;
    }
    else {
      twiddle_step = 2;
    }
  }
  else if (twiddle_step == 2) {
    if (tot_error < tot_error_best) {
      twiddle_step = 3;
    }
    else {
      twiddle_step = 4;
    }
  }

  // Perform calculations based on decided logic
  switch (twiddle_step) {
    case 1: {
      cout << "  Case 1, K_it=" << K_it;
      dK[K_it] *= 1.1;
      tot_error_best = tot_error;
      K_best = K;
      break;
    }

    case 2: {
      cout << "  Case 2, K_it=" << K_it;
      K[K_it] -= 2* dK[K_it];
      break;
    }

    case 3: {
      cout << "  Case 3, K_it=" << K_it;
      dK[K_it] *= 1.1;
      tot_error_best = tot_error;
      K_best = K;
      break;
    }

    case 4: {
      cout << "  Case 4, K_it=" << K_it;
      K[K_it] += dK[K_it];
      dK[K_it] *= 0.9;
      break;
    }
  }

  // Prepare for next execution
  if (twiddle_step != 2) {
    K_it = (K_it + 1) % 3;
    K[K_it] += dK[K_it];
    twiddle_step = 0;
  }
  tot_error_last = tot_error;
  tot_error = 0.0;

  // Output new parameters
  cout << ". New params = [" <<K[0]<<","<<K[1]<<","<<K[2]<< "]." << endl;
  Kp = K[0]; Ki = K[1]; Kd = K[2];


  /*
    for i in range(len(p)):
        ###STEP 0###
        p[i] += dp[i]
        robot = make_robot()
        x_trajectory, y_trajectory, err = run(robot, p)

        if err < best_err:
            ###STEP 1###
            best_err = err
            dp[i] *= 1.1
        else:
            ###STEP 2###
            p[i] -= 2 * dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                ###STEP 3####
                best_err = err
                dp[i] *= 1.1
            else:
                ###STEP 4###
                p[i] += dp[i]
                dp[i] *= 0.9
    it += 1
  */

}
