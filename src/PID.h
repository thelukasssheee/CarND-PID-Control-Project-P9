#ifndef PID_H
#define PID_H


#include <math.h>
#include <vector>
#include <ctime>

class PID {
  public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;
    double tot_error = 0.;
    double tot_error_last = 0.;
    double tot_error_best = 0.;

    /*
    * Init-values
    */
    double Kp_init;
    double Ki_init;
    double Kd_init;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /*
    * Twiggle optimization parameters
    */
    std::vector<double> dK;
    std::vector<double> K;
    unsigned int iteration;
    unsigned int twiddle_step;
    unsigned int K_it;
    double tolerance;

    /*
    * Initialization flag
    */
    bool is_initialized = false;
    bool optimizer_on = false;

    /*
    * Timing
    */
    double t0;
    double t1;
    std::clock_t start;



    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init();
    // void Init(double Kp, double Ki, double Kd);

    /*
    * Calculate steer angle based on PID.
    */
    double CalcSteerAngle(double cte,double speed, double angle);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    void TotalError(double cte);

    /*
    * Twiddle optimizer to tune parameters
    */
    void Twiddle();
    void Twiddle_Logic();


};

#endif /* PID_H */
