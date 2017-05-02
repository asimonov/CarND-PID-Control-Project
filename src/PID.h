#ifndef PID_H
#define PID_H

#include <vector>
#include <deque>
#include <cstdlib>
#include <iostream>

using namespace std;


class PID {
public:
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
  void Init(double Kp, double Ki, double Kd, unsigned long integral_len=100, unsigned long twiddle_len=5600);

  /*
  * Predict Steering angle.
  */
  double PredictSteering(double cte);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle parameters if accumulated enough history.
  */
  void TwiddleIfEnoughHistory();



private:
    /*
    * Coefficients
    */
    double Kp_;
    double Ki_;
    double Kd_;
    /*
    * Coefficient changes for Twiddle
    */
    double d_Kp_;
    double d_Ki_;
    double d_Kd_;
    int idx_current_param_; // which parameter we are optimizing now
    int up_down_unch_; // is current param bumped up (+1), down (-1) or unchanged (0).
    double best_error_; // best error seen so far in twiddle

    deque<double> cte_history_;
    unsigned long integral_len_;
    unsigned long twiddle_len_;
    unsigned long total_steps_;
};

#endif /* PID_H */
