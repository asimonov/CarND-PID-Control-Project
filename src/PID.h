#ifndef PID_H
#define PID_H

#include <vector>
#include <deque>

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
  void Init(double Kp, double Ki, double Kd, unsigned int memory_len = 100);

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


private:
    /*
    * Errors
    */
    double p_error_;
    double i_error_;
    double d_error_;

    /*
    * Coefficients
    */
    double Kp_;
    double Ki_;
    double Kd_;

    deque<double> cte_history_;

    unsigned long memory_len_;
    unsigned long total_steps_;
};

#endif /* PID_H */
