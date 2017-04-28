#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, unsigned int memory_len)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  memory_len_ = memory_len;
  total_steps_ = 0;
  cte_history_ = deque<double>();
}

double PID::PredictSteering(double cte){
  double angle = 0.0;
  double cte_diff = 0.0;
  double cte_int = cte;
  if (cte_history_.size()>1)
  {
    cte_diff = cte - cte_history_.front();
    for (double d : cte_history_)
      cte_int += d;
  }
  angle = -Kp_*cte - Ki_*cte_int - Kd_*cte_diff;
  return angle;
}

void PID::UpdateError(double cte) {
  cte_history_.push_front(cte);
  if (cte_history_.size() > memory_len_)
    cte_history_.pop_back();
  total_steps_++;
}

double PID::TotalError() {
  double res = 0.0;
  for (double d : cte_history_)
    res += d*d;
  return res;
}

