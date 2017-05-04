#include <stdlib.h>
#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, unsigned long integral_len, unsigned long twiddle_len)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  // initial changes to try in Twiddle
  d_Kp_ = 0.1 * Kp;
  d_Ki_ = 0.1 * Ki;
  d_Kd_ = 0.1 * Kd;
  idx_current_param_ = 0;
  up_down_unch_ = 0;
  best_error_ = 1e+10;
  // keep history of error
  integral_len_ = integral_len;
  twiddle_len_ = twiddle_len;
  total_steps_ = 0;
  cte_history_ = deque<double>();
  speed_history_ = deque<double>();
  dt_history_ = deque<double>();
}

double PID::PredictSteering(double cte, double speed, double dt){
  double angle = 0.0;
  double factor = 100.0;
  double speed_adj = speed > 0 ? speed : 0.001;
  double cte_adj = cte/(factor * speed_adj * dt);
  double cte_diff = 0.0;
  double cte_int = cte_adj;
  // assuming history objects do not yet contain the current observations
  if (cte_history_.size()>1)
  {
    int cnt = 0;
    for (long i=0; i<std::min(integral_len_, cte_history_.size()); i++)
    {
      double c = cte_history_[i] / (factor * speed_history_[i]*dt_history_[i]);
      if (i==0)
        cte_diff = (cte_adj - c);
      cte_int += c;
      cnt++;
    }
    cte_int /= cnt;
  }
  angle = -Kp_*cte_adj - Ki_*cte_int - Kd_*cte_diff;
  return angle;
}

void PID::UpdateError(double cte, double speed, double dt) {
  cte_history_.push_front(cte);
  double speed_adj = speed > 0 ? speed : 0.001;
  speed_history_.push_front(speed_adj);
  dt_history_.push_front(dt);
  if (cte_history_.size() > std::max(integral_len_, twiddle_len_))
  {
    cte_history_.pop_back();
    speed_history_.pop_back();
    dt_history_.pop_back();
  }
  total_steps_++;
}

double PID::TotalError() {
  double res = 0.0;
  for (double d : cte_history_)
    res += d*d;
  return res;
}

void PID::TwiddleIfEnoughHistory()
{
  std::ldiv_t d = std::ldiv(total_steps_, twiddle_len_);

  if (d.rem == 0)
  {
    if (d.quot==1)
      best_error_ = TotalError();
    std::cout << "Twiddle step " << d.quot << ". Best error: " << best_error_ << std::endl;
    switch (idx_current_param_)
    {
      case 0:// Kp tweaking
        std::cout << "Twiddle tweaking: Kp" << std::endl;

        if (up_down_unch_ == 0) {
          std::cout << "bump: up" << std::endl;
          up_down_unch_ = 1;
          Kp_ += d_Kp_;
        } else if (up_down_unch_ == 1) {
          double error = TotalError();
          std::cout << "error: " << error << std::endl;
          if (error < best_error_)
          {
            best_error_ = error;
            std::cout << "bump size: increase" << std::endl;
            d_Kp_ *= 1.1;
            std::cout << "bump: up" << std::endl;
            Kp_ += d_Kp_;
          } else {
            std::cout << "bump: down" << std::endl;
            up_down_unch_ = -1;
            Kp_ -= 2*d_Kp_;
          }
        } else {
          // up_down_unch_ == -1
          double error = TotalError();
          std::cout << "error: " << error << std::endl;
          if (error < best_error_)
          {
            best_error_ = error;
            std::cout << "bump size: increase" << std::endl;
            d_Kp_ *= 1.1;
            std::cout << "bump: down" << std::endl;
            Kp_ -= d_Kp_;
          } else {
            std::cout << "bump: up (to unch)" << std::endl;
            Kp_ += d_Kp_;
            std::cout << "bump size: decrease" << std::endl;
            d_Kp_ *= 0.9;
            up_down_unch_ = 0;

            std::cout << "move to next param" << std::endl;
            idx_current_param_ = 1;
            TwiddleIfEnoughHistory(); // just handle the next parameter
          }
        }
        break;

      case 1:// Ki tweaking
        std::cout << "Twiddle tweaking: Ki" << std::endl;

        if (up_down_unch_ == 0) {
          std::cout << "bump: up" << std::endl;
          up_down_unch_ = 1;
          Ki_ += d_Ki_;
        } else if (up_down_unch_ == 1) {
          double error = TotalError();
          std::cout << "error: " << error << std::endl;
          if (error < best_error_)
          {
            best_error_ = error;
            std::cout << "bump size: increase" << std::endl;
            d_Ki_ *= 1.1;
            std::cout << "bump: up" << std::endl;
            Ki_ += d_Ki_;
          } else {
            std::cout << "bump: down" << std::endl;
            up_down_unch_ = -1;
            Ki_ -= 2*d_Ki_;
          }
        } else {
          // up_down_unch_ == -1
          double error = TotalError();
          std::cout << "error: " << error << std::endl;
          if (error < best_error_)
          {
            best_error_ = error;
            std::cout << "bump size: increase" << std::endl;
            d_Ki_ *= 1.1;
            std::cout << "bump: down" << std::endl;
            Ki_ -= d_Ki_;
          } else {
            std::cout << "bump: up (to unch)" << std::endl;
            Ki_ += d_Ki_;
            std::cout << "bump size: decrease" << std::endl;
            d_Ki_ *= 0.9;
            up_down_unch_ = 0;

            std::cout << "move to next param" << std::endl;
            idx_current_param_ = 2;
            TwiddleIfEnoughHistory(); // just handle the next parameter
          }
        }
        break;

      case 2:// Kd tweaking
        std::cout << "Twiddle tweaking: Kd" << std::endl;

        if (up_down_unch_ == 0) {
          std::cout << "bump: up" << std::endl;
          up_down_unch_ = 1;
          Kd_ += d_Kd_;
        } else if (up_down_unch_ == 1) {
          double error = TotalError();
          std::cout << "error: " << error << std::endl;
          if (error < best_error_)
          {
            best_error_ = error;
            std::cout << "bump size: increase" << std::endl;
            d_Kd_ *= 1.1;
            std::cout << "bump: up" << std::endl;
            Kd_ += d_Kd_;
          } else {
            std::cout << "bump: down" << std::endl;
            up_down_unch_ = -1;
            Kd_ -= 2*d_Kd_;
          }
        } else {
          // up_down_unch_ == -1
          double error = TotalError();
          std::cout << "error: " << error << std::endl;
          if (error < best_error_)
          {
            best_error_ = error;
            std::cout << "bump size: increase" << std::endl;
            d_Kd_ *= 1.1;
            std::cout << "bump: down" << std::endl;
            Kd_ -= d_Kd_;
          } else {
            std::cout << "bump: up (to unch)" << std::endl;
            Kd_ += d_Kd_;
            std::cout << "bump size: decrease" << std::endl;
            d_Kd_ *= 0.9;
            up_down_unch_ = 0;

            std::cout << "move to next param" << std::endl;
            idx_current_param_ = 0;
            TwiddleIfEnoughHistory(); // just handle the next parameter
          }
        }
        break;
    }
    std::cout << "Twiddle end params: Kp = " << Kp_ << " Ki = " << Ki_ << " Kd = " << Kd_ << std::endl << std::endl;

  }
}

