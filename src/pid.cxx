#include <PID/pid.hxx>

PIDController::PIDController()
{

}

void PIDController::set_target(double target){ this.target = target; }
double PIDController::error()
{
  double current_error = this.target - curent_temp;
  return current_error;
}
double PIDController::calc_error()
{
  double int_val = this->error() * this->sample_time; // calculate integral value
  this->sum_integral += int_val;
  double dir_val = (this->error() - this->prev_error) / this->sample_time; // calculate derivative
  this->prev_error = this->error(); // set previous error for next run

  // calculate the total Percent ON 
  double error_out = (this->kp * this->error()) + (this->ki * this->sum_integral) + (this->kd * dir_val);




  return error_out; // this is the percent power of temperature
}


