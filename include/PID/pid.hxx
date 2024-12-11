#ifndef PID_CONTORLLER_HXX
#define PID_CONTORLLER_HXX



class PIDController
{
  private:
    double kp; ///< Proportional Gain
    double ki; ///< Integral Gain
    double kd; ///< Derivative Gain
    double sum_integral; ///< integral acumulator
    double prev_error; ///< Previous Error
  public:
    PIDController(double kp, double ki, double kd); ///< Constructor
    void set_target(double target);
    double error();
    double calc_error();
    double get_temp_val; 
};

#endif
