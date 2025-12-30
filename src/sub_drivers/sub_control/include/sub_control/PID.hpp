#ifndef SUB_CONTROL_PID_HPP_
#define SUB_CONTROL_PID_HPP_

class PID {
   public:
    PID();
    PID(double kp, double ki, double kd, double tau_d,
        double out_min, double out_max);

    double update(double error, double measurement, double dt);
    void reset();

    void set_gains(double kp, double ki, double kd);
    void set_limits(double out_min, double out_max);

   private:
    double kp_, ki_, kd_;
    double tau_d_;
    double out_min_, out_max_;

    double integral_;
    double deriv_filt_; 
    double prev_meas_;
    bool have_prev_;
};

#endif  // SUB_CONTROL_PID_HPP_
