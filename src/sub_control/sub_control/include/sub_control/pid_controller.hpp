#ifndef SUB_CONTROL_PID_CONTROLLER_HPP_
#define SUB_CONTROL_PID_CONTROLLER_HPP_

#include <span>

class PID_Controller {
   public:
    PID_Controller() = default;
    PID_Controller(double kp, double ki, double kd);
    PID_Controller(double kp, double ki, double kd, double output_limit);
    PID_Controller(std::span<const double> gains);

    void configure(double kp, double ki, double kd);
    void configure(double kp, double ki, double kd, double output_limit);
    void configure(std::span<const double> gains);

    // output_limit_override > 0 replaces the configured output limit for this
    // call (clamp and anti-windup), e.g. to cap the yaw rate during a spin.
    double update(double measurement, double error, double dt, double output_limit_override = 0.0);

    double kp() const { return kp_; }
    double output_limit() const { return output_limit_; }

    void reset();

   private:
    static constexpr double ALPHA = 0.4;

    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double output_limit_{0.0};

    double integral_{0.0};
    double prev_measurement_{0.0};
    double smoothed_derivative_{0.0};
    bool primed_{false};
};

#endif  // SUB_CONTROL_PID_CONTROLLER_HPP_
