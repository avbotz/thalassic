#ifndef SUB_CONTROL_PID_CONTROLLER_HPP_
#define SUB_CONTROL_PID_CONTROLLER_HPP_

#include <span>

class PID_Controller {
   public:
    PID_Controller() = default;
    PID_Controller(double kp, double ki, double kd);
    PID_Controller(double kp, double ki, double kd, double output_limit);
    // smooth_saturation: ease the output up to output_limit with tanh instead of
    // a hard clamp. Used by the outer (pos/att) loops so the inner loops chase a
    // continuous setpoint rather than a clipped corner.
    PID_Controller(std::span<const double> gains, bool smooth_saturation = false);

    void configure(double kp, double ki, double kd);
    void configure(double kp, double ki, double kd, double output_limit);
    void configure(std::span<const double> gains);

    double update(double measurement, double error, double dt);

    void reset();

   private:
    static constexpr double ALPHA = 0.4;

    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double output_limit_{0.0};
    bool smooth_saturation_{false};

    double integral_{0.0};
    double prev_measurement_{0.0};
    double smoothed_derivative_{0.0};
};

#endif  // SUB_CONTROL_PID_CONTROLLER_HPP_
