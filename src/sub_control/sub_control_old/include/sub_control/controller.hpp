#ifndef SUB_CONTROL_CONTROLLER_HPP_
#define SUB_CONTROL_CONTROLLER_HPP_

#include <array>

constexpr int CONTROL_AXES = 3;

struct ControllerConfig {
    std::array<double, CONTROL_AXES> position_gain{0.8, 0.8, 0.8};
    std::array<double, CONTROL_AXES> position_integral_gain{0.08, 0.08, 0.08};
    std::array<double, CONTROL_AXES> attitude_gain{0.8, 0.8, 0.7};
    std::array<double, CONTROL_AXES> velocity_kp{32.0, 32.0, 40.0};
    std::array<double, CONTROL_AXES> velocity_ki{4.0, 4.0, 6.0};
    std::array<double, CONTROL_AXES> angular_rate_kp{3.0, 3.0, 4.0};
    std::array<double, CONTROL_AXES> angular_rate_ki{0.05, 0.05, 0.08};
    std::array<double, CONTROL_AXES> max_velocity{1.0, 1.0, 0.7};
    std::array<double, CONTROL_AXES> max_angular_rate{0.35, 0.35, 0.3};
    std::array<double, CONTROL_AXES> max_linear_acceleration{0.8, 0.8, 0.6};
    std::array<double, CONTROL_AXES> max_angular_acceleration{0.45, 0.45, 0.35};
    std::array<double, CONTROL_AXES> velocity_integral_limit{12.0, 12.0, 16.0};
    std::array<double, CONTROL_AXES> angular_integral_limit{0.5, 0.5, 0.5};
    std::array<double, CONTROL_AXES> position_integral_limit{1.0, 1.0, 1.0};
    std::array<double, CONTROL_AXES> max_force{80.0, 80.0, 100.0};
    std::array<double, CONTROL_AXES> max_torque{8.0, 8.0, 5.0};
};

struct ControlInput {
    std::array<double, CONTROL_AXES> position_error{};
    std::array<double, CONTROL_AXES> attitude_error{};
    std::array<double, CONTROL_AXES> velocity{};
    std::array<double, CONTROL_AXES> angular_rate{};
    std::array<double, CONTROL_AXES> velocity_command{};
    std::array<double, CONTROL_AXES> angular_rate_command{};
    bool position_control{false};
    bool attitude_control{true};
};

struct ControlOutput {
    std::array<double, 6> wrench{};
    std::array<double, CONTROL_AXES> velocity_setpoint{};
    std::array<double, CONTROL_AXES> angular_rate_setpoint{};
    std::array<double, CONTROL_AXES> velocity_error{};
    std::array<double, CONTROL_AXES> angular_rate_error{};
};

class StateFeedbackController {
   public:
    explicit StateFeedbackController(const ControllerConfig& config = ControllerConfig{});

    ControlOutput update(const ControlInput& input, double dt);
    void reset();
    void set_config(const ControllerConfig& config);

   private:
    ControllerConfig config_;
    std::array<double, CONTROL_AXES> velocity_integral_{};
    std::array<double, CONTROL_AXES> angular_integral_{};
    std::array<double, CONTROL_AXES> position_integral_{};
    std::array<double, CONTROL_AXES> velocity_setpoint_{};
    std::array<double, CONTROL_AXES> angular_rate_setpoint_{};

    static double smooth_limit(double value, double limit);
    static double slew(double current, double target, double rate_limit, double dt);
    static double update_pi(double error, double kp, double ki, double integral_limit, double output_limit, double dt,
                            double& integral);
    static double update_guidance(double error, double kp, double ki, double integral_limit, double output_limit,
                                  double dt, double& integral);
};

#endif  // SUB_CONTROL_CONTROLLER_HPP_
