#ifndef SUB_CONTROL_REFERENCE_GENERATOR_HPP_
#define SUB_CONTROL_REFERENCE_GENERATOR_HPP_

#include "sub_control/trajectory.hpp"
#include "sub_control/utils.hpp"

#include <array>
#include <cstdint>
#include <string>

// Values match sub_control_interfaces/MotionSetpoint.
enum class AxisMode : std::uint8_t { KEEP = 0, POSITION = 1, VELOCITY = 2, HOLD = 3, EFFORT = 4 };
enum class VelocityFrame : std::uint8_t { HEADING = 0, WORLD = 1 };

// One MotionSetpoint, decoded. Rotations are ZYX Euler angles and their rates.
struct MotionCommand {
    std::array<AxisMode, NUM_DOF> mode{};
    Vector6d position{Vector6d::Zero()};
    Vector6d velocity{Vector6d::Zero()};
    Vector6d effort{Vector6d::Zero()};
    VelocityFrame velocity_frame{VelocityFrame::HEADING};
    bool altitude{false};
    double expires{0.0};  // [s] clock time after which VELOCITY and EFFORT axes HOLD; 0 never
};

struct ReferenceLimits {
    ShaperLimits horizontal{0.5, 0.25, 0.5};
    ShaperLimits vertical{0.3, 0.2, 0.4};
    ShaperLimits roll{0.5, 1.0, 2.0};
    ShaperLimits pitch{0.5, 1.0, 2.0};
    ShaperLimits yaw{0.6, 0.8, 2.0};
    double translation_leash{0.5};  // [m] the reference stays this close to the vehicle
    double rotation_leash{0.5};     // [rad]
};

// The trajectory generator: holds each axis's mode and target and shapes them
// into a smooth reference with ReferenceLimits. Translation is in odom ENU (x/y
// shaped together so moves are straight lines); z is depth or, while holding
// altitude, DVL altitude; rotations are ZYX Euler angles kept continuous (no
// wrap) so a turn always takes the short way and never jumps.
class ReferenceGenerator {
   public:
    // Every axis holds `pose` ([x, y, z, roll, pitch, yaw], z in depth
    // coordinates), except roll and pitch, which level out.
    void reset(const Vector6d& pose);

    // Applies the axes `command` sets. `pose` is the measured pose and
    // `altitude` the measured DVL altitude (NaN if unknown). Returns why the
    // command was rejected, or "" if it was applied.
    std::string apply(const MotionCommand& command, const Vector6d& pose, double altitude);

    // Advances the reference by dt toward the targets. `pose` is the measured
    // pose with z in the coordinates altitude() says.
    void update(double now, double dt, const Vector6d& pose, const ReferenceLimits& limits);

    // Axes without feedback (e.g. x/y while the DVL is out) follow the measured
    // pose instead of a trajectory, keeping their mode and target so they pick up
    // from wherever the vehicle is once feedback returns. EFFORT axes always do.
    void set_passive(int axis, bool passive);
    bool passive(int axis) const { return passive_[axis]; }

    // Gives up holding altitude: z holds its current depth instead.
    void drop_altitude(double depth_z);

    AxisMode mode(int axis) const { return mode_[axis]; }
    bool altitude() const { return altitude_; }
    VelocityFrame velocity_frame() const { return velocity_frame_; }

    // Where each axis is headed: its POSITION target, or else the reference.
    Vector6d target() const;
    // VELOCITY target of an axis in its command frame, or 0.
    double velocity_target(int axis) const { return mode_[axis] == AxisMode::VELOCITY ? velocity_target_[axis] : 0.0; }
    // Open-loop effort of an EFFORT axis (body, [N] or [N m]), or 0.
    double effort(int axis) const { return mode_[axis] == AxisMode::EFFORT ? effort_[axis] : 0.0; }

    Vector6d position() const;
    Vector6d velocity() const;
    Vector6d acceleration() const;

   private:
    void start_hold(int axis);
    double continuous_angle(int axis, double angle) const;

    std::array<AxisMode, NUM_DOF> mode_{AxisMode::HOLD, AxisMode::HOLD, AxisMode::HOLD,
                                        AxisMode::HOLD, AxisMode::HOLD, AxisMode::HOLD};
    std::array<bool, NUM_DOF> passive_{};
    Vector6d position_target_{Vector6d::Zero()};
    Vector6d velocity_target_{Vector6d::Zero()};
    Vector6d effort_{Vector6d::Zero()};
    VelocityFrame velocity_frame_{VelocityFrame::HEADING};
    std::array<double, NUM_DOF> expires_{};  // [s] per VELOCITY or EFFORT axis, 0 never
    bool altitude_{false};

    KinematicShaper<2> horizontal_;
    KinematicShaper<1> vertical_;
    std::array<KinematicShaper<1>, 3> rotation_;
};

#endif  // SUB_CONTROL_REFERENCE_GENERATOR_HPP_
