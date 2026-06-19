#include "sub_control/utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace {

struct ThrusterPose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct ThrusterMap {
    // -1 to 1 (representing 1100 - 1900 µs)
    double pwm;
    // newtons
    double force;
};

// Thruster poses copied verbatim from layout.scn.j2, so they live in the
// Stonefish base_link_ned frame (X=Right, Y=Back, Z=Down). The constructor
// rotates them into the controller's FLU body frame before building B.
//   thrusters 0-3 : vertical units (pitch 90)          -> heave / roll / pitch
//   thrusters 4-7 : horizontal vectored units (45-deg) -> surge / sway / yaw
const std::array<ThrusterPose, NUM_THRUSTERS> THRUSTER_GEOMETRY{{
    //   x      y      z    roll  pitch        yaw
    {-0.23, -0.22, 0.000, 0.0, M_PI / 2.0, 0.0},         // thr 0  vertical
    {0.23, -0.22, 0.000, 0.0, M_PI / 2.0, 0.0},          // thr 1  vertical
    {-0.23, 0.22, 0.000, 0.0, M_PI / 2.0, 0.0},          // thr 2  vertical
    {0.23, 0.22, 0.000, 0.0, M_PI / 2.0, 0.0},           // thr 3  vertical
    {-0.29, -0.34, -0.08, 0.0, 0.0, -1.0 * M_PI / 4.0},  // thr 4  horizontal
    {0.29, -0.34, -0.08, 0.0, 0.0, -3.0 * M_PI / 4.0},   // thr 5  horizontal
    {-0.29, 0.34, -0.08, 0.0, 0.0, -3.0 * M_PI / 4.0},   // thr 6  horizontal
    {0.29, 0.34, -0.08, 0.0, 0.0, -1.0 * M_PI / 4.0},    // thr 7  horizontal
}};

auto THRUSTER_LOOKUP_TABLE = std::to_array<ThrusterMap>({{-1, -39.90792904},
                                                         {-0.99, -39.72258662},
                                                         {-0.98, -39.45569354},
                                                         {-0.97, -38.8774252},
                                                         {-0.96, -38.25467469},
                                                         {-0.95, -37.94329943},
                                                         {-0.94, -37.49847763},
                                                         {-0.93, -37.27606673},
                                                         {-0.92, -36.78676275},
                                                         {-0.91, -36.43090531},
                                                         {-0.9, -35.85263697},
                                                         {-0.89, -35.18540428},
                                                         {-0.88, -34.51817158},
                                                         {-0.87, -33.85093888},
                                                         {-0.86, -33.31715272},
                                                         {-0.85, -32.4719913},
                                                         {-0.84, -31.89372297},
                                                         {-0.83, -31.40441899},
                                                         {-0.82, -30.47029321},
                                                         {-0.81, -29.80306051},
                                                         {-0.8, -29.35823871},
                                                         {-0.79, -28.82445255},
                                                         {-0.78, -28.0682555},
                                                         {-0.77, -27.66791588},
                                                         {-0.76, -27.1786119},
                                                         {-0.75, -26.55586138},
                                                         {-0.74, -26.0665574},
                                                         {-0.73, -25.31036034},
                                                         {-0.72, -24.99898509},
                                                         {-0.71, -24.59864547},
                                                         {-0.7, -24.06485931},
                                                         {-0.69, -23.35314443},
                                                         {-0.68, -23.08625135},
                                                         {-0.67, -22.33005429},
                                                         {-0.66, -21.92971467},
                                                         {-0.65, -21.52937506},
                                                         {-0.64, -20.773178},
                                                         {-0.63, -20.41732056},
                                                         {-0.62, -19.83905222},
                                                         {-0.61, -19.39423042},
                                                         {-0.6, -19.03837298},
                                                         {-0.59, -18.23769375},
                                                         {-0.58, -17.79287195},
                                                         {-0.57, -17.30356797},
                                                         {-0.56, -16.68081745},
                                                         {-0.55, -16.36944219},
                                                         {-0.54, -15.79117385},
                                                         {-0.53, -15.25738769},
                                                         {-0.52, -14.85704808},
                                                         {-0.51, -14.590155},
                                                         {-0.5, -14.10085102},
                                                         {-0.49, -13.7005114},
                                                         {-0.48, -13.21120742},
                                                         {-0.47, -12.76638562},
                                                         {-0.46, -12.32156382},
                                                         {-0.45, -11.78777767},
                                                         {-0.44, -11.38743805},
                                                         {-0.43, -10.98709843},
                                                         {-0.42, -10.80916971},
                                                         {-0.41, -10.31986573},
                                                         {-0.4, -10.00849047},
                                                         {-0.39, -9.608150851},
                                                         {-0.38, -9.252293413},
                                                         {-0.37, -8.851953794},
                                                         {-0.36, -8.540578535},
                                                         {-0.35, -8.051274556},
                                                         {-0.34, -7.695417117},
                                                         {-0.33, -7.295077498},
                                                         {-0.32, -7.028184419},
                                                         {-0.31, -6.67232698},
                                                         {-0.3, -6.360951721},
                                                         {-0.29, -6.049576462},
                                                         {-0.28, -5.693719023},
                                                         {-0.27, -5.293379404},
                                                         {-0.26, -5.026486325},
                                                         {-0.25, -4.715111066},
                                                         {-0.24, -4.359253627},
                                                         {-0.23, -4.092360548},
                                                         {-0.22, -3.780985289},
                                                         {-0.21, -3.46961003},
                                                         {-0.2, -3.158234771},
                                                         {-0.19, -2.891341691},
                                                         {-0.18, -2.535484252},
                                                         {-0.17, -2.313073353},
                                                         {-0.16, -2.046180274},
                                                         {-0.15, -1.779287195},
                                                         {-0.14, -1.467911936},
                                                         {-0.13, -1.245501036},
                                                         {-0.12, -1.023090137},
                                                         {-0.11, -0.8451614175},
                                                         {-0.1, -0.667232698},
                                                         {-0.09, -0.4893039785},
                                                         {-0.08, -0.3558574389},
                                                         {-0.07, 0},
                                                         {-0.06, 0},
                                                         {-0.05, 0},
                                                         {-0.04, 0},
                                                         {-0.03, 0},
                                                         {-0.02, 0},
                                                         {-0.01, 0},
                                                         {0, 0},
                                                         {0.01, 0},
                                                         {0.02, 0},
                                                         {0.03, 0},
                                                         {0.04, 0},
                                                         {0.05, 0},
                                                         {0.06, 0.4003396188},
                                                         {0.07, 0.5337861584},
                                                         {0.08, 0.7561970578},
                                                         {0.09, 0.9786079571},
                                                         {0.1, 1.245501036},
                                                         {0.11, 1.512394116},
                                                         {0.12, 1.779287195},
                                                         {0.13, 2.135144634},
                                                         {0.14, 2.446519893},
                                                         {0.15, 2.846859512},
                                                         {0.16, 3.158234771},
                                                         {0.17, 3.51409221},
                                                         {0.18, 3.914431828},
                                                         {0.19, 4.270289267},
                                                         {0.2, 4.626146706},
                                                         {0.21, 5.026486325},
                                                         {0.22, 5.471308124},
                                                         {0.23, 5.916129922},
                                                         {0.24, 6.271987361},
                                                         {0.25, 6.67232698},
                                                         {0.26, 7.072666599},
                                                         {0.27, 7.606452757},
                                                         {0.28, 8.006792376},
                                                         {0.29, 8.540578535},
                                                         {0.3, 8.896435974},
                                                         {0.31, 9.296775592},
                                                         {0.32, 9.741597391},
                                                         {0.33, 10.14193701},
                                                         {0.34, 10.76468753},
                                                         {0.35, 11.20950933},
                                                         {0.36, 11.60984895},
                                                         {0.37, 12.1436351},
                                                         {0.38, 12.54397472},
                                                         {0.39, 13.0332787},
                                                         {0.4, 13.61154704},
                                                         {0.41, 14.10085102},
                                                         {0.42, 14.54567282},
                                                         {0.43, 15.07945898},
                                                         {0.44, 15.56876295},
                                                         {0.45, 16.19151347},
                                                         {0.46, 16.59185309},
                                                         {0.47, 17.25908579},
                                                         {0.48, 17.88183631},
                                                         {0.49, 18.41562247},
                                                         {0.5, 18.94940862},
                                                         {0.51, 19.52767696},
                                                         {0.52, 20.06146312},
                                                         {0.53, 20.81766018},
                                                         {0.54, 21.35144634},
                                                         {0.55, 21.75178596},
                                                         {0.56, 22.33005429},
                                                         {0.57, 23.35314443},
                                                         {0.58, 23.84244841},
                                                         {0.59, 24.68760983},
                                                         {0.6, 25.26587817},
                                                         {0.61, 26.02207522},
                                                         {0.62, 26.77827228},
                                                         {0.63, 27.04516536},
                                                         {0.64, 27.8458446},
                                                         {0.65, 28.33514858},
                                                         {0.66, 29.18030999},
                                                         {0.67, 29.93650705},
                                                         {0.68, 30.47029321},
                                                         {0.69, 31.09304373},
                                                         {0.7, 31.53786553},
                                                         {0.71, 32.38302694},
                                                         {0.72, 33.09474182},
                                                         {0.73, 33.53956362},
                                                         {0.74, 34.42920722},
                                                         {0.75, 35.363333},
                                                         {0.76, 36.11953005},
                                                         {0.77, 36.65331621},
                                                         {0.78, 37.45399545},
                                                         {0.79, 38.12122815},
                                                         {0.8, 38.78846084},
                                                         {0.81, 39.81155098},
                                                         {0.82, 40.6567124},
                                                         {0.83, 41.72428472},
                                                         {0.84, 42.21358869},
                                                         {0.85, 42.96978575},
                                                         {0.86, 44.21528679},
                                                         {0.87, 44.39321551},
                                                         {0.88, 45.59423436},
                                                         {0.89, 46.1725027},
                                                         {0.9, 46.92869976},
                                                         {0.91, 47.41800374},
                                                         {0.92, 48.35212952},
                                                         {0.93, 49.15280875},
                                                         {0.94, 49.77555927},
                                                         {0.95, 50.44279197},
                                                         {0.96, 50.75416723},
                                                         {0.97, 51.19898903},
                                                         {0.98, 51.43622732},
                                                         {0.99, 51.43622732},
                                                         {1, 51.43622732}});

}  // namespace

ThrusterAllocator::ThrusterAllocator() {
    // THRUSTER_GEOMETRY is in the Stonefish base_link_ned frame (X=Right, Y=Back,
    // Z=Down) while the controller commands wrenches in the FLU body frame, so
    // rotate every pose into FLU: (x, y, z)_flu = (-y, -x, -z)_ned. This is the
    // same body transform the sim IMU/DVL remappers use; det = +1, so moment arms
    // (cross products) carry over cleanly. A positive command drives thrust along
    // the thruster's +x, which in FLU makes the verticals push up (+1.0 -> up) and
    // the horizontals push forward along the 45-deg vectored ring.
    Eigen::Matrix3d ned_to_flu;
    ned_to_flu << 0.0, -1.0, 0.0,
                  -1.0, 0.0, 0.0,
                  0.0, 0.0, -1.0;

    // Build the actuation matrix B (6 x 8): column i is the body wrench produced
    // by 1 N of thrust on thruster i, [ axis_i ; r_i x axis_i ]. axis_i is the
    // thruster's +x rotated by its (roll, pitch, yaw); r_i is its position.
    Eigen::Matrix<double, NUM_DOF, NUM_THRUSTERS> B;
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        const ThrusterPose& g = THRUSTER_GEOMETRY[i];
        const Eigen::Matrix3d rot =
            (Eigen::AngleAxisd(g.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(g.pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(g.roll, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        const Eigen::Vector3d axis = ned_to_flu * (rot * Eigen::Vector3d::UnitX());
        const Eigen::Vector3d r = ned_to_flu * Eigen::Vector3d(g.x, g.y, g.z);
        B.block<3, 1>(0, i) = axis;
        B.block<3, 1>(3, i) = r.cross(axis);
    }

    // A = pinv(B): the min-norm thruster forces that realize a desired wrench.
    const Eigen::Matrix<double, NUM_THRUSTERS, NUM_DOF> A = B.completeOrthogonalDecomposition().pseudoInverse();

    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        for (int d = 0; d < NUM_DOF; ++d) {
            alloc_[t][d] = A(t, d);
        }
    }
    for (int d = 0; d < NUM_DOF; ++d) {
        for (int t = 0; t < NUM_THRUSTERS; ++t) {
            act_[d][t] = B(d, t);
        }
    }
}

std::array<double, NUM_DOF> ThrusterAllocator::wrench_from_forces(
    const std::array<double, NUM_THRUSTERS>& forces) const {
    std::array<double, NUM_DOF> wrench{};
    for (int d = 0; d < NUM_DOF; ++d) {
        double v = 0.0;
        for (int t = 0; t < NUM_THRUSTERS; ++t) {
            v += act_[d][t] * forces[t];
        }
        wrench[d] = v;
    }
    return wrench;
}

std::array<double, NUM_THRUSTERS> ThrusterAllocator::allocate(const std::array<double, NUM_DOF>& wrench,
                                                              double max_force,
                                                              const std::array<double, NUM_DOF>& axis_weights) const {
    std::array<double, NUM_THRUSTERS> unconstrained{};
    double peak_force = 0.0;
    for (int thruster = 0; thruster < NUM_THRUSTERS; ++thruster) {
        for (int axis = 0; axis < NUM_DOF; ++axis) {
            unconstrained[thruster] += alloc_[thruster][axis] * wrench[axis];
        }
        peak_force = std::max(peak_force, std::fabs(unconstrained[thruster]));
    }
    if (max_force > 0.0 && peak_force <= max_force) {
        return unconstrained;
    }

    Eigen::Matrix<double, NUM_DOF, NUM_THRUSTERS> B;
    Eigen::Matrix<double, NUM_DOF, 1> desired;
    Eigen::DiagonalMatrix<double, NUM_DOF> weights;
    for (int d = 0; d < NUM_DOF; ++d) {
        desired(d) = wrench[d];
        weights.diagonal()(d) = std::max(axis_weights[d], 1e-3);
        for (int t = 0; t < NUM_THRUSTERS; ++t) {
            B(d, t) = act_[d][t];
        }
    }

    const Eigen::Matrix<double, NUM_DOF, NUM_THRUSTERS> weighted_b = weights * B;
    const Eigen::Matrix<double, NUM_DOF, 1> weighted_desired = weights * desired;
    constexpr double regularization = 1e-8;
    const Eigen::Matrix<double, NUM_THRUSTERS, NUM_THRUSTERS> hessian =
        weighted_b.transpose() * weighted_b +
        regularization * Eigen::Matrix<double, NUM_THRUSTERS, NUM_THRUSTERS>::Identity();
    const Eigen::Matrix<double, NUM_THRUSTERS, 1> gradient_offset = weighted_b.transpose() * weighted_desired;

    Eigen::Matrix<double, NUM_THRUSTERS, 1> forces = hessian.ldlt().solve(gradient_offset);
    if (max_force <= 0.0) {
        forces.setZero();
    } else {
        forces = forces.cwiseMax(-max_force).cwiseMin(max_force);

        // Projected gradient refinement solves the box-constrained problem. The
        // row-sum bound is a cheap upper bound on the largest eigenvalue.
        double lipschitz = 0.0;
        for (int row = 0; row < NUM_THRUSTERS; ++row) {
            lipschitz = std::max(lipschitz, hessian.row(row).cwiseAbs().sum());
        }
        const double step = 1.0 / std::max(lipschitz, 1e-6);
        for (int iteration = 0; iteration < 256; ++iteration) {
            const auto gradient = hessian * forces - gradient_offset;
            forces = (forces - step * gradient).cwiseMax(-max_force).cwiseMin(max_force);
        }
    }

    std::array<double, NUM_THRUSTERS> result{};
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        result[t] = forces(t);
    }
    return result;
}

std::array<double, NUM_DOF> ThrusterAllocator::max_wrench(double max_force) const {
    std::array<double, NUM_DOF> mw{};
    for (int d = 0; d < NUM_DOF; ++d) {
        double cost = 0.0;  // peak thruster force per unit wrench in this DOF
        for (int t = 0; t < NUM_THRUSTERS; ++t) {
            cost = std::max(cost, std::fabs(alloc_[t][d]));
        }
        mw[d] = (cost > 1e-9) ? max_force / cost : 0.0;
    }
    return mw;
}

double force_to_norm(double force_n) {
    if (std::fabs(force_n) < 1e-6) {
        return 0.0;
    }
    if (force_n <= THRUSTER_LOOKUP_TABLE[0].force) {
        return -1.0;
    }
    if (force_n >= THRUSTER_LOOKUP_TABLE[THRUSTER_LOOKUP_TABLE.size() - 1].force) {
        return 1.0;
    }

    for (size_t i = 1; i < THRUSTER_LOOKUP_TABLE.size(); ++i) {
        if (THRUSTER_LOOKUP_TABLE[i].force >= force_n) {
            const double o0 = THRUSTER_LOOKUP_TABLE[i - 1].force;
            const double o1 = THRUSTER_LOOKUP_TABLE[i].force;
            const double t = (o1 > o0) ? (force_n - o0) / (o1 - o0) : 0.0;

            return THRUSTER_LOOKUP_TABLE[i - 1].pwm +
                   t * (THRUSTER_LOOKUP_TABLE[i].pwm - THRUSTER_LOOKUP_TABLE[i - 1].pwm);
        }
    }
    return 1.0;
}

double norm_to_force(double normalized) {
    normalized = std::clamp(normalized, -1.0, 1.0);
    if (normalized <= THRUSTER_LOOKUP_TABLE.front().pwm) {
        return THRUSTER_LOOKUP_TABLE.front().force;
    }
    if (normalized >= THRUSTER_LOOKUP_TABLE.back().pwm) {
        return THRUSTER_LOOKUP_TABLE.back().force;
    }

    for (size_t i = 1; i < THRUSTER_LOOKUP_TABLE.size(); ++i) {
        if (THRUSTER_LOOKUP_TABLE[i].pwm >= normalized) {
            const double x0 = THRUSTER_LOOKUP_TABLE[i - 1].pwm;
            const double x1 = THRUSTER_LOOKUP_TABLE[i].pwm;
            const double fraction = (normalized - x0) / (x1 - x0);
            return THRUSTER_LOOKUP_TABLE[i - 1].force +
                   fraction * (THRUSTER_LOOKUP_TABLE[i].force - THRUSTER_LOOKUP_TABLE[i - 1].force);
        }
    }
    return THRUSTER_LOOKUP_TABLE.back().force;
}

std::array<double, 3> attitude_error(const std::array<double, 3>& target_rpy,
                                     const std::array<double, 3>& current_rpy) {
    const auto rotation = [](const std::array<double, 3>& rpy) {
        return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    };

    const Eigen::Matrix3d error_rotation = rotation(current_rpy).transpose() * rotation(target_rpy);
    const Eigen::AngleAxisd error(error_rotation);
    const Eigen::Vector3d rotation_vector = error.axis() * error.angle();
    return {rotation_vector.x(), rotation_vector.y(), rotation_vector.z()};
}
