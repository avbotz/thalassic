// Mission executive tests. They need no other ROS node: control/error is
// scripted by writing MissionNode's fields directly, the way its callbacks
// would, and trees are ticked from this thread.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sub_mission/nodes/mission.hpp"
#include "sub_mission/utils.hpp"
#include "sub_vision_interfaces/msg/detection_array.hpp"

using namespace std::chrono_literals;

namespace {

class RosEnvironment : public ::testing::Environment {
   public:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};

const auto *const ros_environment = ::testing::AddGlobalTestEnvironment(new RosEnvironment);

const std::filesystem::path RESOURCES = SUB_MISSION_RESOURCES;

class MissionTest : public ::testing::Test {
   protected:
    void SetUp() override {
        node = std::make_shared<MissionNode>();
        node->killed = false;
        node->registerNodes(factory);
        for (const auto &entry : std::filesystem::directory_iterator(RESOURCES / "trees")) {
            factory.registerBehaviorTreeFromFile(entry.path());
        }
    }

    BT::Tree tree(const std::string &body) {
        factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)" + body +
                                             "</BehaviorTree></root>");
        return factory.createTree("Test");
    }

    // A control/error message reporting every axis on target.
    void arrive() {
        node->control_errors.fill(0.0);
        for (auto &updates : node->control_error_updates) {
            ++updates;
        }
    }

    // Ticks until the tree stops RUNNING or `limit` passes, reporting the
    // vehicle on target after every tick when `arriving`.
    BT::NodeStatus run(BT::Tree &tree, const bool arriving, const std::chrono::milliseconds limit = 3s) {
        const auto deadline = std::chrono::steady_clock::now() + limit;
        BT::NodeStatus status = tree.tickOnce();
        while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
            if (arriving) {
                arrive();
            }
            std::this_thread::sleep_for(5ms);
            status = tree.tickOnce();
        }
        return status;
    }

    struct Seen {
        std::string class_id;
        double score;
        double bearing_horizontal;
        double bearing_vertical = 0.0;
        double distance = std::numeric_limits<double>::quiet_NaN();  // as sub_vision without a range
        std::vector<std::pair<std::string, std::string>> extra = {};
    };

    // One camera frame from sub_vision, delivered to the node's VisionClient.
    void frame(const std::string &camera, const std::string &task, const std::vector<Seen> &seen) {
        const std::string topic = camera == "down" ? "vision/detections_down" : "vision/detections";
        auto &publisher = publishers[topic];
        if (!publisher) {
            publisher =
                node->create_publisher<sub_vision_interfaces::msg::DetectionArray>(topic, rclcpp::SensorDataQoS());
        }
        sub_vision_interfaces::msg::DetectionArray msg;
        msg.task = task;
        for (const Seen &s : seen) {
            sub_vision_interfaces::msg::Detection detection;
            detection.detection.results.resize(1);
            detection.detection.results[0].hypothesis.class_id = s.class_id;
            detection.detection.results[0].hypothesis.score = s.score;
            detection.bearing_horizontal = static_cast<float>(s.bearing_horizontal);
            detection.bearing_vertical = static_cast<float>(s.bearing_vertical);
            detection.distance_m = static_cast<float>(s.distance);
            for (const auto &[key, value] : s.extra) {
                detection.extra.emplace_back();
                detection.extra.back().key = key;
                detection.extra.back().value = value;
            }
            msg.detections.push_back(detection);
        }
        const auto before = node->vision().latest(camera).received_at;
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (node->vision().latest(camera).received_at == before && std::chrono::steady_clock::now() < deadline) {
            publisher->publish(msg);
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(5ms);
        }
        ASSERT_NE(node->vision().latest(camera).received_at, before) << "frame not delivered";
    }

    // Ticks while showing `seen` to `camera`, until done or `limit` passes,
    // reporting the vehicle on target after every tick when `arriving`.
    BT::NodeStatus watch(BT::Tree &tree, const std::string &camera, const std::string &task,
                         const std::vector<Seen> &seen, const std::chrono::milliseconds limit = 3s,
                         const bool arriving = true) {
        const auto deadline = std::chrono::steady_clock::now() + limit;
        BT::NodeStatus status = tree.tickOnce();
        while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
            if (arriving) {
                arrive();
            }
            frame(camera, task, seen);
            status = tree.tickOnce();
        }
        return status;
    }

    // A frame of the vehicle's, as description_launch.py publishes it.
    void mount(const std::string &frame, const double x, const double y, const double z) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.child_frame_id = frame;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        transform.transform.rotation.w = 1.0;
        node->tfBuffer().setTransform(transform, "test", true);
    }

    std::shared_ptr<MissionNode> node;
    BT::BehaviorTreeFactory factory;
    std::map<std::string, rclcpp::Publisher<sub_vision_interfaces::msg::DetectionArray>::SharedPtr> publishers;
};

TEST_F(MissionTest, EveryPackagedMissionLoads) {
    std::size_t count = 0;
    for (const auto &entry : std::filesystem::directory_iterator(RESOURCES / "missions")) {
        try {
            const BT::Tree loaded = factory.createTreeFromFile(entry.path());
        } catch (const std::exception &error) {
            ADD_FAILURE() << entry.path().filename() << ": " << error.what();
        }
        ++count;
    }
    EXPECT_GT(count, 0u);
}

TEST_F(MissionTest, CommandsCarryTargetsUnchanged) {
    // The mission works in sub_control's frames: odom ENU and body FLU.
    rclcpp::Clock clock;
    const SetpointMsg position = positionCommand(clock, {1.0, 2.0, -3.0}, false);
    EXPECT_DOUBLE_EQ(position.position[1], 2.0);
    EXPECT_DOUBLE_EQ(position.position[2], -3.0);
    const SetpointMsg velocity = linearVelocityCommand(clock, {UNSPECIFIED_PORT, 0.5, -0.25});
    EXPECT_DOUBLE_EQ(velocity.velocity[1], 0.5);
    EXPECT_DOUBLE_EQ(velocity.velocity[2], -0.25);
    EXPECT_DOUBLE_EQ(attitudeCommand(clock, {UNSPECIFIED_PORT, UNSPECIFIED_PORT, 0.5}).position[5], 0.5);
    EXPECT_DOUBLE_EQ(angularVelocityCommand(clock, {UNSPECIFIED_PORT, UNSPECIFIED_PORT, -0.5}).velocity[5], -0.5);
}

TEST_F(MissionTest, NormalizeAngleWrapsAnyAngle) {
    EXPECT_DOUBLE_EQ(normalizeAngle(0.5), 0.5);
    EXPECT_NEAR(normalizeAngle(2.0 * M_PI + 0.5), 0.5, 1e-12);
    EXPECT_NEAR(normalizeAngle(-3.5 * M_PI), 0.5 * M_PI, 1e-12);
    EXPECT_LE(std::fabs(normalizeAngle(1e12)), M_PI);
    // An infinite angle (a bad port value or bearing) used to loop forever.
    EXPECT_TRUE(std::isnan(normalizeAngle(std::numeric_limits<double>::infinity())));
    EXPECT_TRUE(std::isnan(normalizeAngle(UNSPECIFIED_PORT)));
}

TEST_F(MissionTest, OdometryAndErrorsAreTakenUnchanged) {
    nav_msgs::msg::Odometry odometry;
    odometry.pose.pose.position.x = 1.0;
    odometry.pose.pose.position.y = 2.0;
    odometry.pose.pose.position.z = -3.0;
    odometry.pose.pose.orientation.w = std::cos(0.25);
    odometry.pose.pose.orientation.z = std::sin(0.25);  // yaw 0.5, a turn left
    node->odometry_callback(odometry);
    EXPECT_DOUBLE_EQ(node->measured_pos[1], 2.0);
    EXPECT_DOUBLE_EQ(node->measured_pos[2], -3.0);
    EXPECT_NEAR(node->measured_att[2], 0.5, 1e-9);

    sub_control_interfaces::msg::Error error;
    error.pos_error = {0.1, 0.2, -0.3};
    error.att_error = {0.0, 0.0, 0.4};
    node->control_error_callback(error);
    EXPECT_DOUBLE_EQ(node->control_errors[1], 0.2);
    EXPECT_DOUBLE_EQ(node->control_errors[2], -0.3);
    EXPECT_DOUBLE_EQ(node->control_errors[8], 0.4);
}

TEST_F(MissionTest, PosSetpointWaitsForArrival) {
    auto t = tree(R"(<PosSetpoint z="-1.0"/>)");
    EXPECT_EQ(run(t, false, 200ms), BT::NodeStatus::RUNNING);
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->commanded_pos[2], -1.0);
}

TEST_F(MissionTest, PosSetpointFailsWhenItCannotArrive) {
    auto t = tree(R"(<PosSetpoint z="-1.0" timeout_msec="100"/>)");
    EXPECT_EQ(run(t, false), BT::NodeStatus::FAILURE);
}

TEST_F(MissionTest, MissingBlackboardEntryFailsWithoutMoving) {
    auto t = tree(R"(<PosSetpoint x="{@not_set}"/>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(std::isnan(node->commanded_pos[0]));
    EXPECT_TRUE(std::isnan(node->commanded_pos[1]));
}

TEST_F(MissionTest, MoveRelativeTreatsNanAsNoOffset) {
    // As in gate.xml: x moves, y and z stay where they are.
    auto t = tree(R"(<MoveRelative x="1.0" y="nan" z="nan"/>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->commanded_pos[0], 1.0);
    EXPECT_DOUBLE_EQ(node->commanded_pos[1], 0.0);
    EXPECT_TRUE(std::isnan(node->commanded_pos[2]));
}

TEST_F(MissionTest, MoveRelativeIsForwardLeftUp) {
    // Facing odom +y, a quarter turn left of the start: forward is +y and
    // left is -x.
    node->commanded_pos = {0.0, 0.0, -1.0};
    node->commanded_att[2] = M_PI_2;
    auto t = tree(R"(<MoveRelative x="1.0" y="2.0" z="-0.5"/>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_NEAR(node->commanded_pos[0], -2.0, 1e-9);
    EXPECT_NEAR(node->commanded_pos[1], 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(node->commanded_pos[2], -1.5);
}

TEST_F(MissionTest, MoveRelativeUpRaisesAnAltitude) {
    auto t = tree(R"(<Sequence><AltitudeSetpoint z="1.0"/><MoveRelative z="0.5"/></Sequence>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(node->commanded_altitude);
    EXPECT_DOUBLE_EQ(node->commanded_pos[2], 1.5);
}

TEST_F(MissionTest, MoveRelativeAfterAVelocityStartsFromWhereTheVehicleIs) {
    // A velocity leaves no target behind: the next relative move is from the
    // measured position, not from the target held before the velocity.
    node->commanded_pos = {0.0, 0.0, -1.0};
    node->measured_pos = {3.0, 0.5, -1.0};
    auto t = tree(R"(<Sequence><VelocitySetpoint x="0.25"/><MoveRelative x="1.0"/></Sequence>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->commanded_pos[0], 4.0);
    EXPECT_DOUBLE_EQ(node->commanded_pos[1], 0.5);
}

TEST_F(MissionTest, MoveRelativePosUsesOdomAxes) {
    // Facing odom +y makes no difference: x and y are odom's.
    node->commanded_pos = {0.0, 0.0, -1.0};
    node->commanded_att[2] = M_PI_2;
    auto t = tree(R"(<MoveRelativePos x="1.0" y="2.0"/>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->commanded_pos[0], 1.0);
    EXPECT_DOUBLE_EQ(node->commanded_pos[1], 2.0);
    EXPECT_DOUBLE_EQ(node->commanded_pos[2], -1.0);
}

TEST_F(MissionTest, NavigateToTransformMovesTheTargetFrameOntoTheSource) {
    // The torpedo is 0.05 m ahead of the camera, 0.01 m to its right and
    // 0.25 m below it: the vehicle moves back, left and up by as much.
    mount("cam", 0.30, 0.0, 0.20);
    mount("torpedo", 0.35, -0.01, -0.05);
    node->commanded_pos = {1.0, 2.0, -1.0};
    node->commanded_att[2] = M_PI_2;  // facing odom +y: forward is +y, left is -x
    auto t = tree(R"(<NavigateToTransform source_frame="cam" target_frame="torpedo"/>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_NEAR(node->commanded_pos[0], 1.0 - 0.01, 1e-9);
    EXPECT_NEAR(node->commanded_pos[1], 2.0 - 0.05, 1e-9);
    EXPECT_NEAR(node->commanded_pos[2], -1.0 + 0.25, 1e-9);
}

TEST_F(MissionTest, NavigateToTransformFailsWithoutTheFrames) {
    auto t = tree(R"(<NavigateToTransform source_frame="cam" target_frame="nowhere"/>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(std::isnan(node->commanded_pos[0]));
}

TEST_F(MissionTest, TimedVelocityHoldsAfterItsDuration) {
    // The duration from a Script, as in slalom.xml.
    auto t = tree(R"(<Sequence><Script code="ms := 50"/><TimedVelocity y="0.25" duration_msec="{ms}"/></Sequence>)");
    EXPECT_EQ(run(t, false), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[1], 0.0);
    EXPECT_TRUE(std::isnan(node->commanded_pos[1]));
}

TEST_F(MissionTest, AttitudeMovesTimeOut) {
    auto absolute = tree(R"(<AttSetpoint yaw="1.0" timeout_msec="100"/>)");
    EXPECT_EQ(run(absolute, false), BT::NodeStatus::FAILURE);
    auto relative = tree(R"(<AddAttSetpoint yaw="1.0" timeout_msec="100"/>)");
    EXPECT_EQ(run(relative, false), BT::NodeStatus::FAILURE);
    auto altitude = tree(R"(<AltitudeSetpoint z="1.0" timeout_msec="100"/>)");
    EXPECT_EQ(run(altitude, false), BT::NodeStatus::FAILURE);
}

TEST_F(MissionTest, SpinThatCannotTurnTimesOutHoldingHeading) {
    node->measured_att[2] = 0.3;
    auto t = tree(R"(<Spin yaw="6.2831853" timeout_msec="100"/>)");
    EXPECT_EQ(run(t, false), BT::NodeStatus::FAILURE);
    EXPECT_DOUBLE_EQ(node->last_angvel_setpoint[2], 0.0);
    EXPECT_DOUBLE_EQ(node->commanded_att[2], 0.3);
}

TEST_F(MissionTest, SpinHandsOverToTheFinalHeadingBeforeTheEnd) {
    // Turning at the rate commanded, a 2 rad spin at 2 rad/s takes 1 s; the
    // final heading is a target from pi/2 before the end, about 0.2 s in, so
    // that sub_control brakes into it.
    auto t = tree(R"(<Spin yaw="-2.0" rate="2.0"/>)");
    const auto started = std::chrono::steady_clock::now();
    BT::NodeStatus status = t.tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::isnan(node->commanded_att[2]) &&
           std::chrono::steady_clock::now() - started < 2s) {
        arrive();
        std::this_thread::sleep_for(5ms);
        status = t.tickOnce();
    }
    EXPECT_LT(std::chrono::steady_clock::now() - started, 700ms);
    EXPECT_DOUBLE_EQ(node->commanded_att[2], -2.0);
    EXPECT_DOUBLE_EQ(node->last_angvel_setpoint[2], 0.0);
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
}

TEST_F(MissionTest, SavePoseWritesOnlyConnectedPorts) {
    node->commanded_pos = {1.0, 2.0, -3.0};
    node->commanded_att[2] = 0.5;
    auto t = tree(R"(<SavePose yaw="{@entry_yaw}"/>)");
    EXPECT_EQ(run(t, false), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(t.rootBlackboard()->get<double>("entry_yaw"), 0.5);
}

TEST_F(MissionTest, StopReleasesEveryTarget) {
    auto t = tree(R"(<Sequence><VelocitySetpoint x="0.25"/><AttSetpoint yaw="0.4"/><Stop/></Sequence>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
    EXPECT_TRUE(std::isnan(node->commanded_att[2]));
    EXPECT_TRUE(std::isnan(node->commanded_pos[0]));
}

TEST_F(MissionTest, TargetsEndTheRecordedVelocity) {
    // What WaitUntilHit measures speed against: an axis given a target no
    // longer moves at the velocity it had, and one left alone still does.
    auto t = tree(R"(<Sequence><VelocitySetpoint x="0.25" z="-0.1"/><AngularVelocitySetpoint yaw="0.3"/>
                                <PosSetpoint x="1.0" y="0.0"/><AttSetpoint yaw="0.5"/></Sequence>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[2], -0.1);
    EXPECT_DOUBLE_EQ(node->last_angvel_setpoint[2], 0.0);
}

TEST_F(MissionTest, TransitTakesDepthThenHops) {
    auto t = tree(R"(<SubTree ID="Transit" z="-1.2" x="4.0" y="-0.8"/>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->commanded_pos[2], -1.2);
    EXPECT_DOUBLE_EQ(node->commanded_pos[0], 4.0);
    EXPECT_DOUBLE_EQ(node->commanded_pos[1], -0.8);
}

TEST_F(MissionTest, TransitCarriesOnWhenAMoveFails) {
    // Every move fails at once here (as a timed-out one does, only sooner),
    // yet the transit still succeeds and the next task gets its attempt.
    node->killed = true;
    auto t = tree(R"(<SubTree ID="Transit" z="-1.2" x="4.0" y="0.0"/>)");
    EXPECT_EQ(run(t, false), BT::NodeStatus::SUCCESS);
}

TEST_F(MissionTest, TaskOverBudgetRecoversEntryHeading) {
    node->commanded_att[2] = 0.7;
    auto t = tree(R"(
        <Sequence>
          <SavePose yaw="{@task_entry_yaw}"/>
          <Fallback>
            <Timeout msec="200">
              <Sequence><AttSetpoint yaw="2.0"/><Sleep msec="10000"/></Sequence>
            </Timeout>
            <SubTree ID="RecoverTaskEntry"/>
          </Fallback>
        </Sequence>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->commanded_att[2], 0.7);
}

TEST_F(MissionTest, PoolMissionsSetEveryPoolSideValue) {
    // What competition.xml and its task trees read from the pool files.
    const std::vector<std::string> keys = {"gate_exit_yaw", "torp_search_yaw", "octagon_search_yaw"};
    std::size_t pools = 0;
    for (const auto &entry : std::filesystem::directory_iterator(RESOURCES / "missions")) {
        std::stringstream text;
        text << std::ifstream(entry.path()).rdbuf();
        if (text.str().find(R"(<SubTree ID="CompetitionRun")") == std::string::npos) {
            continue;
        }
        for (const std::string &key : keys) {
            EXPECT_NE(text.str().find("@" + key + " :="), std::string::npos)
                << entry.path().filename() << " does not set @" << key;
        }
        ++pools;
    }
    EXPECT_GT(pools, 0u);
}

TEST_F(MissionTest, VisionNeverSteersShallowerThanHalfAMetre) {
    // Bins without a range, holding a camera distance the pool cannot give:
    // the old bins.xml step, which drove the sub up through the surface.
    node->measured_pos = {0.0, 0.0, -0.6};
    auto t = tree(R"(<DownContinuousAlign task="bin" desired_distance="2.3" min_msec="0" update_msec="1"
                                          timeout_msec="400"/>)");
    watch(t, "down", "bin", {{"0", 0.9, 0.0}}, 1s);
    EXPECT_DOUBLE_EQ(node->commanded_pos[2], -0.5);
}

TEST_F(MissionTest, SweepAngleTakesThePipeOnTheGivenSide) {
    // Red ahead; the white pipe to its right scores higher.
    const std::vector<Seen> row = {{"0", 0.9, 0.0}, {"1", 0.5, -0.3}, {"1", 0.9, 0.3}};
    auto left = tree(R"(<SweepAngle task="slalom" class_id="1" side="left" reference_yaw="0.0" attempts="2"
                                    yaw="{@white_yaw}"/>)");
    ASSERT_EQ(watch(left, "front", "slalom", row), BT::NodeStatus::SUCCESS);
    EXPECT_NEAR(left.rootBlackboard()->get<double>("white_yaw"), 0.3, 1e-6);

    factory.clearRegisteredBehaviorTrees();
    auto any = tree(R"(<SweepAngle task="slalom" class_id="1" attempts="2" yaw="{@white_yaw}"/>)");
    ASSERT_EQ(watch(any, "front", "slalom", row), BT::NodeStatus::SUCCESS);
    EXPECT_NEAR(any.rootBlackboard()->get<double>("white_yaw"), -0.3, 1e-6);
}

TEST_F(MissionTest, SweepAngleRejectsAnUnknownSide) {
    auto t = tree(R"(<SweepAngle task="slalom" class_id="1" side="Left" reference_yaw="0.0" yaw="{@w}"/>)");
    EXPECT_EQ(run(t, true), BT::NodeStatus::FAILURE);
}

TEST_F(MissionTest, AlignToDetectionTurnsAndDivesTowardsTheTarget) {
    // Right of and below the image center: turn clockwise, to a smaller yaw,
    // and go deeper, to a smaller z.
    node->measured_pos = {0.0, 0.0, -1.0};
    auto t = tree(R"(<AlignToDetection task="gate" depth="true" timeout_msec="300"/>)");
    watch(t, "front", "gate", {{"0", 0.9, 0.2, 0.2}}, 1s);
    EXPECT_NEAR(node->commanded_att[2], -0.2, 1e-6);
    EXPECT_NEAR(node->commanded_pos[2], -1.0 - 1.5 * 0.2, 1e-6);
}

TEST_F(MissionTest, DownAlignMovesOverTheTarget) {
    // Down camera: image up is forward and image right is the vehicle's
    // right, so a target up and right of the center is ahead and to the right.
    auto t = tree(R"(<DownAlignToDetection task="bin" tolerance="0.1" update_msec="1" timeout_msec="300"/>)");
    watch(t, "down", "bin", {{"0", 0.9, 0.2, -0.2}}, 1s);
    EXPECT_NEAR(node->commanded_pos[0], std::tan(0.2), 1e-6);
    EXPECT_NEAR(node->commanded_pos[1], -std::tan(0.2), 1e-6);
}

TEST_F(MissionTest, DownSweepStepsRightFirst) {
    auto t = tree(R"(<DownForwardSweepAlign task="bin" forward_step="1.0" lateral_sweep_step="0.5"
                                            update_msec="10000"/>)");
    EXPECT_EQ(run(t, false, 100ms), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->commanded_pos[0], 1.0);
    EXPECT_DOUBLE_EQ(node->commanded_pos[1], -0.5);
}

TEST_F(MissionTest, ForwardContinuousAlignTurnsTowardTheTargetWhileMovingForward) {
    auto t = tree(R"(<ForwardContinuousAlign task="gate" forward_velocity="0.5" timeout_msec="5000"/>)");
    EXPECT_EQ(watch(t, "front", "gate", {{"8", 0.9, 0.2}}, 200ms), BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_att[2], -0.2, 1e-6);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.5);
}

TEST_F(MissionTest, ForwardContinuousAlignStopsWhenClose) {
    auto t = tree(R"(<ForwardContinuousAlign task="gate" forward_velocity="0.5" close_distance="2.0"/>)");
    EXPECT_EQ(watch(t, "front", "gate", {{"8", 0.9, 0.0, 0.0, 1.5}}), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
}

TEST_F(MissionTest, LateralAlignStepsTowardATargetOnTheRight) {
    // Right of center at 2 m: step right (a smaller y) by 2 tan(0.2), keeping
    // the heading.
    auto t = tree(R"(<LateralAlignToDetection task="gate" class_id="8" target_distance="2.0" align_depth="false"
                                              max_lateral_step="1.0" update_msec="1" timeout_msec="5000"/>)");
    EXPECT_EQ(watch(t, "front", "gate", {{"8", 0.9, 0.2}}, 200ms), BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_pos[0], 0.0, 1e-6);
    EXPECT_NEAR(node->commanded_pos[1], -2.0 * std::tan(0.2), 1e-6);
    EXPECT_TRUE(std::isnan(node->commanded_att[2]));
}

TEST_F(MissionTest, LateralAlignLocksATargetOffsetToTheLeft) {
    // Centered, but aiming 0.5 m left of the detection: one ray locks a target
    // 0.5 m to the left.
    auto t = tree(R"(<LateralAlignToDetection task="gate" class_id="8" target_distance="2.0" align_depth="false"
                                              target_offset_left="0.5" lock_target="true" lock_frames="1"
                                              timeout_msec="5000"/>)");
    EXPECT_EQ(watch(t, "front", "gate", {{"8", 0.9, 0.0}}, 200ms, false), BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_pos[1], 0.5, 1e-6);
}

TEST_F(MissionTest, SearchForDetectionSweepsLeftFirstAndStopsOnADetection) {
    auto t = tree(R"(<SearchForDetection task="slalom" class_id="2" y="0.10" sweep_left_m="1.0"
                                         sweep_right_m="2.0"/>)");
    EXPECT_EQ(run(t, false, 100ms), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[1], 0.10);
    node->measured_pos[1] = 1.1;  // 1 m left: back the other way
    EXPECT_EQ(run(t, false, 100ms), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[1], -0.10);
    EXPECT_EQ(watch(t, "front", "slalom", {{"2", 0.9, 0.1}}), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[1], 0.0);
}

TEST_F(MissionTest, SearchForDetectionTakesItsSettingsFromSubTreeLiterals) {
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Inner">
        <SearchForDetection task="gate" y="{v}" sweep_left_m="{l}" sweep_right_m="{r}" timeout_msec="{t}"/>
        </BehaviorTree></root>)");
    auto t = tree(R"(<SubTree ID="Inner" v="0.10" l="5.0" r="10.0" t="140000"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[1], 0.10);
    t.haltTree();
}

TEST_F(MissionTest, VisionVelocityKeepsTheSpeedOfAnAxisItLeavesAlone) {
    auto v = tree(R"(<VelocitySetpoint z="-0.2"/>)");
    EXPECT_EQ(v.tickOnce(), BT::NodeStatus::SUCCESS);
    auto t = tree(R"(<SearchForDetection task="gate" x="0.1" timeout_msec="0"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.1);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[1], 0.0);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[2], -0.2);
    t.haltTree();
}

TEST_F(MissionTest, SelectGateRoleTakesTheLeftOpeningsRole) {
    auto t = tree(R"(<SelectGateRole task="gate" class_id="8" confirmation_frames="2"/>)");
    EXPECT_EQ(watch(t, "front", "gate", {{"8", 0.9, 0.0, 0.0, 3.0, {{"gate_role", "SEARCH"}}}}),
              BT::NodeStatus::SUCCESS);
    EXPECT_EQ(node->role, "SEARCH");
}

TEST_F(MissionTest, ForwardFixedTransitStopsOncePastAlongItsHeading) {
    node->measured_att[2] = M_PI_2;  // heading odom +y
    auto t = tree(R"(<ForwardFixedTransit vision_distance_m="1.0" pass_distance="0.5" completion_tolerance_m="0.0"
                                          forward_velocity="0.25"/>)");
    EXPECT_EQ(run(t, false, 50ms), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.25);
    node->measured_pos[0] = 1.6;  // sideways: no progress
    EXPECT_EQ(run(t, false, 50ms), BT::NodeStatus::RUNNING);
    node->measured_pos[1] = 1.6;
    EXPECT_EQ(run(t, false), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
}

TEST_F(MissionTest, ForwardFixedTransitFailsOnAMissingMaxDist) {
    auto t = tree(R"(<ForwardFixedTransit vision_distance_m="1.0" max_dist="{missing}"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
}

TEST_F(MissionTest, ForwardFixedTransitTakesMaxDistFromASubTreeLiteral) {
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4">
      <BehaviorTree ID="Pass"><ForwardFixedTransit vision_distance_m="1.0" max_dist="{max_transit_distance}"/></BehaviorTree>
      <BehaviorTree ID="Outer"><SubTree ID="Pass" max_transit_distance="0.5"/></BehaviorTree></root>)");
    auto t = factory.createTree("Outer");
    // A 1 m pass over the 0.5 m limit: the literal was read, not the 10 m default.
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
}

TEST_F(MissionTest, OrientToDetectionAtDistMovesOntoTheBoardsNormal) {
    // Board dead ahead at 3 m whose face is squared by a 0.2 rad turn left:
    // the point 1.7 m out along its normal is ahead and to the right.
    auto t = tree(R"(<OrientToDetectionAtDist task="torp_find" class_id="0" desired_distance="1.7"
                                              max_position_step="0.6" require_heading="true" update_msec="1"/>)");
    const double heading_deg = 0.2 * 180.0 / M_PI;
    EXPECT_EQ(watch(t, "front", "torp_find",
                    {{"0", 0.9, 0.0, 0.0, 3.0, {{"heading_yaw_deg", std::to_string(heading_deg)}}}}, 300ms),
              BT::NodeStatus::RUNNING);
    const double forward = 3.0 - 1.7 * std::cos(0.2);
    const double left = -1.7 * std::sin(0.2);
    EXPECT_NEAR(node->commanded_pos[0], 0.6 * forward / std::hypot(forward, left), 1e-3);
    EXPECT_NEAR(node->commanded_pos[1], 0.6 * left / std::hypot(forward, left), 1e-3);
    EXPECT_NEAR(node->commanded_att[2], 0.0, 1e-6);
}

TEST_F(MissionTest, OrientToDetectionAtDistTurnsToABoardWithoutAHeading) {
    auto t = tree(R"(<OrientToDetectionAtDist task="torp_find" class_id="0" require_heading="true" update_msec="1"
                                              timeout_msec="5000"/>)");
    // Right of center and cut off by the image edge, so no heading: turn
    // clockwise toward it, and do not move.
    EXPECT_EQ(watch(t, "front", "torp_find", {{"0", 0.9, 0.3, 0.0, 3.0, {{"heading_status", "unavailable"}}}}, 300ms),
              BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_att[2], -0.3, 1e-6);
    EXPECT_TRUE(std::isnan(node->commanded_pos[0]));
}

TEST_F(MissionTest, OrientToDetectionAtDistWithoutAHeadingTurnsOncePerUpdate) {
    auto t = tree(R"(<OrientToDetectionAtDist task="torp_find" class_id="0" require_heading="true" update_msec="100000"
                                              timeout_msec="5000"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    frame("front", "torp_find", {{"0", 0.9, 0.3, 0.0, 3.0}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // the first turn is immediate
    EXPECT_NEAR(node->commanded_att[2], -0.3, 1e-6);
    node->measured_att[2] = -0.3;
    frame("front", "torp_find", {{"0", 0.9, 0.1, 0.0, 3.0}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_att[2], -0.3, 1e-6);  // the next waits for update_msec
}

TEST_F(MissionTest, DownAlignTurnsByThePathMarkersYaw) {
    // A path marker 20 degrees counter-clockwise of the heading: once three
    // frames agree, turn left by that much.
    auto t = tree(R"(<DownContinuousAlign task="path" update_msec="1" timeout_msec="5000"/>)");
    EXPECT_EQ(watch(t, "down", "path", {{"0", 0.9, 0.0, 0.0, 1.0, {{"yaw_deg", "20.0"}}}}, 300ms),
              BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_att[2], 20.0 * M_PI / 180.0, 1e-6);
}

TEST_F(MissionTest, DownSweepAlignGoesRightFirst) {
    auto t = tree(R"(<DownSweepAlign task="bin" lateral_step="1.0" sample_timeout_msec="20"/>)");
    EXPECT_EQ(run(t, false, 100ms), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->commanded_pos[0], 0.0);
    EXPECT_DOUBLE_EQ(node->commanded_pos[1], -1.0);
}

TEST_F(MissionTest, OctagonSurfaceSweepTurnsLeftThenRises) {
    auto t = tree(R"(<OctagonSurfaceSweep task="octagon_search_image" yaw_step_deg="30.0" surface_z="-0.3"/>)");
    EXPECT_EQ(run(t, false, 50ms), BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_att[2], 30.0 * M_PI / 180.0, 1e-9);
    EXPECT_EQ(watch(t, "front", "octagon_search_image", {{"0", 0.9, 0.0}}), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->commanded_pos[2], -0.3);
}

TEST_F(MissionTest, DownFilterFailsOnMissingClassEntry) {
    auto t = tree(R"(<DownForwardSweepAlign task="bin" class_id="{missing_class}" update_msec="10000"/>)");
    EXPECT_EQ(run(t, false, 50ms), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(std::isnan(node->commanded_pos[0]));
}

TEST_F(MissionTest, DownForwardSweepHoldsWhenItGivesUp) {
    auto t = tree(R"(<DownForwardSweepAlign task="bin" forward_step="1.0" update_msec="10" timeout_msec="100"/>)");
    EXPECT_EQ(run(t, false, 1s), BT::NodeStatus::FAILURE);
    // HOLD releases the x/y targets the sweep left running ahead.
    EXPECT_TRUE(std::isnan(node->commanded_pos[0]));
    EXPECT_TRUE(std::isnan(node->commanded_pos[1]));
}

TEST_F(MissionTest, ForwardContinuousAlignCountsFramesInARow) {
    auto t = tree(R"(<ForwardContinuousAlign task="gate" class_id="8" forward_velocity="0.5" confirmation_frames="2"
                                              timeout_msec="5000"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    frame("front", "gate", {{"8", 0.9, 0.0}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    frame("front", "gate", {{"3", 0.9, 0.0}});  // a frame without it
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    frame("front", "gate", {{"8", 0.9, 0.0}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
    frame("front", "gate", {{"8", 0.9, 0.0}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.5);
}

TEST_F(MissionTest, SweepAngleAveragesYawsFromTheHeadingEachWasSeenOn) {
    auto t = tree(R"(<SweepAngle task="slalom" class_id="1" attempts="2" yaw="{@w}"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // step 0: yaw target 0
    arrive();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // sampling
    node->measured_att[2] = 0.05;
    frame("front", "slalom", {{"1", 0.9, 0.25}});  // object at yaw -0.2
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    node->measured_att[2] = 0.0;
    frame("front", "slalom", {{"1", 0.9, 0.2}});  // still -0.2
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_NEAR(t.rootBlackboard()->get<double>("w"), -0.2, 1e-6);
}

TEST_F(MissionTest, DownSweepAlignStopsWhereItFindsIt) {
    auto t = tree(R"(<DownSweepAlign task="bin" lateral_step="1.0" sample_timeout_msec="20"/>)");
    EXPECT_EQ(run(t, false, 100ms), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->commanded_pos[1], -1.0);
    EXPECT_EQ(watch(t, "down", "bin", {{"0", 0.9, 0.0}}, 1s, false), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(std::isnan(node->commanded_pos[1]));
}

TEST_F(MissionTest, DownForwardAlignOffsetAlongTheActualHeading) {
    node->measured_att[2] = M_PI_2;  // facing odom +y, no yaw target
    auto t = tree(R"(<DownForwardAlign task="bin" forward_step="1.0" update_msec="1" timeout_msec="5000"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    // Image up (negative vertical) is forward: 1 m ahead plus tan(0.2) more.
    frame("down", "bin", {{"0", 0.9, 0.0, -0.2}});
    std::this_thread::sleep_for(5ms);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_pos[0], 0.0, 1e-6);
    EXPECT_NEAR(node->commanded_pos[1], 1.0 + std::tan(0.2), 1e-6);
}

TEST_F(MissionTest, DownForwardAlignCentersOnAFrameBetweenCorrections) {
    auto t = tree(R"(<DownForwardAlign task="bin" forward_step="1.0" update_msec="300" timeout_msec="5000"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // start
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // first correction: 1 m ahead
    EXPECT_NEAR(node->commanded_pos[0], 1.0, 1e-6);
    EXPECT_NEAR(node->commanded_pos[1], 0.0, 1e-6);
    frame("down", "bin", {{"0", 0.9, 0.2, 0.0}});      // right of center
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // too soon to correct
    std::this_thread::sleep_for(350ms);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_pos[0], 1.0, 1e-6);
    EXPECT_NEAR(node->commanded_pos[1], -std::tan(0.2), 1e-6);
}

TEST_F(MissionTest, FrontNodesRejectTheDownCamera) {
    auto t = tree(R"(<ForwardContinuousAlign camera="down" task="path"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(MissionTest, VelocityAlignmentStopsOnItsTargetWhileFramesAreRejected) {
    auto t = tree(R"(<LateralAlignToDetection task="slalom" class_id="2" target_distance="2.0"
                                              use_detection_distance="true" align_depth="false" tolerance="0.05"
                                              max_bearing_jump_rad="0.2" use_lateral_velocity_alignment="true"
                                              lateral_alignment_velocity="0.10" blind_alignment_frames="1"
                                              settle_frames="1" min_msec="0" update_msec="1" timeout_msec="5000"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    frame("front", "slalom", {{"2", 0.9, 0.2, 0.0, 1.0}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[1], -0.10);  // target on the right
    // The vehicle reaches the ray's point while every frame is a rejected jump.
    node->measured_pos[1] = -std::tan(0.2);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 5 && status == BT::NodeStatus::RUNNING; ++i) {
        frame("front", "slalom", {{"2", 0.9, -0.6, 0.0, 1.0}});
        status = t.tickOnce();
    }
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[1], 0.0);
}

TEST_F(MissionTest, UnknownCameraFailsAtStart) {
    auto t = tree(R"(<WaitForDetection camera="Front" task="gate" start_timeout_after_first_frame="true"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(MissionTest, YawAlignmentRefusesTheDownCamera) {
    auto t = tree(R"(<AlignToDetection camera="down" task="bin"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
    auto s = tree(R"(<SweepCheck camera="down" task="bin"/>)");
    EXPECT_EQ(s.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(MissionTest, NonFiniteBearingsNeverMatch) {
    auto t = tree(R"(<WaitForDetection task="gate" timeout_msec="300"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    frame("front", "gate", {{"8", 0.9, std::numeric_limits<double>::quiet_NaN()}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    frame("front", "gate", {{"8", 0.9, 0.1}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(MissionTest, MissingFilterEntryFails) {
    auto t = tree(R"(<WaitForDetection task="gate" class_id="{missing_class}" timeout_msec="300"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(MissionTest, SweepCheckAveragesTheYawEachFrameWasSeenFrom) {
    auto t = tree(R"(<SweepCheck task="gate" attempts="2"/>)");
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // yaw step 0
    arrive();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // arrived: sampling
    node->measured_att[2] = 0.05;                      // still settling
    frame("front", "gate", {{"0", 0.9, 0.10}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    node->measured_att[2] = 0.0;
    frame("front", "gate", {{"0", 0.9, 0.05}});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_NEAR(node->commanded_att[2], -0.05, 1e-6);
}

TEST_F(MissionTest, InvalidSettingsFailAtStart) {
    for (const char *body : {
             R"(<AlignToDetection task="gate" depth="true" max_depth_step="-0.5"/>)",
             R"(<LoadModel task="gate" timeout_msec="0"/>)",
             R"(<OrientToDetectionAtDist task="torp_find" tolerance="0"/>)",
             R"(<OrientToDetectionAtDist camera="down" task="torp_find"/>)",
             R"(<DownContinuousAlign task="bin" center_gain="0"/>)",
             R"(<DownContinuousAlign task="bin" desired_distance="0"/>)",
             R"(<DownForwardAlign task="bin" center_gain="-1"/>)",
             R"(<DownForwardSweepAlign task="bin" timeout_msec="0"/>)",
             R"(<SweepAngle task="slalom" sample_timeout_msec="nan" yaw="{@w}"/>)",
             R"(<SweepAngle camera="down" task="slalom" yaw="{@w}"/>)",
             R"(<SweepAngle camera="side" task="slalom" yaw="{@w}"/>)",
             R"(<DownPatternScan task="bin" class_id="{nope}"/>)",
         }) {
        factory.clearRegisteredBehaviorTrees();
        auto t = tree(body);
        EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE) << body;
    }
}

TEST_F(MissionTest, PackagedTreeSettingsStart) {
    // The settings the packaged trees give these nodes (with sample values for
    // those read from the blackboard), and a DownForwardAlign that only steps
    // forward, without centering: each must start.
    for (const char *body : {
             R"(<DownForwardSweepAlign task="torp_fire_blood" class_id="1" min_score="0.35" max_dist="20.0"
                                       forward_step="0.5" timeout_msec="15000"/>)",
             R"(<DownPatternScan task="torp_fire_blood" class_id="1" min_score="0.35" movement_dist="1.65"/>)",
             R"(<DownContinuousAlign task="torp_fire_blood" class_id="1" min_score="0.45" tolerance="0.35"
                                     orient="false" min_msec="2500" timeout_msec="30000"/>)",
             R"(<ForwardContinuousAlign task="octagon_table" min_score="0.5" max_dist="20.0" forward_velocity="1.0"
                                        close_distance="1.5" continue_on_loss="false"/>)",
             R"(<ForwardContinuousAlign task="torp_find" min_score="0.2" max_dist="4.0" forward_velocity="0.6"
                                        close_distance="2.4" continue_on_loss="false" max_yaw_step_deg="35.0"/>)",
             R"(<OrientToDetectionAtDist task="torp_find" class_id="0" min_score="0.1" desired_distance="1.7"
                                         tolerance="0.08" distance_tolerance="0.20" orient_tolerance_deg="5.0"
                                         max_position_step="0.60" require_heading="true" settle_frames="1"
                                         min_msec="500" timeout_msec="40000"/>)",
             R"(<ForwardFixedTransit vision_distance_m="1.0" pass_distance="0.60" min_transit_distance="0.0"
                                     max_planned_distance="1.2" max_dist="6.0" forward_velocity="0.25"
                                     timeout_msec="30000"/>)",
             R"(<ForwardFixedTransit vision_distance_m="2.5" pass_distance="0.80" min_transit_distance="1.0"
                                     max_planned_distance="3.90" max_dist="4.25" forward_velocity="0.25"
                                     timeout_msec="35000"/>)",
             R"(<DownForwardAlign task="bin" center_gain="0"/>)",
         }) {
        factory.clearRegisteredBehaviorTrees();
        auto t = tree(body);
        EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING) << body;
        t.haltTree();
    }
}

class ExecuteTest : public ::testing::Test {
   protected:
    // A mission file holding just `body`, loaded the way `mission` loads one.
    void load(const std::string &body) {
        const auto path = std::filesystem::path(::testing::TempDir()) / "execute_test_mission.xml";
        std::ofstream(path) << R"(<root BTCPP_format="4" main_tree_to_execute="Run"><BehaviorTree ID="Run">)" << body
                            << "</BehaviorTree></root>";
        node = std::make_shared<MissionNode>();
        node->mission = path.string();
        ASSERT_TRUE(node->load_mission());
        node->killed = false;
    }

    std::shared_ptr<MissionNode> node;
};

TEST_F(ExecuteTest, KillEndsTheRun) {
    load(R"(<Sleep msec="20000"/>)");
    const auto started = std::chrono::steady_clock::now();
    std::thread execution([this] { node->execute(); });
    std::this_thread::sleep_for(300ms);
    node->killed = true;
    execution.join();
    EXPECT_LT(std::chrono::steady_clock::now() - started, 3s);
}

TEST_F(ExecuteTest, KillShorterThanATickEndsTheRun) {
    // sub_control re-zeroes the pose at the release, however soon it comes:
    // here the kill and the release both arrive before either is taken.
    load(R"(<Sleep msec="20000"/>)");
    const auto kill_switch =
        node->create_publisher<std_msgs::msg::Bool>("kill_switch", rclcpp::QoS(1).transient_local());
    const auto matched = std::chrono::steady_clock::now() + 2s;
    while (kill_switch->get_subscription_count() == 0 && std::chrono::steady_clock::now() < matched) {
        std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(kill_switch->get_subscription_count(), 0u);

    const auto started = std::chrono::steady_clock::now();
    std::atomic<bool> ended{false};
    std::thread execution([this, &ended] {
        node->execute();
        ended = true;
    });
    std_msgs::msg::Bool kill;
    kill.data = true;
    kill_switch->publish(kill);
    kill.data = false;
    kill_switch->publish(kill);
    std::this_thread::sleep_for(300ms);
    while (!ended && std::chrono::steady_clock::now() - started < 3s) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(5ms);
    }
    EXPECT_TRUE(ended);
    node->interrupt();
    execution.join();
}

TEST_F(ExecuteTest, FinishedRunHoldsStill) {
    load(R"(<VelocitySetpoint x="0.25"/>)");
    node->execute();
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
}

TEST_F(ExecuteTest, InterruptedRunHoldsStill) {
    // SIGINT or SIGTERM mid-run: the velocity the tree started must not carry on.
    load(R"(<Sequence><VelocitySetpoint x="0.25"/><Sleep msec="20000"/></Sequence>)");
    const auto started = std::chrono::steady_clock::now();
    std::thread execution([this] { node->execute(); });
    std::this_thread::sleep_for(300ms);
    node->interrupt();
    execution.join();
    EXPECT_LT(std::chrono::steady_clock::now() - started, 3s);
    EXPECT_DOUBLE_EQ(node->last_velocity_setpoint[0], 0.0);
}

}  // namespace
