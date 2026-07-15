#include "actions.hpp"

#include "sub_mission/nodes/mission.hpp"

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include "sub_driver_interfaces/srv/launch_torpedo.hpp"
#include "sub_driver_interfaces/srv/set_dropper.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {

using LaunchTorpedo = sub_driver_interfaces::srv::LaunchTorpedo;
using SetDropper = sub_driver_interfaces::srv::SetDropper;
using Trigger = std_srvs::srv::Trigger;
using SteadyClock = std::chrono::steady_clock;

class ResetStateOnReviveAction : public BT::StatefulActionNode {
   public:
    ResetStateOnReviveAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node,
                             rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          logger_(logger),
          client_(node.create_client<Trigger>("reset_state_on_revive")) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<int>("timeout_msec", 5000, "Maximum wait for control reset response")};
    }

    BT::NodeStatus onStart() override {
        int timeout_msec = 5000;
        getInput("timeout_msec", timeout_msec);
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec);
        sent_request_ = false;
        return tickImpl();
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }
    void onHalted() override { dropPendingRequest(); }

   private:
    BT::NodeStatus tickImpl() {
        if (!sent_request_) {
            if (!client_->service_is_ready()) {
                if (SteadyClock::now() >= deadline_) {
                    RCLCPP_ERROR(logger_, "ResetStateOnRevive timed out waiting for service.");
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::RUNNING;
            }
            auto future_and_id = client_->async_send_request(std::make_shared<Trigger::Request>());
            request_id_ = future_and_id.request_id;
            future_ = future_and_id.future.share();
            sent_request_ = true;
        }
        if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const auto response = future_.get();
            if (!response->success) {
                RCLCPP_ERROR(logger_, "ResetStateOnRevive failed: %s", response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(logger_, "ResetStateOnRevive succeeded: %s", response->message.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_ERROR(logger_, "ResetStateOnRevive timed out waiting for response.");
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void dropPendingRequest() {
        if (sent_request_ && future_.valid() &&
            future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            client_->remove_pending_request(request_id_);
        }
        sent_request_ = false;
    }

    rclcpp::Logger logger_;
    rclcpp::Client<Trigger>::SharedPtr client_;
    std::shared_future<Trigger::Response::SharedPtr> future_;
    std::int64_t request_id_ = 0;
    SteadyClock::time_point deadline_;
    bool sent_request_ = false;
};

class DropBallsAction : public BT::StatefulActionNode {
   public:
    DropBallsAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          logger_(logger),
          client_(node.create_client<SetDropper>("set_dropper")) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<int>("dropper_id", 0, "Dropper id to command"),
                BT::InputPort<bool>("open", true, "Dropper state to command"),
                BT::InputPort<int>("timeout_msec", 5000, "Maximum wait for set_dropper response")};
    }

    BT::NodeStatus onStart() override {
        int dropper_id = 0;
        bool open = true;
        int timeout_msec = 5000;
        getInput("dropper_id", dropper_id);
        getInput("open", open);
        getInput("timeout_msec", timeout_msec);

        if (dropper_id < 0 || dropper_id > 1) {
            RCLCPP_ERROR(logger_, "%s: invalid dropper_id %d.", name().c_str(), dropper_id);
            return BT::NodeStatus::FAILURE;
        }
        dropper_id_ = static_cast<std::uint8_t>(dropper_id);
        open_ = open;
        sent_request_ = false;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec);
        return tickImpl();
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }

    void onHalted() override { dropPendingRequest(); }

   private:
    BT::NodeStatus tickImpl() {
        if (!node_.subAlive()) {
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        if (!sent_request_) {
            if (!client_->service_is_ready()) {
                if (SteadyClock::now() >= deadline_) {
                    RCLCPP_ERROR(logger_, "DropBalls timed out waiting for set_dropper service.");
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::RUNNING;
            }

            auto request = std::make_shared<SetDropper::Request>();
            request->dropper_id = dropper_id_;
            request->open = open_;
            auto future_and_id = client_->async_send_request(request);
            request_id_ = future_and_id.request_id;
            future_ = future_and_id.future.share();
            sent_request_ = true;
            RCLCPP_INFO(logger_, "DropBalls: requested dropper %u %s.", static_cast<unsigned int>(dropper_id_),
                        open_ ? "open" : "closed");
        }
        if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const auto response = future_.get();
            if (!response->success) {
                RCLCPP_ERROR(logger_, "DropBalls failed: %s", response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(logger_, "DropBalls succeeded: %s", response->message.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_ERROR(logger_, "DropBalls timed out waiting for set_dropper response.");
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void dropPendingRequest() {
        if (sent_request_ && future_.valid() &&
            future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            client_->remove_pending_request(request_id_);
        }
        sent_request_ = false;
    }

    MissionNode &node_;
    rclcpp::Logger logger_;
    rclcpp::Client<SetDropper>::SharedPtr client_;
    std::shared_future<SetDropper::Response::SharedPtr> future_;
    std::int64_t request_id_ = 0;
    SteadyClock::time_point deadline_;
    std::uint8_t dropper_id_ = 0;
    bool open_ = true;
    bool sent_request_ = false;
};

class ShootTorpedoAction : public BT::StatefulActionNode {
   public:
    ShootTorpedoAction(const std::string &name, const BT::NodeConfig &config, MissionNode &node, rclcpp::Logger logger)
        : BT::StatefulActionNode(name, config),
          node_(node),
          logger_(logger),
          client_(node.create_client<LaunchTorpedo>("launch_torpedo")) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<int>("torpedo_id", "Torpedo id to command"),
                BT::InputPort<bool>("open", true, "Torpedo thruster state to command"),
                BT::InputPort<int>("timeout_msec", 5000, "Maximum wait for launch_torpedo response")};
    }

    BT::NodeStatus onStart() override {
        int torpedo_id = -1;
        bool open = true;
        int timeout_msec = 5000;
        getInput("torpedo_id", torpedo_id);
        getInput("open", open);
        getInput("timeout_msec", timeout_msec);

        if (torpedo_id < 0 || torpedo_id > 1) {
            RCLCPP_ERROR(logger_, "%s: invalid torpedo_id %d.", name().c_str(), torpedo_id);
            return BT::NodeStatus::FAILURE;
        }
        torpedo_id_ = static_cast<std::uint8_t>(torpedo_id);
        open_ = open;
        sent_request_ = false;
        deadline_ = SteadyClock::now() + std::chrono::milliseconds(timeout_msec);
        return tickImpl();
    }

    BT::NodeStatus onRunning() override { return tickImpl(); }

    void onHalted() override { dropPendingRequest(); }

   private:
    BT::NodeStatus tickImpl() {
        if (!node_.subAlive()) {
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        if (!sent_request_) {
            if (!client_->service_is_ready()) {
                if (SteadyClock::now() >= deadline_) {
                    RCLCPP_ERROR(logger_, "%s timed out waiting for launch_torpedo service.", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::RUNNING;
            }

            auto request = std::make_shared<LaunchTorpedo::Request>();
            request->torpedo_id = torpedo_id_;
            request->open = open_;
            auto future_and_id = client_->async_send_request(request);
            request_id_ = future_and_id.request_id;
            future_ = future_and_id.future.share();
            sent_request_ = true;
            RCLCPP_INFO(logger_, "%s: requested torpedo %u %s.", name().c_str(), torpedo_id_,
                        open_ ? "open" : "closed");
        }
        if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const auto response = future_.get();
            if (!response->success) {
                RCLCPP_ERROR(logger_, "%s failed: %s", name().c_str(), response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(logger_, "%s succeeded: %s", name().c_str(), response->message.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        if (SteadyClock::now() >= deadline_) {
            RCLCPP_ERROR(logger_, "%s timed out waiting for launch_torpedo response.", name().c_str());
            dropPendingRequest();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void dropPendingRequest() {
        if (sent_request_ && future_.valid() &&
            future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            client_->remove_pending_request(request_id_);
        }
        sent_request_ = false;
    }

    MissionNode &node_;
    rclcpp::Logger logger_;
    rclcpp::Client<LaunchTorpedo>::SharedPtr client_;
    std::shared_future<LaunchTorpedo::Response::SharedPtr> future_;
    std::int64_t request_id_ = 0;
    SteadyClock::time_point deadline_;
    std::uint8_t torpedo_id_ = 0;
    bool open_ = true;
    bool sent_request_ = false;
};

}  // namespace

void registerActuatorActions(BT::BehaviorTreeFactory &factory, MissionNode &node, rclcpp::Logger logger) {
    factory.registerBuilder<ResetStateOnReviveAction>(
        "ResetStateOnRevive", [&node, logger](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<ResetStateOnReviveAction>(name, config, node, logger);
        });

    factory.registerBuilder<DropBallsAction>("DropBalls",
                                             [&node, logger](const std::string &name, const BT::NodeConfig &config) {
                                                 return std::make_unique<DropBallsAction>(name, config, node, logger);
                                             });

    factory.registerBuilder<ShootTorpedoAction>("ShootTorpedo",
                                                [&node, logger](const std::string &name,
                                                                const BT::NodeConfig &config) {
                                                    return std::make_unique<ShootTorpedoAction>(name, config, node,
                                                                                                logger);
                                                });
}
