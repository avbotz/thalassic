#include "sub_mission/utils.hpp"

#include <exception>

#include "ament_index_cpp/get_package_share_directory.hpp"

const char *statusName(const BT::NodeStatus status) {
    switch (status) {
        case BT::NodeStatus::SUCCESS:
            return "SUCCESS";
        case BT::NodeStatus::FAILURE:
            return "FAILURE";
        case BT::NodeStatus::RUNNING:
            return "RUNNING";
        case BT::NodeStatus::IDLE:
            return "IDLE";
        case BT::NodeStatus::SKIPPED:
            return "SKIPPED";
    }

    return "UNKNOWN";
}

std::string missionTreePath() {
    try {
        return ament_index_cpp::get_package_share_directory("sub_mission") + "/resources/main_mission.xml";
    } catch (const std::exception &) {
        return "resources/main_mission.xml";
    }
}

std::string bool_to_text(const bool value) {
    return value ? "true" : "false";
}
