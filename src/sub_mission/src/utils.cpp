#include "sub_mission/utils.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <system_error>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace {

std::string resourcesDirectory() {
    try {
        return ament_index_cpp::get_package_share_directory("sub_mission") + "/resources";
    } catch (const std::exception &) {
        return "resources";
    }
}

std::vector<std::string> listXmlFiles(const std::string &directory, const bool stem_only) {
    std::vector<std::string> files;
    std::error_code error;
    for (const auto &entry : std::filesystem::directory_iterator(directory, error)) {
        // Skip dangling symlinks: with --symlink-install, deleting a tree in
        // src leaves a dead link in install/ that would fail every mission.
        if (entry.path().extension() == ".xml" && std::filesystem::exists(entry.path(), error)) {
            files.push_back(stem_only ? entry.path().stem().string() : entry.path().string());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

}  // namespace

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

std::string missionsDirectory() { return resourcesDirectory() + "/missions"; }

std::string treesDirectory() { return resourcesDirectory() + "/trees"; }

std::string resolveMissionPath(const std::string &name_or_path) {
    if (name_or_path.find('/') != std::string::npos || std::filesystem::exists(name_or_path)) {
        return name_or_path;
    }
    std::string file_name = name_or_path;
    const std::string extension = ".xml";
    if (file_name.size() < extension.size() ||
        file_name.compare(file_name.size() - extension.size(), extension.size(), extension) != 0) {
        file_name += extension;
    }
    return missionsDirectory() + "/" + file_name;
}

std::vector<std::string> availableMissions() { return listXmlFiles(missionsDirectory(), true); }

std::vector<std::string> treeFiles() { return listXmlFiles(treesDirectory(), false); }
