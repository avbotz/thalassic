/*
 * This file creates the node that
 * controls the mission_node. The mission_node
 * is actually the one doing the dirty work, but
 * this code just pokes mission_node along and
 * tells it when to start and stop.
 *
 * The difference between restart_node and mission_node
 * is that if the kill switch is flipped during restart node,
 * restart_node will restart mission_node (turn the mission
 * on again from the beginning) once the sub is turned on again.
 * This is useful for when a competition run is botched,
 * the diver kills the sub, places the sub at the start,
 * and turns the sub back on, so the sub can automatically
 * restart the mission without manually typing to launch mission_node.
 */

#include "sub_mission/nodes/restart.hpp"

#include <sys/wait.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "sub_mission/utils.hpp"

namespace {

using namespace std::chrono_literals;

volatile std::sig_atomic_t shutdown_requested = 0;
pid_t active_child = -1;

void handle_signal(int) { shutdown_requested = 1; }

// Waits for the child to exit and be reaped, and for the rest of its process
// group (the mission process `ros2 run` started) to be gone too.
bool wait_for_group_exit(const pid_t pid, const std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
        const pid_t result = waitpid(pid, nullptr, WNOHANG);
        const bool reaped = result == pid || (result < 0 && errno == ECHILD);
        if (reaped && kill(-pid, 0) < 0 && errno == ESRCH) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        std::this_thread::sleep_for(100ms);
    }
}

void terminate_child(const rclcpp::Logger &logger) {
    const pid_t pid = active_child;
    if (pid <= 0) {
        return;
    }

    RCLCPP_INFO(logger, "Stopping mission process group %d.", pid);
    if (kill(-pid, SIGINT) < 0 && errno != ESRCH) {
        RCLCPP_WARN(logger, "SIGINT to mission process group %d failed: %s", pid, std::strerror(errno));
    }
    if (!wait_for_group_exit(pid, 3s)) {
        RCLCPP_WARN(logger, "Mission process group %d did not exit after SIGINT; sending SIGTERM.", pid);
        if (kill(-pid, SIGTERM) < 0 && errno != ESRCH) {
            RCLCPP_WARN(logger, "SIGTERM to mission process group %d failed: %s", pid, std::strerror(errno));
        }
        if (!wait_for_group_exit(pid, 2s)) {
            RCLCPP_WARN(logger, "Mission process group %d did not exit after SIGTERM; sending SIGKILL.", pid);
            if (kill(-pid, SIGKILL) < 0 && errno != ESRCH) {
                RCLCPP_WARN(logger, "SIGKILL to mission process group %d failed: %s", pid, std::strerror(errno));
            }
            wait_for_group_exit(pid, 1s);
        }
    }
    active_child = -1;
}

void log_exit(const rclcpp::Logger &logger, const pid_t pid, const int status) {
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        RCLCPP_INFO(logger, "Mission process %d exited.", pid);
    } else if (WIFEXITED(status)) {
        RCLCPP_ERROR(logger, "Mission process %d exited with status %d.", pid, WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
        RCLCPP_ERROR(logger, "Mission process %d was killed by signal %d (%s).", pid, WTERMSIG(status),
                     strsignal(WTERMSIG(status)));
    }
}

bool sub_alive(const std::shared_ptr<RestartNode> &node) {
    rclcpp::spin_some(node);
    return node->subAlive();
}

// Whether the kill switch has been engaged, however briefly, since `kills`.
bool killed_since(const std::shared_ptr<RestartNode> &node, const std::uint64_t kills) {
    rclcpp::spin_some(node);
    return node->kills() != kills;
}

}  // namespace

int main(int argc, char **argv) {
    // Init node. SIGINT and SIGTERM are handled here, not by rclcpp, so that
    // the mission process is stopped before this one exits.
    RCLCPP_INFO(rclcpp::get_logger("restart"), "Init node");
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const rclcpp::Logger logger = rclcpp::get_logger("restart");
    std::shared_ptr<RestartNode> node;
    try {
        node = std::make_shared<RestartNode>();
    } catch (const std::exception &error) {
        RCLCPP_FATAL(logger, "%s", error.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
    const std::string mission_path = resolveMissionPath(node->mission);
    std::error_code error;
    if (!std::filesystem::exists(mission_path, error)) {
        RCLCPP_FATAL(logger, "Mission file '%s' does not exist.", mission_path.c_str());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // The mission's command line, built here: between fork() and exec the
    // child may only make async-signal-safe calls.
    std::vector<std::string> arguments = {"ros2", "run", "sub_mission", "mission", "--ros-args"};
    // Named as common_launch.py names `mission` when it runs it directly.
    arguments.insert(arguments.end(), {"-r", "__node:=sub_mission"});
    const std::string node_namespace = node->get_namespace();
    if (!node_namespace.empty() && node_namespace != "/") {
        arguments.insert(arguments.end(), {"-r", "__ns:=" + node_namespace});
    }
    // On the simulation clock if this node is.
    const bool sim_time = node->get_parameter("use_sim_time").as_bool();
    arguments.insert(arguments.end(), {"-p", "mission:=" + node->mission, "-p", "role:=" + node->role, "-p",
                                       "groot_port:=" + std::to_string(node->groot_port), "-p",
                                       std::string("use_sim_time:=") + (sim_time ? "true" : "false")});
    std::vector<char *> child_argv;
    for (std::string &argument : arguments) {
        child_argv.push_back(argument.data());
    }
    child_argv.push_back(nullptr);

    while (rclcpp::ok() && !shutdown_requested) {
        // Wait for kill switch
        if (!sub_alive(node)) {
            RCLCPP_INFO(logger, "Sub is killed, waiting...");
        }
        while (rclcpp::ok() && !shutdown_requested && !sub_alive(node)) {
            std::this_thread::sleep_for(0.5s);
        }
        if (!rclcpp::ok() || shutdown_requested) {
            break;
        }
        // Any kill from here on ends this run, however short: sub_control
        // re-zeroes the pose at every release.
        const std::uint64_t kills = node->kills();

        RCLCPP_INFO(logger, "Sub is alive. Activate mission.");

        // The child process runs the mission; the parent can stop it.
        const pid_t pid = fork();
        if (pid < 0) {
            RCLCPP_ERROR(logger, "Failed to fork mission process: %s", std::strerror(errno));
            std::this_thread::sleep_for(1s);
            continue;
        }

        if (pid == 0) {
            // A process group of its own, which terminate_child() stops
            // whole (`ros2 run` and the mission it starts), and which a
            // Ctrl-C meant for this process does not reach.
            setsid();
            execvp(child_argv[0], child_argv.data());
            // _exit, not exit: this process's atexit handlers and DDS state
            // are the parent's.
            _exit(127);
        }

        // Parent process can end mission node when kill switch is flipped
        active_child = pid;
        while (rclcpp::ok() && !shutdown_requested) {
            int status = 0;
            if (waitpid(pid, &status, WNOHANG) == pid) {
                log_exit(logger, pid, status);
                active_child = -1;
                break;
            }

            if (killed_since(node, kills)) {
                RCLCPP_INFO(logger, "Sub was killed. Deactivate mission.");
                terminate_child(logger);
                break;
            }

            std::this_thread::sleep_for(0.5s);
        }

        if (active_child > 0) {
            terminate_child(logger);
        }

        // Run again only after a kill and release, however the mission ended.
        while (rclcpp::ok() && !shutdown_requested && !killed_since(node, kills)) {
            std::this_thread::sleep_for(0.5s);
        }
    }

    terminate_child(logger);
    rclcpp::shutdown();
    return 0;
}
