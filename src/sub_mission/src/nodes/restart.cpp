/*
 * This file creates the node that 
 * controls the mission_node. The mission_node
 * is actually the one doing the dirty work, but
 * this code just pokes mission_node along and
 * tells it when to start and stop.
 * 
 * The difference between restart_node and mission_node
 * is that if the kill switched is flipped during restart node,
 * restart_node will restart mission_node (turn the mission
 * on again from the beginning) once the sub is turned on again.
 * This is useful for when a competition run is botched,
 * the diver kills the sub, places the sub at the start,
 * and turns the sub back on, so the sub can automatically
 * restart the mission without manually typing to launch mission_node.
 */

#include "sub_mission/nodes/restart.hpp"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <sys/wait.h>

namespace
{

using namespace std::chrono_literals;

volatile std::sig_atomic_t shutdown_requested = 0;
pid_t active_child = -1;

void handle_signal(int)
{
    shutdown_requested = 1;
}

bool wait_for_child_exit(const pid_t pid, const std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        int status = 0;
        const pid_t result = waitpid(pid, &status, WNOHANG);
        if (result == pid)
        {
            return true;
        }
        if (result < 0 && errno == ECHILD)
        {
            return true;
        }
        std::this_thread::sleep_for(100ms);
    }
    return false;
}

void terminate_child(const rclcpp::Logger &logger)
{
    const pid_t pid = active_child;
    if (pid <= 0)
    {
        return;
    }

    RCLCPP_INFO(logger, "Stopping mission process group %d.", pid);
    if (kill(-pid, SIGINT) < 0 && errno != ESRCH)
    {
        RCLCPP_WARN(logger, "SIGINT to mission process group %d failed: %s", pid, std::strerror(errno));
    }
    if (!wait_for_child_exit(pid, 3s))
    {
        RCLCPP_WARN(logger, "Mission process group %d did not exit after SIGINT; sending SIGTERM.", pid);
        if (kill(-pid, SIGTERM) < 0 && errno != ESRCH)
        {
            RCLCPP_WARN(logger, "SIGTERM to mission process group %d failed: %s", pid, std::strerror(errno));
        }
    }
    if (!wait_for_child_exit(pid, 2s))
    {
        RCLCPP_WARN(logger, "Mission process group %d did not exit after SIGTERM; sending SIGKILL.", pid);
        if (kill(-pid, SIGKILL) < 0 && errno != ESRCH)
        {
            RCLCPP_WARN(logger, "SIGKILL to mission process group %d failed: %s", pid, std::strerror(errno));
        }
        wait_for_child_exit(pid, 1s);
    }
    active_child = -1;
}

bool sub_alive(const std::shared_ptr<MissionNode> &node)
{
    rclcpp::spin_some(node);
    return node->subAlive();
}

}  // namespace

int main(int argc, char** argv)
{
    // Init node
    RCLCPP_INFO(rclcpp::get_logger("restart"), "Init node");
    rclcpp::init(argc, argv);
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const rclcpp::Logger logger = rclcpp::get_logger("restart");
    auto node = std::make_shared<MissionNode>();

    while (rclcpp::ok() && !shutdown_requested)
    {
        // Wait for kill switch
        while (rclcpp::ok() && !shutdown_requested && !sub_alive(node))
        {
            RCLCPP_INFO(logger, "Sub is killed, waiting...");
            std::this_thread::sleep_for(0.5s);
        }
        if (!rclcpp::ok() || shutdown_requested)
        {
            break;
        }

        RCLCPP_INFO(logger, "Sub is alive. Activate mission.");

        // Creates two child processes: child process executes mission, parent process can kill it
        pid_t pid = fork();
        if (pid < 0)
        {
            RCLCPP_ERROR(logger, "Failed to fork mission process: %s", std::strerror(errno));
            std::this_thread::sleep_for(1s);
            continue;
        }

        // Child process runs the mission node
        if (pid == 0)
        {
            // sub is alive, activate
            setsid();

            std::string mission_param = "mission:=" + node->mission;
            std::string role_param = "role:=" + node->role;
            const std::string node_namespace = node->get_namespace();
            if (!node_namespace.empty() && node_namespace != "/")
            {
                std::string namespace_remap = "__ns:=" + node_namespace;
                execlp("ros2", "ros2", "run", "sub_mission", "mission", "--ros-args", "-r", namespace_remap.c_str(),
                       "-p", mission_param.c_str(), "-p", role_param.c_str(), static_cast<char *>(nullptr));
            }

            execlp("ros2", "ros2", "run", "sub_mission", "mission", "--ros-args", "-p", mission_param.c_str(), "-p",
                   role_param.c_str(), static_cast<char *>(nullptr));
            std::exit(127);
        }

        // Parent process can end mission node when kill switch is flipped
        active_child = pid;
        while (rclcpp::ok() && !shutdown_requested)
        {
            int status = 0;
            const pid_t exited = waitpid(pid, &status, WNOHANG);
            if (exited == pid)
            {
                RCLCPP_INFO(logger, "Mission process %d exited.", pid);
                active_child = -1;
                break;
            }

            if (!sub_alive(node))
            {
                // sub is no longer alive, deactivate
                RCLCPP_INFO(logger, "Sub is not alive. Deactivate mission.");
                terminate_child(logger);
                break;
            }

            std::this_thread::sleep_for(0.5s);
        }

        if (active_child > 0)
        {
            terminate_child(logger);
        }

        while (rclcpp::ok() && !shutdown_requested && sub_alive(node))
        {
            std::this_thread::sleep_for(0.5s);
        }
    }

    terminate_child(logger);
    rclcpp::shutdown();
    return 0;
}
