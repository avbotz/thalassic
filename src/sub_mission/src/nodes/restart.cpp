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

int main(int argc, char** argv)
{
    // Init node
    RCLCPP_INFO(rclcpp::get_logger("restart"), "Init node");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionNode>();

    service_client::init_clients();

    while (rclcpp::ok())
    {
        // Wait for kill switch
        while (rclcpp::ok() && !service_client::alive())
        {
            RCLCPP_INFO(rclcpp::get_logger("restart"), "Sub is not alive yet, waiting...");
            std::this_thread::sleep_for(0.5s);
        }

        RCLCPP_INFO(rclcpp::get_logger("restart"), "Sub is alive. Activate mission.");

        // Creates two child processes: child process executes mission, parent process can kill it
        pid_t pid = fork();

        // Child process runs the mission node
        if (pid == 0)
        {
            // sub is alive, activate
            setsid();

            std::string command = std::string("ros2 run sub_mission mission --ros-args -p 'mission:=") +
                node->mission + std::string("' -p 'role:=") + node->role + std::string("'");

            system(command.c_str());
        }

        // Parent process can end mission node when kill switch is flipped
        else
        {
            while (rclcpp::ok())
            {
                if (!service_client::alive())
                {
                    // sub is no longer alive, deactivate
                    RCLCPP_INFO(rclcpp::get_logger("restart"), "Sub is not alive. Deactivate mission.");
                    kill(pid*-1, SIGINT);
                    break;
                }

                std::this_thread::sleep_for(0.5s);
            }
        }
    }

    return 0;
}
