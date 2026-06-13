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
#include "sub_mission/utils.hpp"

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

            std::string command = std::string("ros2 run sub_mission mission --ros-args -p ") +
                std::string("POOL_A:=") + bool_to_text(node->POOL_A) + std::string(" -p ") +
                std::string("POOL_B:=") + bool_to_text(node->POOL_B) + std::string(" -p ") +
                std::string("POOL_C:=") + bool_to_text(node->POOL_C) + std::string(" -p ") +
                std::string("POOL_D:=") + bool_to_text(node->POOL_D) + std::string(" -p ") +
                std::string("HEADS:=") + bool_to_text(node->HEADS) + std::string(" -p ") +
                std::string("TAILS:=") + bool_to_text(node->TAILS) + std::string(" -p ") +
                std::string("SIM:=") + bool_to_text(node->SIM) + std::string(" -p ") +
                std::string("COIN_FLIP:=") + bool_to_text(node->COIN_FLIP) + std::string(" -p ") +
                std::string("GATE:=") + bool_to_text(node->GATE) + std::string(" -p ") +
                std::string("BUOY:=") + bool_to_text(node->BUOY) + std::string(" -p ") +
                std::string("BINS:=") + bool_to_text(node->BINS) + std::string(" -p ") +
                std::string("TORP:=") + bool_to_text(node->TORP) + std::string(" -p ") +
                std::string("OCTAGON:=") + bool_to_text(node->OCTAGON) + std::string(" -p ") +
                std::string("PRELIM:=") + bool_to_text(node->PRELIM) + std::string(" -p ") +
                std::string("POOL_TEST:=") + bool_to_text(node->POOL_TEST) + std::string(" -p ") +
                std::string("VISION_TEST:=") + bool_to_text(node->VISION_TEST) + std::string(" -p ") +
                std::string("PID_TUNING_SEQUENCE:=") + bool_to_text(node->PID_TUNING_SEQUENCE);
                
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
