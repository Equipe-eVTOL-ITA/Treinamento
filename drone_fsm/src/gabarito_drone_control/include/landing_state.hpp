#include <Eigen/Eigen>
#include <chrono>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: LANDING");

        float landing_height = *blackboard.get<float>("landing_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");

        pos = drone->getLocalPosition();

        goal = Eigen::Vector3d({pos[0], pos[1], landing_height + 0.1});

        start_time = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed_time > 15) {
            return "LANDED";
        }

        pos = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.10){
            return "LANDED";
        }

        Eigen::Vector3d distance = goal - pos;
        if (distance.norm() > max_velocity){
            distance = distance.normalized() * max_velocity;
        }
        Eigen::Vector3d little_goal = distance + pos;
        
        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        // SEND 3 REPETITIVE COMMANDS TO SWITCH TO LAND MODE
        // drone->log("Entering Land Mode to make sure it is landed.");
        // for (int i = 0 ;i < 3; i++){
            // drone->land();
            // rclcpp::sleep_for(std::chrono::milliseconds(500));
        // }
    }

private:
    Drone* drone;
    float initial_yaw;
    Eigen::Vector3d pos, goal;
    std::chrono::steady_clock::time_point start_time; 
    const float max_velocity = 1.0;
};