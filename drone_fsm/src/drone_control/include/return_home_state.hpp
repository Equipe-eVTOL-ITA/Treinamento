#include <Eigen/Eigen>
#include <chrono>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: RETURN HOME");

        home_pos = *blackboard.get<Eigen::Vector3d>("home_position");
        pos = drone->getLocalPosition();
        initial_yaw = *blackboard.get<float>("initial_yaw");

        home_horizon = Eigen::Vector3d({home_pos.x(), home_pos.y(), pos.z()});

        start_time = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        pos = drone->getLocalPosition();

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed_time > 15) {
            return "AT HOME";
        }

        if (!over_base){
            if ((pos - home_horizon).norm() < 0.10){
                over_base = true;
                drone->log("Hovering over Home, now descending.");
                return "";
            }

            distance = home_horizon - pos;
            if (distance.norm() > max_velocity){
                distance = distance.normalized() * max_velocity;
            }
            little_goal = distance + pos;
        }
        else
        {
            if ((pos - home_pos).norm() < 0.10){
                return "AT HOME";
            }

            distance = home_pos - pos;
            if (distance.norm() > max_velocity){
                distance = distance.normalized() * max_velocity;
            }
            little_goal = distance + pos;
        }

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        drone->log("At home, now entered Land Mode for precaution.");
        drone->land();
        rclcpp::sleep_for(std::chrono::seconds(5));
        drone->disarmSync();
    }

private:
    Drone* drone;
    bool over_base = false;
    Eigen::Vector3d home_pos, pos, home_horizon, distance, little_goal;
    std::chrono::steady_clock::time_point start_time; 
    float initial_yaw;
    const float max_velocity = 1.0;
};