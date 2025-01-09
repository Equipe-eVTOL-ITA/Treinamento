#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"


class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: INITIAL TAKEOFF");

        drone->toOffboardSync();
        drone->armSync();
        
        pos = drone->getLocalPosition();
        float takeoff_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");

        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

        goal = Eigen::Vector3d({pos[0], pos[1], takeoff_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.15){
            return "INITIAL TAKEOFF COMPLETED";
        }

        Eigen::Vector3d distance = goal - pos;

        if (distance.norm() > max_velocity){
            distance = distance.normalized() * max_velocity;
        }
        Eigen::Vector3d little_goal = distance + pos;
        
        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);
        
        return "";
    }

private:
    const float max_velocity = 1.0;
    Eigen::Vector3d pos, goal;
    Drone* drone;
    float initial_yaw;
};