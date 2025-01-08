#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"


class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: TAKEOFF");
        
        finished_bases = *blackboard.get<bool>("finished_bases");
        float takeoff_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");

        pos = drone->getLocalPosition();
        goal = Eigen::Vector3d({pos[0], pos[1], takeoff_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.10){
            if (finished_bases)
                return "FINISHED BASES";
            else
                return "NEXT BASE";
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
    bool finished_bases;
};