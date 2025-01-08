#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"

class VisitBasesState : public fsm::State {
public:
    VisitBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: VISIT BASES");

        // DETERMINAR QUAL BASE EH A PROXIMA
        bases = blackboard.get<std::vector<Base>>("bases");
        for (Base& base : *bases){
            if (!base.is_visited){
                this->base_to_visit = &base;
                break;
            }
        }

        float takeoff_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");

        goal = Eigen::Vector3d(this->base_to_visit->coordinates.x(), this->base_to_visit->coordinates.y(), takeoff_height);

        drone->log("Visiting base at: " + std::to_string(goal.x()) + " " + std::to_string(goal.y())); 
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        Eigen::Vector3d pos  = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.10)
            return "ARRIVED AT BASE";

        Eigen::Vector3d distance = goal - pos;
        if (distance.norm() > max_velocity){
            distance = distance.normalized() * max_velocity;
        }
        Eigen::Vector3d little_goal = distance + pos;
        
        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {

        blackboard.set<float>("landing_height", this->base_to_visit->coordinates.z());
        this->base_to_visit->is_visited = true;

        // DETERMINAR SE JA VISITOU TODAS AS BASES
        bool finished_bases = true;
        for (Base& base : *bases){
            if (!base.is_visited){
                finished_bases = false;
                break;
            }
        }
        blackboard.set<bool>("finished_bases", finished_bases);
    }

private:
    Drone* drone;
    std::vector<Base>* bases;
    Base* base_to_visit;
    Eigen::Vector3d goal;
    const float max_velocity = 1.0;
    float initial_yaw;
};