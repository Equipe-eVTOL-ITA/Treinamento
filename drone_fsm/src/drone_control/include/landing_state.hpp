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

        //TODO: Implementar logica do estado
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        //TODO: Implementar logica do estado

        // if (condicao1)
            // return "EVENT_1";
        // else
            // return "EVENT_2";
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        
        // TODO: Implementar logica do estado
    }

private:
    Drone* drone;
};