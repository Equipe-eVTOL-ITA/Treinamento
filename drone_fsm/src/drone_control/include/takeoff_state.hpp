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