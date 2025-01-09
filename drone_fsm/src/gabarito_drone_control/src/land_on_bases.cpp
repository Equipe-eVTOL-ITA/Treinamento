#include "initial_takeoff_state.hpp"
#include "landing_state.hpp"
#include "return_home_state.hpp"
#include "takeoff_state.hpp"
#include "visit_bases_state.hpp"
#include "Base.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>


class LandOnBasesFSM : public fsm::FSM {
public:
    LandOnBasesFSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        // 1.1 Instancie a Classe Drone
        this->blackboard_set<Drone>("drone", new Drone());
        Drone* drone = blackboard_get<Drone>("drone");

        // SETANDO A HOME POSITION
        const Eigen::Vector3d fictual_home = Eigen::Vector3d({1.2, -1.0, -0.6});
        drone->setHomePosition(fictual_home);
        const Eigen::Vector3d home_pos = drone->getLocalPosition(); 
        const Eigen::Vector3d orientation = drone->getOrientation();

        // BASES DA ARENA
        std::vector<Base> bases;
        bases.push_back({home_pos, true});
        bases.push_back({Eigen::Vector3d({1.0, -4.0, -1.505})}); //suspended landing platform 1
        bases.push_back({Eigen::Vector3d({2.0, -7.0, -0.005})}); //suspended landing platform 2
        bases.push_back({Eigen::Vector3d({4.0, -5.0, -0.005})}); //landing platform 1
        bases.push_back({Eigen::Vector3d({6.0, -3.0, -0.005})}); //landing platform 2
        bases.push_back({Eigen::Vector3d({7.0, -1.0, -1.005})}); //landing platform 3

        // 1.2 Publique variaveis no Blackboard
        this->blackboard_set<std::vector<Base>>("bases", bases);
        this->blackboard_set<Eigen::Vector3d>("home_position", home_pos);
        this->blackboard_set<bool>("finished_bases", false);
        this->blackboard_set<float>("initial_yaw", orientation[2]);
        this->blackboard_set<float>("takeoff_height", -3.0);

        // 1.3 Adicione os Estados
        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("VISIT BASE", std::make_unique<VisitBasesState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        //1.4 Adicione as Transições

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "VISIT BASE"},{"SEG FAULT", "ERROR"}});

        // Visit Base transitions
        this->add_transitions("VISIT BASE", {{"ARRIVED AT BASE", "LANDING"},{"SEG FAULT", "ERROR"}});

        // Landing transitions
        this->add_transitions("LANDING", {{"LANDED", "TAKEOFF"},{"SEG FAULT", "ERROR"}});

        // Takeoff transitions
        this->add_transitions("TAKEOFF",{
                                {"NEXT BASE", "VISIT BASE"},
                                {"FINISHED BASES", "RETURN HOME"},
                                {"SEG FAULT", "ERROR"}});

        // Return Home transitions
        this->add_transitions("RETURN HOME", {{"AT HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fsm_node"), my_fsm() {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Run at approximately 20 Hz
            std::bind(&NodeFSM::executeFSM, this));
    }

    void executeFSM() {
        if (rclcpp::ok() && !my_fsm.is_finished()) {
            my_fsm.execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    LandOnBasesFSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}



