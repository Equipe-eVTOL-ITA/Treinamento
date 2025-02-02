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

        // TODO: 1.1 Instancie a Classe Drone


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

        // TODO: 1.2 Publique variaveis no Blackboard


        // TODO: 1.3 Adicione os Estados


        //TODO: 1.4 Adicione as Transições

        
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



