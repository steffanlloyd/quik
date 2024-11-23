#include "rclcpp/rclcpp.hpp"
#include "quik/srv/ik_service.hpp"
#include "quik/srv/fk_service.hpp"
#include "quik/srv/jacobian_service.hpp"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/ros_helpers.hpp"

using namespace Eigen;
using namespace std;

class KinematicsServiceNode : public rclcpp::Node
{
public:
    KinematicsServiceNode() : Node("kinematics_service")
    {
        // Parse robot parameters, build robot and assign it to this->R
        this->R = std::make_shared<quik::Robot<Dynamic>>(quik::ros_helpers::robotFromNodeParameters(*this));
        RCLCPP_INFO(this->get_logger(), "Loaded robot successfully. Robot configuration is:");
        this->R->print();

        // Build IKSolver and declare parameters
        this->IKS = std::make_shared<quik::IKSolver<Dynamic>>(quik::ros_helpers::IKSolverFromNodeParameters(*this, this->R));
          RCLCPP_INFO(this->get_logger(), "Built IKSolver object. Configuration is:");
        this->IKS->printOptions();

        // Declare services
        // Init inverse kinematics service
        this->ik_srv_ = this->create_service<quik::srv::IKService>(
            "ik_service",
            std::bind(&quik::ros_helpers::ik_service_handler_, 
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            this->IKS));
        RCLCPP_INFO(this->get_logger(), "Inverse kinematics service initialized on /ik_service");

        // Init forward kinematics service
        this->fk_srv_ = this->create_service<quik::srv::FKService>(
            "fk_service",
            std::bind(&quik::ros_helpers::fk_service_handler_, 
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            this->R));
        RCLCPP_INFO(this->get_logger(), "Forward kinematics service initialized on /fk_service");

        // Init jacobian service
        this->jacobian_srv_ = this->create_service<quik::srv::JacobianService>(
            "jacobian_service",
            std::bind(&quik::ros_helpers::jacobian_service_handler_,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            this->R));
        RCLCPP_INFO(this->get_logger(), "Jacobian service initialized on /jacobian_service");

    }

    std::shared_ptr<quik::IKSolver<Dynamic>> IKS;
    std::shared_ptr<quik::Robot<Dynamic>> R;

private:
    rclcpp::Service<quik::srv::IKService>::SharedPtr ik_srv_;
    rclcpp::Service<quik::srv::FKService>::SharedPtr fk_srv_;
    rclcpp::Service<quik::srv::JacobianService>::SharedPtr jacobian_srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsServiceNode>());
    rclcpp::shutdown();
    return 0;
}