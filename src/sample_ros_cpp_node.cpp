/**
 * @file sample_ros_cpp_node.cpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief This file contains sample code for using the quik kinematics package
 * directly in your C++ code (e.g. without using the service client). This will be 
 * much faster, and for most robots on a typical PC you'd be looking about 1-3 microseconds
 * for inverse kinematics or jacobian calls, and 10-30 microseconds for an inverse kinematics call.
 * The robot and inverse kinematics solver in these examples are built using a helper function
 * that loads the parameters from a yaml file that must be specified at runtime. However, these
 * objects can also be built directly: see example in sample_cpp_usage.cpp.
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "rclcpp/rclcpp.hpp"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/ros_helpers.hpp"
#include "quik/utilities.hpp"
#include <chrono>

// Define DOF at compile time for more speed and no dynamic memory allocation
constexpr int DOF=6;

using namespace Eigen;
using namespace std;

class SampleCPPUsageNode : public rclcpp::Node
{
public:
    SampleCPPUsageNode() : Node("sample_cpp_usage_node")
    {
        // Parse robot parameters, build robot and assign it to this->R
        // Note that robotFromNodeParameters is templated with DOF=6 to result in a fixed-size robot.
        // The YAML file must now contain a DOF=6 robot, otherwise an error will be triggered.
        this->R = std::make_shared<quik::Robot<DOF>>(quik::ros_helpers::robotFromNodeParameters<DOF>(*this));
        RCLCPP_INFO(this->get_logger(), "Loaded robot successfully. Robot configuration is:");
        this->R->print();

        // Build IKSolver and declare parameters
        // IKSolver in this case must also be templated with DOF=6.
        this->IKS = std::make_shared<quik::IKSolver<DOF>>(quik::ros_helpers::IKSolverFromNodeParameters<DOF>(*this, this->R));
          RCLCPP_INFO(this->get_logger(), "Built IKSolver object. Configuration is:");
        this->IKS->printOptions();

        // Run random forward/inverse kinematics problem every 5 seconds
        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&SampleCPPUsageNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Set up kinematics loop to run every 5 seconds.");
    }

    std::shared_ptr<quik::IKSolver<DOF>> IKS;
    std::shared_ptr<quik::Robot<DOF>> R;

private:
    void timer_callback()
    {
        // Generate a random pose
        Vector<double,DOF> q = VectorXd::Random(this->R->dof);
        RCLCPP_INFO(this->get_logger(), "Random joint angles generated:\n%s",
                quik::utilities::eigen2str(q.transpose()).c_str());

        // Run forward kinematics
        Vector4d quat;
        Vector3d d; 
        this->R->FKn(q, quat, d);
        RCLCPP_INFO(this->get_logger(), "Got FK result: \n\tQuaternion: %s\n\tPoint: %s",
            quik::utilities::eigen2str(quat.transpose()).c_str(),
            quik::utilities::eigen2str(d.transpose()).c_str());

        // Run jacobian
        Matrix<double, 6, DOF> J(6, R->dof);
        this->R->jacobian(q, J);
            RCLCPP_INFO(this->get_logger(), "Got Jacobian result:\n%s\n",
                quik::utilities::eigen2str(J).c_str());

        // Perturb initial joint angles
        double perturbation = .1;
        Vector<double,DOF> q_perturbed = q + Vector<double,DOF>::Random(this->R->dof) * perturbation;
        RCLCPP_INFO(this->get_logger(), "Joint angles perturbed to:\n%s",
            quik::utilities::eigen2str(q_perturbed.transpose()).c_str());

        // Run inverse kinematics
        Vector<double,DOF> q_star(IKS->R->dof);
        Vector<double,6> e_star;
        int iter;
        quik::BREAKREASON_t breakReason;
    	auto startTime = chrono::high_resolution_clock::now();
        this->IKS->IK( quat, d, q_perturbed, q_star, e_star, iter, breakReason);
        bool success = breakReason == quik::BREAKREASON_TOLERANCE;
        chrono::duration<double, std::nano> elapsed = chrono::high_resolution_clock::now() - startTime;

        // Parse and print IK results
        RCLCPP_INFO(this->get_logger(), "IK result: \n%s",
            quik::utilities::eigen2str(q_star.transpose()).c_str());
        RCLCPP_INFO(this->get_logger(), "Normed error: %.4g", e_star.norm());
        RCLCPP_INFO(this->get_logger(), "Took %d iterations, broke because: %s, success %s. Elapsed time: %.2f microseconds.", 
            iter, quik::breakreason2str(breakReason).c_str(), success ? "true" : "false", elapsed.count()/1e3);
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SampleCPPUsageNode>());
    rclcpp::shutdown();
    return 0;
}