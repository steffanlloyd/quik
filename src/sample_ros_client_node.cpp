/**
 * @file sample_ros_client_node.cpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief The file contains a sample ROS2 node for calling the kinematics service defined in this package.
 * Note, calling kinematics functions in this way is much slower than running them just directly in the
 * code (as shown in sample_ros_cpp_node.cpp), since the ROS service system adds about a millisecond of
 * overhead onto the process. But if this doesn't matter for your application, then definitely use it!
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <Eigen/Dense>
#include "quik/types.hpp"
#include "quik/Robot.hpp"
#include "quik/ros_helpers.hpp"
#include "quik/utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "quik/srv/fk_service.hpp"
#include "quik/srv/ik_service.hpp"
#include "quik/srv/jacobian_service.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
using namespace Eigen;
using namespace quik;
using namespace std;

class SampleClientNode : public rclcpp::Node
{
public:
    SampleClientNode()
    : Node("sample_client_node")
    {
        // Parse robot parameters, build robot and assign it to this->R
        this->R = std::make_shared<Robot<Dynamic>>(ros_helpers::robotFromNodeParameters(*this));
        RCLCPP_INFO(this->get_logger(), "Loaded robot successfully. Robot configuration is:");
        this->R->print();

        // Init service clients and wait until they're available
        this->fk_client_ = this->create_client<quik::srv::FKService>("fk_service");
        while (!this->fk_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "fk_service not available, waiting again...");
        }

        this->ik_client_ = this->create_client<quik::srv::IKService>("ik_service");
        while (!this->ik_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "ik_service not available, waiting again...");
        }

        this->jacobian_client_ = this->create_client<quik::srv::JacobianService>("jacobian_service");
        while (!this->jacobian_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "jacobian_service not available, waiting again...");
        }

        // Run random forward/inverse kinematics problem every 5 seconds
        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&SampleClientNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Set up kinematics loop to run every 5 seconds.");
    }

    std::shared_ptr<Robot<Dynamic>> R;

private:
    void timer_callback()
    {
        // Generate a random pose
        JointState_t<Dynamic> q = Eigen::VectorXd::Random(this->R->dof);
        RCLCPP_INFO(this->get_logger(), "Random joint angles generated:\n%s",
                utilities::eigen2str(q.transpose()).c_str());

        // Call FK service and trigger backwards IK call
        this->fk_task_ = std::async(std::launch::async, [&, q]() {
            // Call FK service
            auto fk_request = ros_helpers::fk_make_request(q);
            auto fk_result = this->fk_client_->async_send_request(fk_request).get();

            // Parse FK result
            Quaternion_t quat; 
            Point3_t d;
            ros_helpers::fk_parse_response(fk_result, quat, d);
            RCLCPP_INFO(this->get_logger(), "Got FK result: \n\tQuaternion: %s\n\tPoint: %s",
                utilities::eigen2str(quat.transpose()).c_str(),
                utilities::eigen2str(d.transpose()).c_str());

            // Perturb initial joint angles
            double perturbation = .1;
            JointState_t<Dynamic> q_perturbed = q + Eigen::VectorXd::Random(this->R->dof) * perturbation;
            RCLCPP_INFO(this->get_logger(), "Joint angles perturbed to:\n%s",
                utilities::eigen2str(q_perturbed.transpose()).c_str());

            // Call IK service
            auto startTime = chrono::high_resolution_clock::now();
            auto ik_request = ros_helpers::ik_make_request(quat, d, q_perturbed);
            auto ik_result = this->ik_client_->async_send_request(ik_request).get();
            chrono::duration<double, std::nano> elapsed = chrono::high_resolution_clock::now() - startTime;
            
            // Parse and print IK results
            JointState_t<Dynamic> q_star(this->R->dof);
            Twist_t e_star; 
            int iter; 
            BreakReason_t breakReason;
            auto success = ros_helpers::ik_parse_response(ik_result, q_star, e_star, iter, breakReason);
            RCLCPP_INFO(this->get_logger(), "IK result: \n%s",
                utilities::eigen2str(q_star.transpose()).c_str());
            RCLCPP_INFO(this->get_logger(), "Normed error: %.4g", e_star.norm());
            RCLCPP_INFO(this->get_logger(), "Took %d iterations, broke because: %s, success %s. Elapsed time: %.2f microseconds.", 
                iter, utilities::breakreason2str(breakReason).c_str(), success ? "true" : "false", elapsed.count()/1e3);
        });

        // Call Jacobian service
        this->jacobian_task_ = std::async(std::launch::async, [&, q]() {
            // Call Jacobian service
            auto jacobian_request = ros_helpers::jacobian_make_request(q);
            auto jacobian_result = this->jacobian_client_->async_send_request(jacobian_request).get();

            // Parse Jacobian results
            Jacobian_t<Dynamic> jacobian(6, this->R->dof);
            ros_helpers::jacobian_parse_response(jacobian_result, jacobian);
            RCLCPP_INFO(this->get_logger(), "Got Jacobian result:\n%s\n",
                utilities::eigen2str(jacobian).c_str());
        });
    }

    rclcpp::Client<quik::srv::FKService>::SharedPtr fk_client_;
    rclcpp::Client<quik::srv::IKService>::SharedPtr ik_client_;
    rclcpp::Client<quik::srv::JacobianService>::SharedPtr jacobian_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::future<void> fk_task_;
    std::future<void> jacobian_task_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SampleClientNode>());
    rclcpp::shutdown();
    return 0;
}