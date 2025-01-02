/**
 * @file ros_kinematics_service_node.cpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief This code defines a ROS2 service server for kinematic purposes (forward/inverse kinematics,
 * and jacobian calls). The services are defined on /fk_service, /ik_service, and /jacobian_service
 * respectively. Note that calling kinematics functions through service calls can be convenient,
 * however it will be slower than just using the CPP functions directly since the ROS2 service call
 * system typically adds about a millisecond of overhead onto any single call. But, if this is tolerable
 * for your application, this can be a convenient way of allowing kinematic operations from both python
 * and C++ nodes.
 * 
 * Sample client nodes are available in src/sample_ros_client_node.cpp (c++) and 
 * python/sample_quik_client_node.py (python). 
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "rclcpp/rclcpp.hpp"
#include "quik/srv/ik_service.hpp"
#include "quik/srv/fk_service.hpp"
#include "quik/srv/jacobian_service.hpp"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/ros_helpers.hpp"

using namespace Eigen;
using namespace quik;
using namespace std;

class KinematicsServiceNode : public rclcpp::Node
{
public:
    KinematicsServiceNode() : Node("kinematics_service")
    {
        // Parse robot parameters, build robot and assign it to this->R
        this->R = std::make_shared<Robot<Dynamic>>(ros_helpers::robotFromNodeParameters<Dynamic>(*this));
        RCLCPP_INFO(this->get_logger(), "Loaded robot successfully. Robot configuration is:");
        this->R->print();

        // Build IKSolver and declare parameters
        this->IKS = std::make_shared<IKSolver<Dynamic,6>>(ros_helpers::IKSolverFromNodeParameters<Dynamic>(*this, this->R));
        RCLCPP_INFO(this->get_logger(), "Built IKSolver object. Configuration is:");
        this->IKS->printOptions();

        // Declare services
        // Init inverse kinematics service
        this->ik_srv_ = this->create_service<quik::srv::IKService>(
            "ik_service",
            std::bind(&KinematicsServiceNode::ik_service_handler_, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(this->get_logger(), "Inverse kinematics service initialized on /ik_service");

        // Init forward kinematics service
        this->fk_srv_ = this->create_service<quik::srv::FKService>(
            "fk_service",
            std::bind(&KinematicsServiceNode::fk_service_handler_, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(this->get_logger(), "Forward kinematics service initialized on /fk_service");

        // Init jacobian service
        this->jacobian_srv_ = this->create_service<quik::srv::JacobianService>(
            "jacobian_service",
            std::bind(&KinematicsServiceNode::jacobian_service_handler_, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(this->get_logger(), "Jacobian service initialized on /jacobian_service");

    }

    std::shared_ptr<IKSolver<Dynamic,6>> IKS;
    std::shared_ptr<Robot<Dynamic>> R;

private:
    rclcpp::Service<quik::srv::IKService>::SharedPtr ik_srv_;
    rclcpp::Service<quik::srv::FKService>::SharedPtr fk_srv_;
    rclcpp::Service<quik::srv::JacobianService>::SharedPtr jacobian_srv_;

    /**
     * @brief Handles the inverse kinematics service requests
     * 
     * @param request_header 
     * @param request 
     * @param response
     */
    void ik_service_handler_(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<quik::srv::IKService::Request> request,
        const std::shared_ptr<quik::srv::IKService::Response> response)
    {
        (void)request_header;

        RCLCPP_DEBUG(this->get_logger(), "Received request on /ik_service");

        // Parse request values
        Quaternion_t quat(
            request->target_pose.orientation.x,
            request->target_pose.orientation.y,
            request->target_pose.orientation.z,
            request->target_pose.orientation.w);
        Point3_t d(
            request->target_pose.position.x,
            request->target_pose.position.y,
            request->target_pose.position.z);

        Eigen::Map<JointState_t<Dynamic>> Q0(request->q_0.data(), request->q_0.size());
        if(Q0.size() != IKS->R->dof){
            RCLCPP_WARN(this->get_logger(), "Provided q_0 is the wrong size. Must be size equal to the robot DOF!");
            return;
        }

        // Init output variables
        JointState_t<Dynamic> Q_star(IKS->R->dof);
        Twist_t e_star;
        int iter;
        BreakReason_t breakReason;

        // Use the IK function
        auto startTime = chrono::high_resolution_clock::now();
        this->IKS->IK( quat, d, Q0, Q_star, e_star, iter, breakReason);
        chrono::duration<double, std::nano> elapsed = chrono::high_resolution_clock::now() - startTime;

        // Convert output values back to response
        for (int i = 0; i < Q_star.size(); ++i) response->q_star.push_back(Q_star[i]);
        for (int i = 0; i < e_star.size(); ++i) response->e_star[i] = e_star[i];
        response->iter = iter;
        response->break_reason = breakReason;
        response->success = breakReason == BREAKREASON_TOLERANCE;

        if(response->success){
            RCLCPP_INFO(this->get_logger(), "Successfully processed IK request. Took %d iterations, break reason: %s, normed error is %.4g. Elapsed time: %.2f microseconds.",
                iter, utilities::breakreason2str(breakReason).c_str(), e_star.norm(), elapsed.count()/1e3);
        }else{
            RCLCPP_WARN(this->get_logger(), "Processed IK request. Warning: algorithm did not converge successfully (break reason is %s. Normed error is: %.4g)", 
                utilities::breakreason2str(breakReason).c_str(), e_star.norm());
        }
    }

    /**
     * @brief Handles the forward kinematics service requests
     * 
     * @param request_header 
     * @param request 
     * @param response 
     */
    void fk_service_handler_(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<quik::srv::FKService::Request> request,
        const std::shared_ptr<quik::srv::FKService::Response> response)
    {
        (void)request_header;

        RCLCPP_DEBUG(this->get_logger(), "Received request on /fk_service");

        // Convert the incoming joint angles to an Eigen Vector
        JointState_t<Dynamic> Q = Eigen::VectorXd::Map(request->q.data(), request->q.size());
        int frame = request->frame;

        // Check inputs
        if(Q.size() != this->R->dof){
            RCLCPP_WARN(this->get_logger(), "Provided Q is the wrong size. Must be size equal to the robot DOF!");
            return;
        }
        if( !(frame == -1 || (frame >= 1 && frame <= this->R->dof+1)) ){
            RCLCPP_WARN(this->get_logger(), "Invalid frame size in FK service request. Must be -1 (for tool frame), or 1-DOF.");
            return;
        }
        
        // Create a 4x4 matrix to store the result
        Hgt_t T;
        Quaternion_t quat;
        Point3_t d;

        // Call the FKn function
        this->R->FKn(Q, T, frame);

        // Convert to quaternion and position
        geometry::hgt2quatpos(T, quat, d);

        // Convert the result to a geometry_msgs::Pose message
        response->pose.position.x = d(0);
        response->pose.position.y = d(1);
        response->pose.position.z = d(2);
        response->pose.orientation.x = quat(0);
        response->pose.orientation.y = quat(1);
        response->pose.orientation.z = quat(2);
        response->pose.orientation.w = quat(3);

        RCLCPP_INFO(this->get_logger(), "Request processed successfully on /fk_service");
    }


    /**
     * @brief Handles the Jacobian service requests
     * 
     * @param request_header 
     * @param request 
     * @param response
     */
    void jacobian_service_handler_(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<quik::srv::JacobianService::Request> request,
        const std::shared_ptr<quik::srv::JacobianService::Response> response)
    {
        (void)request_header;

        RCLCPP_DEBUG(this->get_logger(), "Received request on /jacobian_service");

        // Convert the incoming joint angles to an Eigen Vector
        JointState_t<Dynamic> Q = Eigen::VectorXd::Map(request->q.data(), request->q.size());

        // Check inputs
        if(Q.size() != this->R->dof){
            RCLCPP_WARN(this->get_logger(), "Provided Q is the wrong size. Must be size equal to the robot DOF!");
            return;
        }
        
        // Create a 4x4 matrix to store the result
        Jacobian_t<Dynamic> J(6, this->R->dof);

        // Call the jacobian function with the joint angle calling signature
        this->R->jacobian(Q, J);

        // Flatten the Jacobian matrix and assign to the response
        Eigen::VectorXd J_flat = Eigen::Map<Eigen::VectorXd>(J.data(), J.size());
        response->jacobian.assign(J_flat.data(), J_flat.data() + J_flat.size());

        RCLCPP_INFO(this->get_logger(), "Request processed successfully on /jacobian_service");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsServiceNode>());
    rclcpp::shutdown();
    return 0;
}