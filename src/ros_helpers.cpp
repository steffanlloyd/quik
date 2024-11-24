/**
 * @file ros_helpers.cpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief Defines several helper functions for using quik in ROS, including:
 * - robotFromNodeParameters: Builds a robot from the node's parameters, defined
 *   in a yaml file.
 * - IKSolverFromNodeParameters: Builds an IKsolver object from the nodes parameters,
 *   defined in a yaml file.
 * - The service handles for ik_service, fk_service, and jacobian_service
 * - Helper functions to make and parse service requests for fk_service,
 *   ik_service, and jacobian_service.
 * 
 * Full documentation provided at the header of each function.
 * 
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "quik/ros_helpers.hpp"
#include "Eigen/Dense"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "quik/srv/ik_service.hpp"
#include "quik/srv/fk_service.hpp"
#include "quik/srv/jacobian_service.hpp"

using namespace Eigen;

namespace quik{
namespace ros_helpers{

/**
 * @brief Handles the inverse kinematics service requests
 * 
 * @param request_header 
 * @param request 
 * @param response 
 * @param IKS The IKSolver object (passed as a shared pointer) to solve the inverse kinematics
 * @param logger The rclcpp logger to log errors to, if applicable.
 *               Defaults to an rclcpp logger with name jacobian_service_handler
 */
auto LOGGER_IK = rclcpp::get_logger("ik_service_handler");
void ik_service_handler_(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<quik::srv::IKService::Request> request,
    const std::shared_ptr<quik::srv::IKService::Response> response,
    const std::shared_ptr<quik::IKSolver<Dynamic>> IKS)
{
    (void)request_header;

    RCLCPP_DEBUG(LOGGER_IK, "Received request on /ik_service");

    // Parse request values
    Vector4d quat(
        request->target_pose.orientation.x,
        request->target_pose.orientation.y,
        request->target_pose.orientation.z,
        request->target_pose.orientation.w);
    Vector3d d(
        request->target_pose.position.x,
        request->target_pose.position.y,
        request->target_pose.position.z);

    Eigen::Map<Eigen::VectorXd> Q0(request->q_0.data(), request->q_0.size());
    if(Q0.size() != IKS->R->dof){
        RCLCPP_WARN(LOGGER_IK, "Provided q_0 is the wrong size. Must be size equal to the robot DOF!");
        return;
    }

    // Init output variables
    VectorXd Q_star(IKS->R->dof);
    Vector<double,6> e_star;
    int iter;
    quik::BREAKREASON_t breakReason;

    // Use the IK function
    auto startTime = chrono::high_resolution_clock::now();
    IKS->IK( quat, d, Q0, Q_star, e_star, iter, breakReason);
    chrono::duration<double, std::nano> elapsed = chrono::high_resolution_clock::now() - startTime;

    // Convert output values back to response
    for (int i = 0; i < Q_star.size(); ++i) response->q_star.push_back(Q_star[i]);
    for (int i = 0; i < e_star.size(); ++i) response->e_star[i] = e_star[i];
    response->iter = iter;
    response->break_reason = breakReason;
    response->success = breakReason == quik::BREAKREASON_TOLERANCE;

    if(response->success){
        RCLCPP_INFO(LOGGER_IK, "Successfully processed IK request. Took %d iterations, break reason: %s, normed error is %.4g. Elapsed time: %.2f microseconds.",
            iter, quik::breakreason2str(breakReason).c_str(), e_star.norm(), elapsed.count()/1e3);
    }else{
        RCLCPP_WARN(LOGGER_IK, "Processed IK request. Warning: algorithm did not converge successfully (break reason is %s. Normed error is: %.4g)", 
            quik::breakreason2str(breakReason).c_str(), e_star.norm());
    }
}

/**
 * @brief Handles the forward kinematics service requests
 * 
 * @param request_header 
 * @param request 
 * @param response 
 * @param R The Robot object (passed as a shared pointer) to solve the forward kinematics
 * @param logger The rclcpp logger to log errors to, if applicable.
 *               Defaults to an rclcpp logger with name jacobian_service_handler
 */
auto LOGGER_FK = rclcpp::get_logger("fk_service_handler");
void fk_service_handler_(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<quik::srv::FKService::Request> request,
    const std::shared_ptr<quik::srv::FKService::Response> response,
    const std::shared_ptr<quik::Robot<Dynamic>> R)
{
    (void)request_header;


    RCLCPP_DEBUG(LOGGER_FK, "Received request on /fk_service");

    // Convert the incoming joint angles to an Eigen Vector
    Eigen::VectorXd Q = Eigen::VectorXd::Map(request->q.data(), request->q.size());
    int frame = request->frame;

    // Check inputs
    if(Q.size() != R->dof){
        RCLCPP_WARN(LOGGER_FK, "Provided Q is the wrong size. Must be size equal to the robot DOF!");
        return;
    }
    if( !(frame == -1 || (frame >= 1 && frame <= R->dof+1)) ){
        RCLCPP_WARN(LOGGER_FK, "Invalid frame size in FK service request. Must be -1 (for tool frame), or 1-DOF.");
        return;
    }
    
    // Create a 4x4 matrix to store the result
    Eigen::Matrix4d T;
    Vector<double, 4> quat;
    Vector<double, 3> d;

    // Call the FKn function
    R->FKn(Q, T, frame);

    // Convert to quaternion and position
    quik::geometry::hgt2quatpos(T, quat, d);

    // Convert the result to a geometry_msgs::Pose message
    response->pose.position.x = d(0);
    response->pose.position.y = d(1);
    response->pose.position.z = d(2);
    response->pose.orientation.x = quat(0);
    response->pose.orientation.y = quat(1);
    response->pose.orientation.z = quat(2);
    response->pose.orientation.w = quat(3);

    RCLCPP_INFO(LOGGER_FK, "Request processed successfully on /fk_service");
}


/**
 * @brief Handles the Jacobian service requests
 * 
 * @param request_header 
 * @param request 
 * @param response 
 * @param IKS The Robot object (passed as a shared pointer) to solve the jacobian kinematics
 * @param logger The rclcpp logger to log errors to, if applicable.
 *               Defaults to an rclcpp logger with name jacobian_service_handler
 */
auto LOGGER_JACOBIAN = rclcpp::get_logger("jacobian_service_handler");
void jacobian_service_handler_(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<quik::srv::JacobianService::Request> request,
    const std::shared_ptr<quik::srv::JacobianService::Response> response,
    const std::shared_ptr<quik::Robot<Dynamic>> R)
{
    (void)request_header;

    RCLCPP_DEBUG(LOGGER_JACOBIAN, "Received request on /jacobian_service");

    // Convert the incoming joint angles to an Eigen Vector
    Eigen::VectorXd Q = Eigen::VectorXd::Map(request->q.data(), request->q.size());

    // Check inputs
    if(Q.size() != R->dof){
        RCLCPP_WARN(LOGGER_JACOBIAN, "Provided Q is the wrong size. Must be size equal to the robot DOF!");
        return;
    }
    
    // Create a 4x4 matrix to store the result
    Eigen::Matrix<double, 6, Dynamic> J(6, R->dof);

    // Call the jacobian function with the joint angle calling signature
    R->jacobian(Q, J);

    // Flatten the Jacobian matrix and assign to the response
    Eigen::VectorXd J_flat = Eigen::Map<Eigen::VectorXd>(J.data(), J.size());
    response->jacobian.assign(J_flat.data(), J_flat.data() + J_flat.size());

    RCLCPP_INFO(LOGGER_JACOBIAN, "Request processed successfully on /jacobian_service");
}


/**
 * @brief Makes a forward kinematic service call to the client and returns the future
 * 
 * @param q The desired robot joint angles
 * @return std::shared_ptr<quik::srv::FKService::Request> 
 */
std::shared_ptr<quik::srv::FKService::Request> fk_make_request(const Eigen::VectorXd& q)
{
    auto request = std::make_shared<quik::srv::FKService::Request>();
    for (int i=0; i<q.size(); ++i) request->q.push_back(q(i));
    return request;
}

/**
 * @brief Parses an FK_service response into two eigen objects
 * 
 * @param[in] response 
 * @param[out] quat The quaternion (x,y,z,w)
 * @param[out] d The point (x,y,z)
 */
void fk_parse_response(const quik::srv::FKService::Response::SharedPtr& response,
    Vector4d& quat, Vector3d& d)
{
    quat << response->pose.orientation.x, response->pose.orientation.y, response->pose.orientation.z, response->pose.orientation.w;
    d << response->pose.position.x, response->pose.position.y, response->pose.position.z;
}

/**
 * @brief Builds a jacobian request and returns the future for it.
 * 
 * @param q The robot joint variables (as an Eigen::VectorXd)
 * @return std::shared_ptr<quik::srv::JacobianService::Request>
 */
std::shared_ptr<quik::srv::JacobianService::Request> jacobian_make_request(const Eigen::VectorXd& q)
{
    auto request = std::make_shared<quik::srv::JacobianService::Request>();
    for (int i=0; i<q.size(); ++i) request->q.push_back(q(i));
    return request;
}

/**
 * @brief Parses the Jacobian service response into an Eigen::MatrixXD matrix
 * (of size 6xDOF).
 * 
 * @param[in] response 
 * @param[out] Eigen::MatrixXd The Jacobian matrix. Must be 6xDOF
 */
void jacobian_parse_response(const quik::srv::JacobianService::Response::SharedPtr& response, Eigen::MatrixXd& jacobian)
{
    int dof = response->jacobian.size() / 6;

    // Check that the input matrix is of the correct size
    if (jacobian.rows() != 6 || jacobian.cols() != dof) {
        throw std::invalid_argument("Input matrix must be of size 6xDOF");
    }

    // Assign result
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < dof; ++j) {
            jacobian(i, j) = response->jacobian[i * dof + j];
        }
    }
}

/**
 * @brief Makes an inverse kinematic service request from Eigen objects, and
 * returns the future for it.
 * 
 * @param quat_des The desired quaternion (x,y,z,w) 
 * @param d_des The desired position (x,y,z)
 * @param q_0 The initial guess of joint angles
 * @return std::shared_ptr<quik::srv::IKService::Request>
 */
std::shared_ptr<quik::srv::IKService::Request> ik_make_request(
    const Eigen::Vector4d& quat_des,
    const Eigen::Vector3d& d_des,
    const Eigen::VectorXd& q_0)
{
    auto request = std::make_shared<quik::srv::IKService::Request>();
    request->target_pose.orientation.x = quat_des(0);
    request->target_pose.orientation.y = quat_des(1);
    request->target_pose.orientation.z = quat_des(2);
    request->target_pose.orientation.w = quat_des(3);
    request->target_pose.position.x = d_des(0);
    request->target_pose.position.y = d_des(1);
    request->target_pose.position.z = d_des(2);
    for (int i=0; i<q_0.size(); ++i) request->q_0.push_back(q_0(i));
    return request;
}

/**
 * @brief Parses the inverse kinematics response into Eigen objects
 * 
 * @param response 
 * @param[out] q_star The found joint angles at the requested pose
 * @param[out] e_star The 6-vector of error (twist) at the found joint angles
 * @param[out] iter The number of iterations the algorithm took
 * @param[out] breakReason The reason the algorithm broke out
 * @return success (true or false)
 */
bool ik_parse_response(const quik::srv::IKService::Response::SharedPtr& response,
    VectorXd& q_star,
    Vector<double,6>& e_star,
    int& iter,
    quik::BREAKREASON_t& breakReason)
{
    q_star = Eigen::VectorXd::Map(response->q_star.data(), response->q_star.size());
    e_star = Eigen::VectorXd::Map(response->e_star.data(), response->e_star.size());
    iter = response->iter;
    breakReason = static_cast<quik::BREAKREASON_t>(response->break_reason);
    return response->success;
}

} // End of quik::ros_helpers namespace
} // End of quik namespace