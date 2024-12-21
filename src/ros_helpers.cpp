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
#include "quik/types.hpp"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "quik/srv/ik_service.hpp"
#include "quik/srv/fk_service.hpp"
#include "quik/srv/jacobian_service.hpp"

using namespace Eigen;
using namespace quik;

namespace quik{
namespace ros_helpers{


/**
 * @brief Makes a forward kinematic service call to the client and returns the future
 * 
 * @param q The desired robot joint angles
 * @return std::shared_ptr<quik::srv::FKService::Request> 
 */
std::shared_ptr<quik::srv::FKService::Request> fk_make_request(const JointState_t<Dynamic>& q)
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
    Quaternion_t& quat, Point3_t& d)
{
    quat << response->pose.orientation.x, response->pose.orientation.y, response->pose.orientation.z, response->pose.orientation.w;
    d << response->pose.position.x, response->pose.position.y, response->pose.position.z;
}

/**
 * @brief Builds a jacobian request and returns the future for it.
 * 
 * @param q The robot joint variables (as an JointState_t<Dynamic>)
 * @return std::shared_ptr<quik::srv::JacobianService::Request>
 */
std::shared_ptr<quik::srv::JacobianService::Request> jacobian_make_request(const JointState_t<Dynamic>& q)
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
 * @param[out] Jacobian_t<DOF> The Jacobian matrix. Must be 6xDOF
 */
void jacobian_parse_response(const quik::srv::JacobianService::Response::SharedPtr& response, Jacobian_t<Dynamic>& jacobian)
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
    const Quaternion_t& quat_des,
    const Point3_t& d_des,
    const JointState_t<Dynamic>& q_0)
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
    JointState_t<Dynamic>& q_star,
    Twist_t& e_star,
    int& iter,
    BreakReason_t& breakReason)
{
    q_star = Eigen::VectorXd::Map(response->q_star.data(), response->q_star.size());
    e_star = Eigen::VectorXd::Map(response->e_star.data(), response->e_star.size());
    iter = response->iter;
    breakReason = static_cast<BreakReason_t>(response->break_reason);
    return response->success;
}

} // End of quik::ros_helpers namespace
} // End of quik namespace