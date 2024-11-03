#pragma once

#include "Eigen/Dense"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/Geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "quik/srv/ik_service.hpp"
#include "quik/srv/fk_service.hpp"
#include "quik/srv/jacobian_service.hpp"
#include <iostream>

using namespace Eigen;

namespace quik{
namespace ROSHelpers{

/**
 * @brief Builds a robot based on node parameters for the given node
 * The following parameters must be defined:
 *  - dh: A DOFx4 array of the DH params in the following order:
 *	        [a_1  alpha_1    d_1   theta_1;
 *	         :       :        :       :    
 *	         an   alpha_n    d_n   theta_n ];
 * - link_types: A DOF vector of link types, defined as an array of strings with JOINT_REVOLUTE or JOINT_PRISMATIC
 * - Qsign: A DOF vector of 1 or -1, depending on the direction of the joint.
 * - Tbase: A 4x4 homogeneous transform representing the transform from the world frame to the base joint of the robot
 * - TtoolA A 4x4 homogeneous transform represeting the transform from the last frame to the tool frame. 
 * 
 * @param node The node to draw parameters from
 * @return quik::Robot<Dynamic> 
 */
quik::Robot<Dynamic> robotFromNodeParameters(rclcpp::Node& node)
{
    // Define some helper functions
    // Function to parse parameter into a Matrix4d
    auto parseMatrix4d_ = [](const std::vector<double>& values) {
        if (values.size() != 16) throw std::runtime_error("Invalid size for 4x4 matrix");
        Matrix4d matrix;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix(i, j) = values[i * 4 + j];
            }
        }
        return matrix;
    };
    // Function to parse joint types
    auto getJointType_ = [](const std::string&jointType){
        if (jointType == "JOINT_REVOLUTE" || jointType == "REVOLUTE") return quik::JOINT_REVOLUTE;
        if (jointType == "JOINT_PRISMATIC" || jointType == "PRISMATIC") return quik::JOINT_PRISMATIC;
        throw std::runtime_error("Invalid joint type: " + jointType);
    };

    // Declare parameters for and build robot
    std::vector<double> dh_param = node.declare_parameter("dh", std::vector<double>{-1.0});
    std::vector<std::string> link_types_str = node.declare_parameter("link_types", std::vector<std::string>{"invalid"});
    std::vector<double> q_sign_double = node.declare_parameter("q_sign", std::vector<double>{-1.0});
    Matrix4d Tbase = parseMatrix4d_(node.declare_parameter("Tbase", std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));
    Matrix4d Ttool = parseMatrix4d_(node.declare_parameter("Ttool", std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));

    // Check if parameters are still at their default (invalid) values
    if (dh_param.size() == 1 && dh_param[0] == -1.0)
        RCLCPP_ERROR(node.get_logger(), "Parameter 'dh' has not been set.");
    if (link_types_str.size() == 1 && link_types_str[0] == "invalid")
        RCLCPP_ERROR(node.get_logger(), "Parameter 'link_types' has not been set.");
    if (q_sign_double.size() == 1 && q_sign_double[0] == -1.0)
        RCLCPP_ERROR(node.get_logger(), "Parameter 'q_sign' has not been set.");
    if (q_sign_double.size() != link_types_str.size() || q_sign_double.size() != dh_param.size()/4)
        RCLCPP_ERROR(node.get_logger(), "DH, q_sign and linktype variables don't have congruent sizing");

    // Get Robot and IKSolver arguments, parse them into Eigen objects
    // DH Parameters
    Matrix<double, Dynamic, 4> DH = Map<Matrix<double, 4, Dynamic>>(dh_param.data(), 4, dh_param.size()/4).transpose();

    // Link types
    std::vector<quik::JOINTTYPE_t> link_types_data;
    for (const auto& lt : link_types_str) link_types_data.push_back(getJointType_(lt)); // Convert from string to JOINTTYPE_t
    Vector<quik::JOINTTYPE_t,Dynamic> link_types = Map<Vector<quik::JOINTTYPE_t,Dynamic>, Unaligned>(link_types_data.data(), link_types_data.size());

    // Q sign
    VectorXd q_sign = Eigen::Map<VectorXd>(q_sign_double.data(), q_sign_double.size());

    // Build robot
    return quik::Robot<Dynamic>(
        DH,
        link_types,
        q_sign,
        Tbase,
        Ttool);
}

/**
 * @brief Parses an IKSolver from node parameters:
 *   * max_iterations [int]: Maximum number of iterations of the
 *       algorithm. Default: 100
 *   * algorithm [ALGORITHM_t]: The algorithm to use
 *       ALGORITHM_QUIK - QuIK
 *       ALGORITHM_NR - Newton-Raphson or Levenberg-Marquardt
 *       ALGORITHM_BFGS - BFGS
 *       Default: ALGORITHM_QUIK.
 *   * exit_tolerance [double]: The exit tolerance on the norm of the
 *       error. Default: 1e-12.
 *   * minimum_step_size [double]: The minimum joint angle step size
 *       (normed) before the solver exits. Default: 1e-14.
 *   * relative_improvement_tolerance [double]: The minimum relative
 *       iteration-to-iteration improvement. If this threshold isn't
 *       met, a counter is incremented. If the threshold isn't met
 *       [max_consecutive_grad_fails] times in a row, then the algorithm exits.
 *       For example, 0.05 represents a minimum of 5// relative
 *       improvement. Default: 0.05.
 *   * max_consecutive_grad_fails [int]: The maximum number of relative
 *       improvement fails before the algorithm exits. Default:
 *       20.
 *   * lambda_squared [double]: The square of the damping factor, lambda.
 *       Only applies to the NR and QuIK methods. If given, these
 *       methods become the DNR (also known as levenberg-marquardt)
 *       or the DQuIK algorithm. Ignored for BFGS algorithm.
 *       Default: 0.
 *   * max_linear_step_size [double]: An upper limit of the error step
 *       in a single step. Ignored for BFGS algorithm. Default: 0.3.
 *   * max_angular_step_size [double]: An upper limit of the error step
 *       in a single step. Ignored for BFGS algorithm. Default: 1.
 *   * armijo_sigma [double]: The sigma value used in armijo's
 *       rule, for line search in the BFGS method. Default: 1e-5
 *   * armijo_beta [double]: The beta value used in armijo's
 *       rule, for line search in the BFGS method. Default: 0.5
 * 
 * @param node The node (passed as reference)
 * @param R The robot object (shared pointer)
 * @return quik::IKSolver<Dynamic> 
 */
quik::IKSolver<Dynamic> IKSolverFromNodeParameters(
    rclcpp::Node& node,
    const std::shared_ptr<quik::Robot<Dynamic>> R)
{
    // Define some helper functions
    // Function to parse algorithm types
    auto getAlgorithm_ = [](const std::string& algorithm){
        if (algorithm == "ALGORITHM_QUIK" || algorithm == "QUIK") return quik::ALGORITHM_QUIK;
        if (algorithm == "ALGORITHM_NR" || algorithm == "NR") return quik::ALGORITHM_NR;
        if (algorithm == "ALGORITHM_BFGS" || algorithm == "BFGS") return quik::ALGORITHM_BFGS;
        throw std::runtime_error("Invalid algorithm type: " + algorithm);
    };

    return quik::IKSolver<Dynamic>(
        R,
        node.declare_parameter("max_iterations", 200),
        getAlgorithm_(node.declare_parameter("algorithm", "ALGORITHM_QUIK")),
        node.declare_parameter("exit_tolerance", 1e-12),
        node.declare_parameter("minimum_step_size", 1e-14),
        node.declare_parameter("relative_improvement_tolerance", 0.05),
        node.declare_parameter("max_consecutive_grad_fails", 10),
        node.declare_parameter("max_gradient_fails", 80),
        node.declare_parameter("lambda_squared", 1e-10),
        node.declare_parameter("max_linear_step_size", 0.34),
        node.declare_parameter("max_angular_step_size", 1.0),
        node.declare_parameter("armijo_sigma", 1e-5),
        node.declare_parameter("armijo_beta", 0.5)
    );

}

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

    // Parse request values
    Vector4d quat(
        request->target_pose.orientation.w,
        request->target_pose.orientation.x,
        request->target_pose.orientation.y,
        request->target_pose.orientation.z);
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
    IKS->IK( quat, d, Q0, Q_star, e_star, iter, breakReason);

    // Convert output values back to response
    for (int i = 0; i < Q_star.size(); ++i) response->q_star.push_back(Q_star[i]);
    for (int i = 0; i < e_star.size(); ++i) response->e_star[i] = e_star[i];
    response->iter = iter;
    response->break_reason = breakReason;
    response->success = breakReason == quik::BREAKREASON_TOLERANCE;
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


    RCLCPP_INFO(LOGGER_FK, "Recieved request on /fk_service");

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
    quik::Geometry::hgt2quatpos(T, quat, d);

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
}


} // End of quik::ROSHelper namespace
} // End of quik namespace