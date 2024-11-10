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
quik::Robot<Dynamic> robotFromNodeParameters(rclcpp::Node& node);

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
    const std::shared_ptr<quik::Robot<Dynamic>> R);

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
void ik_service_handler_(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<quik::srv::IKService::Request> request,
    const std::shared_ptr<quik::srv::IKService::Response> response,
    const std::shared_ptr<quik::IKSolver<Dynamic>> IKS);

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
void fk_service_handler_(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<quik::srv::FKService::Request> request,
    const std::shared_ptr<quik::srv::FKService::Response> response,
    const std::shared_ptr<quik::Robot<Dynamic>> R);


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
void jacobian_service_handler_(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<quik::srv::JacobianService::Request> request,
    const std::shared_ptr<quik::srv::JacobianService::Response> response,
    const std::shared_ptr<quik::Robot<Dynamic>> R);


} // End of quik::ROSHelper namespace
} // End of quik namespace