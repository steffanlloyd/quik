#include "rclcpp/rclcpp.hpp"
#include "quik/srv/ik_service.hpp"
#include "quik/srv/fk_service.hpp"
#include "quik/srv/jacobian_service.hpp"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/Geometry.hpp"

using namespace Eigen;
using namespace std;

class KinematicsServiceNode : public rclcpp::Node
{
public:
    KinematicsServiceNode() : Node("kinematics_service")
    {
        // Parse robot parameters, build robot and assign it to this->R
        this->R = std::make_shared<Robot<Dynamic>>(this->buildRobotFromParametersFile());

        RCLCPP_INFO(this->get_logger(), "Loaded robot successfully. Robot configuration is:");
        this->R->print();

        // Build IKSolver and declare parameters
        this->IKS = std::make_shared<IKSolver<Dynamic>>(
            this->R,
            this->declare_parameter("max_iterations", 200),
            this->getAlgorithm_(this->declare_parameter("algorithm", "ALGORITHM_QUIK")),
            this->declare_parameter("exit_tolerance", 1e-12),
            this->declare_parameter("minimum_step_size", 1e-14),
            this->declare_parameter("relative_improvement_tolerance", 0.05),
            this->declare_parameter("max_consecutive_grad_fails", 10),
            this->declare_parameter("max_gradient_fails", 80),
            this->declare_parameter("lambda_squared", 1e-10),
            this->declare_parameter("max_linear_step_size", 0.34),
            this->declare_parameter("max_angular_step_size", 1.0),
            this->declare_parameter("armijo_sigma", 1e-5),
            this->declare_parameter("armijo_beta", 0.5)
        );

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

    std::shared_ptr<IKSolver<Dynamic>> IKS;
    std::shared_ptr<Robot<Dynamic>> R;

private:
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
        int DOF=Q0.rows();

        // Init output variables
        VectorXd Q_star(DOF);
        Vector<double,6> e_star;
        int iter;
        BREAKREASON_t breakReason;

        // Use the IK function
        this->IKS->IK( quat, d, Q0, Q_star, e_star, iter, breakReason);

        // Convert output values back to response
        for (int i = 0; i < Q_star.size(); ++i) response->q_star.push_back(Q_star[i]);
        // for (int i = 0; i < e_star.size(); ++i) response->e_star.push_back(e_star[i]);
        for (int i = 0; i < e_star.size(); ++i) response->e_star[i] = e_star[i];
        response->iter = iter;
        response->break_reason = breakReason;
        response->success = breakReason == BREAKREASON_TOLERANCE;

        // RCLCPP_INFO(this->get_logger(), "Sending back response: [%s]", "joint angles calculated");
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

        // Convert the incoming joint angles to an Eigen Vector
        Eigen::VectorXd Q = Eigen::VectorXd::Map(request->q.data(), request->q.size());
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
        Eigen::Matrix4d T;
        Vector<double, 4> quat;
        Vector<double, 3> d;

        // Call the FKn function
        this->R->FKn(Q, T, frame);

        // Convert to quaternion and position
        Geometry::hgt2quatpos(T, quat, d);

        // Convert the result to a geometry_msgs::Pose message
        response->pose.position.x = d(0);
        response->pose.position.y = d(1);
        response->pose.position.z = d(2);
        response->pose.orientation.x = quat(0);
        response->pose.orientation.y = quat(1);
        response->pose.orientation.z = quat(2);
        response->pose.orientation.w = quat(3);
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

       // Convert the incoming joint angles to an Eigen Vector
        Eigen::VectorXd Q = Eigen::VectorXd::Map(request->q.data(), request->q.size());
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
        Eigen::Matrix<double, 6, Dynamic> J(6, this->R->dof);

        // Call the jacobian function with the joint angle calling signature
        this->R->jacobian(Q, J);

        // Flatten the Jacobian matrix and assign to the response
        Eigen::VectorXd J_flat = Eigen::Map<Eigen::VectorXd>(J.data(), J.size());
        response->jacobian.assign(J_flat.data(), J_flat.data() + J_flat.size());
    }

    /**
     * @brief Reads in the parameters file and builds a robot, assigns it to this->R.
     * 
     * @return The robot object
     */
    Robot<Dynamic> buildRobotFromParametersFile()
    {
        // Declare parameters for and build robot
        std::vector<double> dh_param = this->declare_parameter("dh", std::vector<double>{-1.0});
        std::vector<std::string> link_types_str = this->declare_parameter("link_types", std::vector<std::string>{"invalid"});
        std::vector<double> q_sign_double = this->declare_parameter("q_sign", std::vector<double>{-1.0});
        Matrix4d Tbase = this->parseMatrix4d_(this->declare_parameter("Tbase", std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));
        Matrix4d Ttool = this->parseMatrix4d_(this->declare_parameter("Ttool", std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));

        // Check if parameters are still at their default (invalid) values
        if (dh_param.size() == 1 && dh_param[0] == -1.0)
            RCLCPP_ERROR(this->get_logger(), "Parameter 'dh' has not been set.");
        if (link_types_str.size() == 1 && link_types_str[0] == "invalid")
            RCLCPP_ERROR(this->get_logger(), "Parameter 'link_types' has not been set.");
        if (q_sign_double.size() == 1 && q_sign_double[0] == -1.0)
            RCLCPP_ERROR(this->get_logger(), "Parameter 'q_sign' has not been set.");
        if (q_sign_double.size() != link_types_str.size() || q_sign_double.size() != dh_param.size()/4)
            RCLCPP_ERROR(this->get_logger(), "DH, q_sign and linktype variables don't have congruent sizing");

        // Get Robot and IKSolver arguments, parse them into Eigen objects
        // DH Parameters
        Matrix<double, Dynamic, 4> DH = Map<Matrix<double, 4, Dynamic>>(dh_param.data(), 4, dh_param.size()/4).transpose();

        // Link types
        std::vector<JOINTTYPE_t> link_types_data;
        for (const auto& lt : link_types_str) link_types_data.push_back(this->getJointType_(lt)); // Convert from string to JOINTTYPE_t
        Vector<JOINTTYPE_t,Dynamic> link_types = Map<Vector<JOINTTYPE_t,Dynamic>, Unaligned>(link_types_data.data(), link_types_data.size());

        // Q sign
        VectorXd q_sign = Eigen::Map<VectorXd>(q_sign_double.data(), q_sign_double.size());

        // Build robot
        return Robot<Dynamic>(
            DH,
            link_types,
            q_sign,
            Tbase,
            Ttool);
    }

    void buildIKSolverFromParametersFile()
    {

    }

    /**
     * @brief Help convert string-based YAML parameters into proper ENUM types
     * 
     * @param algorithm 
     * @return ALGORITHM_t 
     */
    ALGORITHM_t getAlgorithm_(const std::string& algorithm)
    {
        if (algorithm == "ALGORITHM_QUIK") return ALGORITHM_QUIK;
        if (algorithm == "ALGORITHM_NR") return ALGORITHM_NR;
        if (algorithm == "ALGORITHM_BFGS") return ALGORITHM_BFGS;
        throw std::runtime_error("Invalid algorithm type: " + algorithm);
    }

    /**
     * @brief Help convert string-based YAML parameters into proper ENUM types
     * 
     * @param jointType 
     * @return JOINTTYPE_t 
     */
    JOINTTYPE_t getJointType_(const std::string& jointType)
    {
        if (jointType == "JOINT_REVOLUTE") return JOINT_REVOLUTE;
        if (jointType == "JOINT_PRISMATIC") return JOINT_PRISMATIC;
        throw std::runtime_error("Invalid joint type: " + jointType);
    }

    /**
     * @brief Parses a 4x4 matrix from a YAML parameter into an
     * Eigen object
     * 
     * @param values 
     * @return Matrix4d 
     */
    Matrix4d parseMatrix4d_(const std::vector<double>& values)
    {
        if (values.size() != 16) throw std::runtime_error("Invalid size for 4x4 matrix");
        Matrix4d matrix;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix(i, j) = values[i * 4 + j];
            }
        }
        return matrix;
    }

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