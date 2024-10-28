#include "rclcpp/rclcpp.hpp"
#include "quik/srv/ik_service.hpp"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/Geometry.hpp"

using namespace Eigen;
using namespace std;

class IKServiceNode : public rclcpp::Node
{
public:
    IKServiceNode() : Node("ik_service")
    {

        // Declare parameters
        this->declare_parameter("max_iterations", 200);
        this->declare_parameter("algorithm", "ALGORITHM_QUIK");
        this->declare_parameter("exit_tolerance", 1e-12);
        this->declare_parameter("minimum_step_size", 1e-14);
        this->declare_parameter("relative_improvement_tolerance", 0.05);
        this->declare_parameter("max_consecutive_grad_fails", 10);
        this->declare_parameter("max_gradient_fails", 80);
        this->declare_parameter("lambda_squared", 1e-10);
        this->declare_parameter("max_linear_step_size", 0.34);
        this->declare_parameter("max_angular_step_size", 1.0);
        this->declare_parameter("armijo_sigma", 1e-5);
        this->declare_parameter("armijo_beta", 0.5);
        this->declare_parameter("Tbase", std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
        this->declare_parameter("Ttool", std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});

        // Declare parameters without default values
        this->declare_parameter("dh", std::vector<double>{-1.0});
        this->declare_parameter("link_types", std::vector<std::string>{"invalid"});
        this->declare_parameter("q_sign", std::vector<double>{-1.0});

        // Get parameters values without defaults
        std::vector<double> dh_param = this->get_parameter("dh").as_double_array();
        std::vector<std::string> link_types_str = this->get_parameter("link_types").as_string_array();
        std::vector<double> q_sign_double = this->get_parameter("q_sign").as_double_array();

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
        Matrix<double, Dynamic, 4> DH = Map<Matrix<double, Dynamic, 4>>(dh_param.data(), dh_param.size()/4, 4);

        // Link types
        std::vector<JOINTTYPE_t> link_types_data;
        for (const auto& lt : link_types_str) link_types_data.push_back(this->getJointType_(lt)); // Convert from string to JOINTTYPE_t
        Vector<JOINTTYPE_t,Dynamic> link_types = Map<Vector<JOINTTYPE_t,Dynamic>, Unaligned>(link_types_data.data(), link_types_data.size());

        // Q sign
        VectorXd q_sign = Eigen::Map<VectorXd>(q_sign_double.data(), q_sign_double.size());

        // Tbase and Ttool
        Matrix4d Tbase = this->parseMatrix4d_(this->get_parameter("Tbase").as_double_array());
        Matrix4d Ttool = this->parseMatrix4d_(this->get_parameter("Ttool").as_double_array());

        // Build robot
        this->R = std::make_shared<Robot<Dynamic>>(
            DH,
            link_types,
            q_sign,
            Tbase,
            Ttool);

        // Build IKSolver
        this->IKS = std::make_shared<IKSolver<Dynamic>>(
            this->R,
            this->get_parameter("max_iterations").as_int(),
            this->getAlgorithm_(this->get_parameter("algorithm").as_string()),
            this->get_parameter("exit_tolerance").as_double(),
            this->get_parameter("minimum_step_size").as_double(),
            this->get_parameter("relative_improvement_tolerance").as_double(),
            this->get_parameter("max_consecutive_grad_fails").as_int(),
            this->get_parameter("max_gradient_fails").as_int(),
            this->get_parameter("lambda_squared").as_double(),
            this->get_parameter("max_linear_step_size").as_double(),
            this->get_parameter("max_angular_step_size").as_double(),
            this->get_parameter("armijo_sigma").as_double(),
            this->get_parameter("armijo_beta").as_double()
        );

        // Declare service
        this->srv_ = this->create_service<quik::srv::IKService>(
            "ik_service",
            std::bind(&IKServiceNode::handle_service, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(), "IKService initialized");
        RCLCPP_INFO(this->get_logger(), "Robot configuration is:");
        this->R->print();
        RCLCPP_INFO(this->get_logger(), "IKSolver configuration is:");
        this->IKS->printOptions();

    }

    std::shared_ptr<IKSolver<Dynamic>> IKS;
    std::shared_ptr<Robot<Dynamic>> R;

private:
    void handle_service(
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
        VectorXd Q0(request->q_0.data.size());
        for (size_t i = 0; i < request->q_0.data.size(); ++i) Q0(i) = request->q_0.data[i];
        int DOF=Q0.rows();

        // Init output variables
        VectorXd Q_star(DOF);
        Vector<double,6> e_star;
        int iter;
        BREAKREASON_t breakReason;

        // Use the IK function
        this->IKS->IK( quat, d, Q0, Q_star, e_star, iter, breakReason);

        // Convert output values back to response
        for (int i = 0; i < Q_star.size(); ++i) response->q_star.data.push_back(Q_star[i]);
        for (int i = 0; i < e_star.size(); ++i) response->e_star.data.push_back(e_star[i]);
        response->iter.data = iter;
        response->break_reason.data = breakReason;

        RCLCPP_INFO(this->get_logger(), "Sending back response: [%s]", "joint angles calculated");
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

    rclcpp::Service<quik::srv::IKService>::SharedPtr srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKServiceNode>());
    rclcpp::shutdown();
    return 0;
}