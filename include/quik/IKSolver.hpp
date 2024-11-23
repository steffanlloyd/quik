/**
 * @file IKSolver.hpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief IKSOLVER Builds a structure that has the parameters and methods to solve inverse kinematics
 * 
 * The key parameters:
 *      - max_iterations [int]: Maximum number of iterations of the
 *        algorithm. Default: 100
 *      - algorithm [ALGORITHM_t]: The algorithm to use
 *        ALGORITHM_QUIK - QuIK
 *        ALGORITHM_NR - Newton-Raphson or Levenberg-Marquardt
 *        ALGORITHM_BFGS - BFGS
 *        Default: ALGORITHM_QUIK.
 *      - exit_tolerance [double]: The exit tolerance on the norm of the
 *        error. Default: 1e-12.
 *      - minimum_step_size [double]: The minimum joint angle step size
 *        (normed) before the solver exits. Default: 1e-14.
 *      - relative_improvement_tolerance [double]: The minimum relative
 *        iteration-to-iteration improvement. If this threshold isn't
 *        met, a counter is incremented. If the threshold isn't met
 *        [max_consecutive_grad_fails] times in a row, then the algorithm exits.
 *        For example, 0.05 represents a minimum of 5// relative
 *        improvement. Default: 0.05.
 *      - max_consecutive_grad_fails [int]: The maximum number of relative
 *        improvement fails before the algorithm exits. Default:
 *        20.
 *      - lambda_squared [double]: The square of the damping factor, lambda.
 *        Only applies to the NR and QuIK methods. If given, these
 *        methods become the DNR (also known as levenberg-marquardt)
 *        or the DQuIK algorithm. Ignored for BFGS algorithm.
 *        Default: 0.
 *      - max_linear_step_size [double]: An upper limit of the error step
 *        in a single step. Ignored for BFGS algorithm. Default: 0.3.
 *      - max_angular_step_size [double]: An upper limit of the error step
 *        in a single step. Ignored for BFGS algorithm. Default: 1.
 *      - armijo_sigma [double]: The sigma value used in armijo's
 *        rule, for line search in the BFGS method. Default: 1e-5
 *      - armijo_beta [double]: The beta value used in armijo's
 *        rule, for line search in the BFGS method. Default: 0.5
 * 
 * Key methods defined are documented below, but are:
 *     - quik::IKSolver::IK(Twt, Q0, Q_star, e_star, iter, breakReason): Computes
 *       the inverse kinematics for a single pose, given as a homogeneous transform.
 *     - quik::IKSolver::IK(Twt, Q0, Q_star, e_star, iter, breakReason): Computes
 *       the inverse kinematics for a single pose, given as a homogeneous transform.
 *       This syntax accepts an array of transforms as input, and outputs an array 
 *       of angles.
 *     - quik::IKSolver::IK(quat, d, Q0, Q_star, e_star, iter, breakReason): Computes
 *       the inverse kinematics for a single pose, given as a quaternion quat and point d.
 *     - quik::IKSolver::IK(quat, d, Q0, Q_star, e_star, iter, breakReason): Computes
 *       the inverse kinematics for a single pose, given as a quaternion quat and point d.
 *       This syntax accepts an array of transforms as input, and outputs an array 
 *       of angles.
 *     - printOptions(): Displays the options of the current object.
 * 
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 */

#pragma once

#include <memory>
#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "quik/Robot.hpp"
#include "quik/geometry.hpp"


using namespace Eigen;
using namespace std;

namespace quik{

enum BREAKREASON_t : uint8_t {
    BREAKREASON_TOLERANCE = 0, // Tolerance reached
    BREAKREASON_MIN_STEP, // minimum step size is reached
    BREAKREASON_MAX_ITER, // Max iterations reached
    BREAKREASON_GRAD_FAILS // Gradient failed to improve
};

inline quik::BREAKREASON_t str2breakreason(std::string breakReason) {
    if (breakReason == "BREAKREASON_TOLERANCE") return quik::BREAKREASON_TOLERANCE;
    else if (breakReason == "BREAKREASON_MIN_STEP") return quik::BREAKREASON_MIN_STEP;
    else if (breakReason == "BREAKREASON_MAX_ITER") return quik::BREAKREASON_MAX_ITER;
    else if (breakReason == "BREAKREASON_GRAD_FAILS") return quik::BREAKREASON_GRAD_FAILS;
    else throw std::runtime_error("Invalid BREAKREASON string");
};

inline std::string breakreason2str(quik::BREAKREASON_t breakReason) {
    switch(breakReason) {
        case quik::BREAKREASON_TOLERANCE: return "BREAKREASON_TOLERANCE";
        case quik::BREAKREASON_MIN_STEP: return "BREAKREASON_MIN_STEP";
        case quik::BREAKREASON_MAX_ITER: return "BREAKREASON_MAX_ITER";
        case quik::BREAKREASON_GRAD_FAILS: return "BREAKREASON_GRAD_FAILS";
        default: return "UNKNOWN_BREAKREASON";
    }
};

enum ALGORITHM_t : uint8_t {
    ALGORITHM_QUIK = 0, // Recommended: The QuIK method
    ALGORITHM_NR, // Newton-Raphson or Levenberg-Marquardt
    ALGORITHM_BFGS // Not recommended: The BFGS line search
};

inline ALGORITHM_t str2algorithm(const std::string& algorithm)
{
    if (algorithm == "ALGORITHM_QUIK") return ALGORITHM_QUIK;
    else if (algorithm == "ALGORITHM_NR") return ALGORITHM_NR;
    else if (algorithm == "ALGORITHM_BFGS") return ALGORITHM_BFGS;
    else throw std::runtime_error("Invalid ALGORITHM_t string");
};

inline std::string algorithm2str(ALGORITHM_t algorithm) {
    switch(algorithm) {
        case ALGORITHM_QUIK: return "ALGORITHM_QUIK";
        case ALGORITHM_NR: return "ALGORITHM_NR";
        case ALGORITHM_BFGS: return "ALGORITHM_BFGS";
        default: return "UNKNOWN_ALGORITHM";
    }
};


template<int DOF=Dynamic>
class IKSolver {
public:

    // @brief Robot R: The robot object that is being solved.
    std::shared_ptr<Robot<DOF>> R;
    
    // @brief max_iterations [int]: Maximum number of iterations of the algorithm. Default: 100
	int max_iterations;
    
    // @brief algorithm [ALGORITHM_t]: The algorithm to use
    //     - ALGORITHM_QUIK - QuIK
    //     - ALGORITHM_NR - Newton-Raphson or Levenberg-Marquardt
    //     - ALGORITHM_BFGS - BFGS
    //     - Default: 0.
	quik::ALGORITHM_t algorithm;

    // @brief The exit tolerance on the norm of the
    // error. Default: 1e-12.
	double exit_tolerance;

    // minimum_step_size [double]: The minimum joint angle step size
    // (normed) before the solver exits. Default: 1e-14.
	double minimum_step_size;

    // @brief relative_improvement_tolerance [double]: The minimum relative
    // iteration-to-iteration improvement. If this threshold isn't
    // met, a counter is incremented. If the threshold isn't met
    // [max_consecutive_grad_fails] times in a row, then the algorithm exits.
    // For example, 0.05 represents a minimum of 5 relative
    // improvement. Default: 0.05.
	double relative_improvement_tolerance;

    // @brief max_consecutive_grad_fails [int]: The maximum number of relative
    // improvement fails before the algorithm exits. Default: 20.
	int max_consecutive_grad_fails;
	int max_gradient_fails;

    // @brief The square of the damping factor, lambda.
    // Only applies to the NR and QuIK methods. If given, these
    // methods become the DNR (also known as levenberg-marquardt)
    // or the DQuIK algorithm. Ignored for BFGS algorithm.
    // Default: 0.
	double lambda_squared;

    // @brief max_linear_step_size [double]: An upper limit of the error step
    // in a single step. Ignored for BFGS algorithm. 
    // Give a negative value to compute it automatically from the characteristic 
    // length of the robot. Default: 0.33 * Robot.characteristicLength().
	double max_linear_step_size;

    // @brief max_angular_step_size [double]: An upper limit of the error step
    // in a single step. Ignored for BFGS algorithm. Default: 1.
	double max_angular_step_size;

    // @brief armijo_sigma [double]: The sigma value used in armijo's
    // rule, for line search in the BFGS method. Default: 1e-5
	double armijo_sigma;


    // @brief armijo_beta [double]: The beta value used in armijo's
    // rule, for line search in the BFGS method. Default: 0.5
	double armijo_beta;

    // Constructor
	IKSolver(
        std::shared_ptr<Robot<DOF>> _R,
        int _max_iterations = 100,
        quik::ALGORITHM_t _algorithm = quik::ALGORITHM_QUIK,
        double _exit_tolerance = 1e-12,
        double _minimum_step_size = 1e-14,
        double _relative_improvement_tolerance = 0.05,
        int _max_consecutive_grad_fails = 5,
        int _max_gradient_fails = 20,
        double _lambda_squared = 0,
        double _max_linear_step_size = -1,
        double _max_angular_step_size = 1,
        double _armijo_sigma = 1e-5,
        double _armijo_beta = 0.5 )
        :   R(_R),
            max_iterations(_max_iterations),
            algorithm(_algorithm),
            exit_tolerance(_exit_tolerance),
            minimum_step_size(_minimum_step_size),
            relative_improvement_tolerance(_relative_improvement_tolerance),
            max_consecutive_grad_fails(_max_consecutive_grad_fails),
            max_gradient_fails(_max_gradient_fails),
            lambda_squared(_lambda_squared),
            max_linear_step_size(_max_linear_step_size),
            max_angular_step_size(_max_angular_step_size),
            armijo_sigma(_armijo_sigma),
            armijo_beta(_armijo_beta)
    {

        if(this->max_linear_step_size <= 0){
            // Automatically compute the best step size from the robot characteristic length
            this->max_linear_step_size = 0.33 * this->R->characteristicLength();
        }
        if(this->max_linear_step_size < 0.05 * this->R->characteristicLength()){
            cout << "Warning! The provided max_linear_step_size is very small." << endl <<
                "The recommended value of max_linear_step_size is 33% of Robot.charactersisticLength(). " <<
                "The provided value of " << this->max_linear_step_size << " is " <<
                this->max_linear_step_size/this->R->characteristicLength()*100 <<
                "% instead. This is very small and will significantly impact the " <<
                "algorithm performance. Recommend to increase it or leave it to be " <<
                "automatically computed." << endl;
        }

        // Input checking
		if(this->max_iterations <= 0) throw std::runtime_error("max_iterations must be positive and nonzero!");
		if(this->exit_tolerance <= 0) throw std::runtime_error("exit_tolerance must be a positive number!");
		if(this->minimum_step_size < 0) throw std::runtime_error("minimum_step_size must be a positive number or zero!");
		if(this->relative_improvement_tolerance <= 0) throw std::runtime_error("relative_improvement_tolerance must be a positive number!");
		if(this->max_consecutive_grad_fails <= 0) throw std::runtime_error("max_consecutive_grad_fails must be a positive integer!");
		if(this->max_gradient_fails <= 0) throw std::runtime_error("max_gradient_fails must be a positive integer!");
		if(this->lambda_squared < 0) throw std::runtime_error("lambda_squared must be a positive number or zero!");
		if(this->max_linear_step_size <= 0) throw std::runtime_error("max_linear_step_size must be a positive number!");
		if(this->max_angular_step_size <= 0) throw std::runtime_error("max_angular_step_size must be a positive number!");
		if(this->armijo_sigma <= 0) throw std::runtime_error("armijo_sigma must be a positive number!");
		if(this->armijo_beta <= 0) throw std::runtime_error("armijo_beta must be a positive number!");
    }

    /**
     * @brief IK A basic IK implementation of the QuIK, NR and BFGS algorithms. 
     * 
     * @param[in] Twt Matrix4d& Twt: A transformation matrix from the world frame
     * to the tool frame.
     * @param[in] Q0 Matrix<double,DOF>& Q0: Initial guesses of the joint angles
     * @param[out] Q_star  Matrix<double,DOF>& Qstar: [DOF] The solved joint angles
     * @param[out] e_star Matrix<double,6>&e: The pose errors at the final solution.
     * @param[out] iter int iter: The number of iterations the algorithm took.
     * @param[out] breakReason BREAKREASON_t breakReason: The reason the algorithm stopped.
     *          See BREAKREASON_t for list of reasons.
     */
    void IK(
		const Matrix4d& Twt,
		const Vector<double,DOF>& Q0,
		Vector<double,DOF>& Q_star,
		Vector<double,6>& e_star,
		int& iter,
		quik::BREAKREASON_t& breakReason) const
    {
        // Initialize variables
        Vector<double,DOF>  Q = Q0;     // Holds the current solution guess
        Vector<double,DOF>  dQ,         // Holds the iterative joint step computed by the algorithm
                            s0,         // Holds the step size in the BFGS line search
                            grad_i,     // Gradient of current iteration (used in BFGS)
                            grad_ip1,   //  Gradient of next iteration (used in BFGS)
                            y;          // Estimated Hessian (used in BFGS)
        Vector<double,6>    e;          // The error vector.
        Matrix<double,6,DOF> J(6, this->R->dof), // Holds the robot jacobian
                            A;          // Holds the Hessian product term for the QuIK algorithm
        Matrix<double,DOF,DOF> H_i;     // H variable, used in bfgs algorithm
        int 	grad_fail_counter = 0,  // Holds a count of the times the gradient has failed
                grad_fail_counter_total = 0;
        double 	e_norm = 0,             // Holds the normed error
                e_prev_norm = 1e10,     // Holds the normed error (previous iteration)
                error_relImprovement = 0,
                cost_i = 1e10,          // Used in BFGS
                cost_ip1 = 1e10,        // Used in BFGS
                gamma,                  // Used in BFGS
                rho,                    // Used in BFGS
                delta;                  // Used in BFGS

        // Init variable that can store all the transforms for each frame of orobt
        constexpr int DOF4 = DOF>0 ? (DOF+1)*4 : -1;
        Matrix<double,DOF4,4> T((this->R->dof+1)*4, 4); // Holds the forward kinematics for each joint

        // Preassign some values
        e.fill(0);
        dQ.fill(0);
        iter = this->max_iterations;
        breakReason = quik::BREAKREASON_MAX_ITER; // Initialize to this, it will be overwritten if it doesn't reach max iter
        
        // Start IK iterations
        for (int i = 0; i < this->max_iterations; i++){
            
            // Get error, forward kinematics and jacobian
            // Only do this for Newton and QuIK, or on first iteration
            if (this->algorithm != quik::ALGORITHM_BFGS || i == 0){
                // Update T with forward kinematics
                this->R->FK( Q, T );
                
                // Get jacobian, store it in J
                this->R->jacobian(T, J, true);
                
                // Update error between target and current error, store in e
                geometry::hgtDiff( T.template bottomRows<4>(), Twt, e );
            }
            
            // Calculate norm
            e_norm = e.norm();

            // Break, if exit tolerance has been reached
            if (e_norm < this->exit_tolerance){
                breakReason = quik::BREAKREASON_TOLERANCE; // Tolerance reached
                iter = i;
                break;
            }
            
            // Check relative improvement in error
            // We break if the relative improvement fails this->max_consecutive_grad_fails times in a row, or if
            // it fails this->max_gradient_fails total
            error_relImprovement = (e_prev_norm - e_norm) / e_prev_norm;
            if (error_relImprovement < this->relative_improvement_tolerance){
                // If relative improvement is below threshold, increment counters
                grad_fail_counter++;
                grad_fail_counter_total++;
                if (grad_fail_counter > this->max_consecutive_grad_fails) {
                    breakReason = quik::BREAKREASON_GRAD_FAILS; // Grad consecutive fails reached
                    iter = i;
                    break;
                }
                if (grad_fail_counter_total > this->max_gradient_fails) {
                    breakReason = quik::BREAKREASON_GRAD_FAILS; // Grad fails reached
                    iter = i;
                    break;
                }
            }else{
                grad_fail_counter = 1;
            }

            // Store prev value
            e_prev_norm = e_norm;
            
            // Clamp error, before taking steps
            this->clampMag(e);
            
            // Go to switch statement to do work of each individual algorithm
            switch (this->algorithm){
                    
                    
                case quik::ALGORITHM_QUIK:
                    // Halley's method (QuIK Method)
                    
                    // First, store the newton step in dQ (note, it's negative)
                    this->lsolve( J, e, dQ);
                    
                    // Then, negate it and divide by two
                    dQ *= -0.5;
                    
                    // Assign jacobian to A so that it gets added to it
                    A = J;

                    // Get gradient product, this gets added automatically since A holds J
                    this->R->hessianProduct( J, dQ, A );
                                        
                    // Resolve
                    this->lsolve(A, e, dQ);
                    dQ *= -1;
                    
                    break;
                    
                    
                    
                case quik::ALGORITHM_NR:
                    // Newton's method
                    this->lsolve( J, e, dQ);
                    dQ *= -1;
                    break;
                    
                    
                    
                case quik::ALGORITHM_BFGS:
                    // BFGS
                    // On first iteration, initialize some variables
                    if (i == 0){
                        H_i = Matrix<double,DOF,DOF>::Identity(this->R->dof, this->R->dof);
                        grad_i = J.transpose() * e;
                        cost_i = 0.5*e.array().square().sum();
                    }
                    
                    // Get initial step
                    s0 = -H_i*grad_i;
                    
                    // Initialize line search
                    gamma = 1;
                    
                    // Recalculate cost and error
                    this->R->FK( Q + gamma*s0, T );
                    geometry::hgtDiff( T.template bottomRows<4>(), Twt, e );
                    cost_ip1 = 0.5*e.array().square().sum();
                    
                    // Do line search
                    while ((cost_i - cost_ip1) < -this->armijo_sigma * grad_i.transpose()*(gamma*s0)){
                        // Reduce gamma
                        gamma = this->armijo_beta * gamma;
                        
                        // Break if step size is too small (prevents infinite loops too)
                        if (gamma < this->minimum_step_size) break;
                        
                        // Recalculate cost
                        this->R->FK( Q + gamma*s0, T );
                        geometry::hgtDiff( T.template bottomRows<4>(), Twt, e );
                        cost_ip1 = 0.5*e.array().square().sum();
                    }
                    
                    // Break out if step size is too small
                    if (gamma < this->minimum_step_size){
                        breakReason = quik::BREAKREASON_MIN_STEP; // reached minimum step size
                        iter = i;
                        break;
                    }
                    
                    // Take step
                    dQ = gamma*s0;
                    
                    // Update gradient (T and e are already updated)
                    this->R->jacobian(T, J);
                    grad_ip1 = J.transpose() * e;
                    
                    // Update gradient
                    y = grad_ip1 - grad_i;
                    rho = dQ.transpose() * y;
                    delta = y.transpose() * H_i * y;
                    if (rho > delta && rho > numeric_limits<double>::epsilon())
                        H_i = H_i + ( (1 + delta/rho) * dQ*dQ.transpose() - dQ*y.transpose()*H_i - H_i*y*dQ.transpose())/rho;
                    else if (delta > numeric_limits<double>::epsilon() && rho > numeric_limits<double>::epsilon())
                        H_i = H_i + (dQ*dQ.transpose())/rho - H_i*(y*y.transpose())*H_i/delta;
                    
                    // Update variables for next time
                    grad_i = grad_ip1;
                    cost_i = cost_ip1;
                    
                    break;

                    
                default:
                    // invalid input
                    cout << "Invalid algorithm specified!" << endl;
                    dQ.fill(0);
                    
                    
            } // end of algorithm switch statement
                        
            // Apply change
            Q += dQ;
            
            // Check grad tolerance, break if necessary
            if (dQ.array().square().sum() < this->minimum_step_size * this->minimum_step_size){
                breakReason = quik::BREAKREASON_MIN_STEP; // minimum step sized reached
                iter = i;
                break;
            }
            
        } // End of IK loop
        
        // Store solutions
        Q_star = Q;
        e_star = e;
    }

    /**
     * @brief IK Alternate calling syntax where the pose is provided using a quaternion
     * and position vector instead of a transformation matrix
     * 
     * @param[in] quat Vector<double,4>& quat: A 4-vector (x,y,z,w), or 4xN matrix of quaternions
     * (one column for each pose to solve).
     * @param[in] d Vector<double,3>& d: A 3-vector (x,y,z), or 3xN matrix of displacement
     * vectors (one column for each pose to solve).
     * @param[in] Q0 Vector<double,DOF>& Q0: Initial guesses of the joint angles
     * @param[out] Q_star  Matrix<double,DOF>& Qstar: [DOF] The solved joint angles
     * @param[out] e_star Matrix<double,6>&e: The pose errors at the final solution.
     * @param[out] iter int iter: The number of iterations the algorithm took.
     * @param[out] breakReason BREAKREASON_t breakReason: The reason the algorithm stopped.
     *          See BREAKREASON_t for list of reasons.
     */
    void IK(
		const Vector<double,4>& quat,
		const Vector<double,3>& d,
		const Vector<double,DOF>& Q0,
		Vector<double,DOF>& Q_star,
		Vector<double,6>& e_star,
		int& iter,
		quik::BREAKREASON_t& breakReason) const
    {
        // Initialize and compute Twt
        Matrix4d Twt;
        geometry::quatpos2hgt(quat, d, Twt);

        // Call the first version of IK
        this->IK(Twt, Q0, Q_star, e_star, iter, breakReason);
    }


    /**
     * @brief IK A basic IK implementation of the QuIK, NR and BFGS algorithms. This calling 
     * syntax allows to solve multiple inverse kinematics at once.
     * 
     * @param[in] Twt Matrix<double,4*N,4>& Twt: A transformation matrix from the world frame
     * to the tool frame. To solve more than 1 transform simulataneously, stack the matrices on 
     * top of each other. So for 4 poses, Twt would be a 16x4 matrix.
     * @param[in] Q0 Matrix<double,DOF,N>& Q0: Initial guesses of the joint angles
     * @param[out] Q_star  Matrix<double,DOF,N>& Qstar: [DOFxN] The solved joint angles
     * @param[out] e_star Matrix<double,6,N>&e: The pose errors at the final solution.
     * @param[out] iter std::vector<int>& iter: The number of iterations the algorithm took.
     * @param[out] breakReason std::vector<BREAKREASON_t>& breakReason: The reason the algorithm stopped.
     *          See BREAKREASON_t for list of reasons.
     */
    void IK(
		const Matrix<double,Dynamic,4>& Twt,
		const Matrix<double,DOF,Dynamic>& Q0,
		Matrix<double,DOF,Dynamic>& Q_star,
		Matrix<double,6,Dynamic>& e_star,
		std::vector<int>& iter,
		std::vector<quik::BREAKREASON_t>& breakReason) const
    {
        // Get size of problem
        int N = (int) Q0.cols();

        std::cout<<"Test1" << " " <<Twt.rows()<<" "<<N<<std::endl;

        // Ensure inputs are properly given
        if(Twt.rows() != 4*N) throw std::runtime_error("Number of rows in Twt should be 4*N (where N is the number of poses to solve).");
        if(Q_star.cols() != N) throw std::runtime_error("Q_star must be a <DOFxN> matrix (where N is the number of poses to solve).");
        if(e_star.cols() != N) throw std::runtime_error("e_star must be a <6xN> matrix (where N is the number of poses to solve).");
        if(static_cast<int>(iter.size()) != N) throw std::runtime_error("iter must be a <6xN> matrix (where N is the number of poses to solve).");
        if(static_cast<int>(breakReason.size()) != N) throw std::runtime_error("breakReason must be a <6xN> matrix (where N is the number of poses to solve).");

        // Start iterations over poses to solve
        for (int i = 0; i < N; i++){
            // Init variables to store answers
            Vector<double,DOF> Q_star_i;
            Vector<double,6> e_star_i;
            int iter_i;
            quik::BREAKREASON_t breakReason_i;

            this->IK(
                Twt.middleRows<4>(4*i),
                Q0.col(i),
                Q_star_i,
                e_star_i,
                iter_i,
                breakReason_i);

            // Assign answers
            Q_star.col(i) = Q_star_i;
            e_star.col(i) = e_star_i;
            iter[i] = iter_i;
            breakReason[i] = breakReason_i;

        } // End of sample loop
        
    } // End of IK()


    /**
     * @brief IK Alternate calling syntax where every pose is called using a quaternion
     * and position vector instead of a transformation matrix
     * 
     * @param[in] quat Matrix<double,4,N>& quat: A 4-vector (x,y,z,w), or 4xN matrix of quaternions
     * (one column for each pose to solve).
     * @param[in] d Matrix<double,3,N>& d: A 3-vector (x,y,z), or 3xN matrix of displacement
     * vectors (one columh for each pose to solve)
     * @param[in] Q0 Matrix<double,DOF,N>& Q0: Initial guesses of the joint angles
     * @param[out] Q_star  Matrix<double,DOF,N>& Qstar: [DOFxN] The solved joint angles
     * @param[out] e_star Matrix<double,6,N>&e: The pose errors at the final solution.
     * @param[out] iter std::vector<int>& iter: The number of iterations the algorithm took.
     * @param[out] breakReason std::vector<BREAKREASON_t>& breakReason: The reason the algorithm stopped.
     *          See BREAKREASON_t for list of reasons.
     */
    void IK(
		const Matrix<double,4,Dynamic>& quat,
		const Matrix<double,3,Dynamic>& d,
		const Matrix<double,DOF,Dynamic>& Q0,
		Matrix<double,DOF,Dynamic>& Q_star,
		Matrix<double,6,Dynamic>& e_star,
		std::vector<int>& iter,
		std::vector<quik::BREAKREASON_t>& breakReason) const
    {
        // Convert to homogenous transform
        int N = quat.cols();
        Matrix<double,Dynamic,4> Twt(N*4,4);
        quik::geometry::quatpos2hgt(quat, d, Twt);

        // Call the first version of IK
        this->IK(Twt, Q0, Q_star, e_star, iter, breakReason);
    }

    
    /**
     * @brief Saturates the magnitude of the error vector before being
     * sent to the rest of the algorithm.
     * 
     * Implemented as defined here:
     * [1] S. R. Buss, “Introduction to Inverse Kinematics
     * with Jacobian Transpose, Pseudoinverse and Damped Least
     * Squares methods,” 2009.
     * https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
     * 
     * @param e 
     */
    void clampMag(Vector<double,6>& e) const
    {
        // Break out early if we're doing BFGS, this isn't relevant
        if (this->algorithm == 2) return;
        
        // Calculate the squared normed error
        // This avoids doing the sqrt unless necessary
        double ei_lin_norm2 = e.head<3>().array().square().sum();
        double ei_ang_norm2 = e.tail<3>().array().square().sum();

        // If either limit is greater than the square of the threshold, then rescale
        // the appropriate part of the error
        if (ei_lin_norm2 > (this->max_linear_step_size * this->max_linear_step_size))
            e.head<3>() *= this->max_linear_step_size / sqrt(ei_lin_norm2);
        
        if (ei_ang_norm2 > (this->max_angular_step_size * this->max_angular_step_size))
            e.tail<3>() *= this->max_angular_step_size / sqrt(ei_ang_norm2);
    } // End of clampMag()

    /**
     * @brief Solves a linear system (very fast!) with damping.
     * 
     * @param[in] A A 6xDOF matrix
     * @param[in] b A 6-vector
     * @param[out] x The solution vector, passed as reference (DOF-vector)
     */
    void lsolve(
        const Matrix<double,6,DOF>& A,
		const Vector<double,6>& b,
		Vector<double,DOF>& x) const
    {
        // Form covariance matrix
        Matrix<double, 6, 6> Astar = A*A.transpose();
        
        // If a damping term is given, add it to the diagonals
        if (this->lambda_squared > 0) Astar.diagonal().array() += this->lambda_squared;
        
        // Do LLT decomposition, since matrix is guaranteed to be positive definite.
        LLT<Matrix<double,6,6>> Astar_llt(Astar);
            
        // Solve
        x = A.transpose() * ( Astar_llt.solve(b) );
    }

    
	/**
	 * @brief Prints out the solver options
	 */
	void printOptions() const
    {
        cout << "\tIKSolver.max_iterations: " << this->max_iterations << endl;
        cout << "\tIKSolver.algorithm: " << quik::algorithm2str(this->algorithm) << endl;
        cout << "\tIKSolver.exit_tolerance: " << this->exit_tolerance << endl;
        cout << "\tIKSolver.minimum_step_size: " << this->minimum_step_size << endl;
        cout << "\tIKSolver.relative_improvement_tolerance: " << this->relative_improvement_tolerance << endl;
        cout << "\tIKSolver.max_consecutive_grad_fails: " << this->max_consecutive_grad_fails << endl;
        cout << "\tIKSolver.lambda_squared: " << this->lambda_squared << endl;
        cout << "\tIKSolver.max_linear_step_size: " << this->max_linear_step_size << endl;
        cout << "\tIKSolver.max_angular_step_size: " << this->max_angular_step_size << endl;
        if (this->algorithm == quik::ALGORITHM_BFGS){
            cout << "\tIKSolver.armijo_sigma: " << this->armijo_sigma << endl;
            cout << "\tIKSolver.armijo_beta: " << this->armijo_beta << endl;
        }
        cout << endl << endl;
    }

}; // End of class definition quik::IKSolver

} // End of namespace quik