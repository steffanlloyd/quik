//
//  IKSolver.hpp
//  QuIK
//
// IKSOLVER Builds a structure that has the parameters and methods to solve inverse kinematics
//
// The key parameters:
//
//       * iterMax [int]: Maximum number of iterations of the
//           algorithm. Default: 100
//       * algorithm [ALGORITHM_t]: The algorithm to use
//           ALGORITHM_QUIK - QuIK
//           ALGORITHM_NR - Newton-Raphson or Levenberg-Marquardt
//           ALGORITHM_BFGS - BFGS
//           Default: ALGORITHM_QUIK.
//       * exitTol [double]: The exit tolerance on the norm of the
//           error. Default: 1e-12.
//       * minStepSize [double]: The minimum joint angle step size
//           (normed) before the solver exits. Default: 1e-14.
//       * relImprovementTol [double]: The minimum relative
//           iteration-to-iteration improvement. If this threshold isn't
//           met, a counter is incremented. If the threshold isn't met
//           [maxGradFails] times in a row, then the algorithm exits.
//           For example, 0.05 represents a minimum of 5// relative
//           improvement. Default: 0.05.
//       * maxGradFails [int]: The maximum number of relative
//           improvement fails before the algorithm exits. Default:
//           20.
//       * lambda2 [double]: The square of the damping factor, lambda.
//           Only applies to the NR and QuIK methods. If given, these
//           methods become the DNR (also known as levenberg-marquardt)
//           or the DQuIK algorithm. Ignored for BFGS algorithm.
//           Default: 0.
//       * maxLinearErrorStep [double]: An upper limit of the error step
//           in a single step. Ignored for BFGS algorithm. Default: 0.3.
//       * maxAngularErrorStep [double]: An upper limit of the error step
//           in a single step. Ignored for BFGS algorithm. Default: 1.
//       * armijoRuleSigma [double]: The sigma value used in armijo's
//           rule, for line search in the BFGS method. Default: 1e-5
//       * armijoRuleBeta [double]: The beta value used in armijo's
//           rule, for line search in the BFGS method. Default: 0.5
//
//  Created by Steffan Lloyd on 2024-10-26.

#pragma once

#include <cassert>
#include <memory>
#include "Eigen/Dense"
#include "quik/Robot.hpp"
#include "quik/Geometry.hpp"


using namespace Eigen;

/**
 * @brief List of break reasons (reasons why the algorithm stopped)
 * 
 */
enum BREAKREASON_t : uint8_t {
    BREAKREASON_TOLERANCE = 0, // Tolerance reached
    BREAKREASON_MIN_STEP, // minimum step size is reached
    BREAKREASON_MAX_ITER, // Max iterations reached
    BREAKREASON_GRAD_FAILS // Gradient failed to improve
};

enum ALGORITHM_t : uint8_t {
    ALGORITHM_QUIK = 0, // Recommended: The QuIK method
    ALGORITHM_NR, // Newton-Raphson or Levenberg-Marquardt
    ALGORITHM_BFGS // Not recommended: The BFGS line search
};

template<int DOF=Dynamic>
class IKSolver {
public:

    // @brief Robot R: The robot object that is being solved.
    std::shared_ptr<Robot<DOF>> R;
    
    // @brief iterMax [int]: Maximum number of iterations of the algorithm. Default: 100
	int iterMax;
    
    // @brief algorithm [ALGORITHM_t]: The algorithm to use
    //     - ALGORITHM_QUIK - QuIK
    //     - ALGORITHM_NR - Newton-Raphson or Levenberg-Marquardt
    //     - ALGORITHM_BFGS - BFGS
    //     - Default: 0.
	ALGORITHM_t algorithm;

    // @brief The exit tolerance on the norm of the
    // error. Default: 1e-12.
	double exitTol;

    // minStepSize [double]: The minimum joint angle step size
    // (normed) before the solver exits. Default: 1e-14.
	double minStepSize;

    // @brief relImprovementTol [double]: The minimum relative
    // iteration-to-iteration improvement. If this threshold isn't
    // met, a counter is incremented. If the threshold isn't met
    // [maxGradFails] times in a row, then the algorithm exits.
    // For example, 0.05 represents a minimum of 5 relative
    // improvement. Default: 0.05.
	double relImprovementTol;

    // @brief maxGradFails [int]: The maximum number of relative
    // improvement fails before the algorithm exits. Default: 20.
	int maxGradFails;
	int maxGradFailsTotal;

    // @brief The square of the damping factor, lambda.
    // Only applies to the NR and QuIK methods. If given, these
    // methods become the DNR (also known as levenberg-marquardt)
    // or the DQuIK algorithm. Ignored for BFGS algorithm.
    // Default: 0.
	double lambda2;

    // @brief maxLinearErrorStep [double]: An upper limit of the error step
    // in a single step. Ignored for BFGS algorithm. Default: 0.3.
	double maxLinearErrorStep;

    // @brief maxAngularErrorStep [double]: An upper limit of the error step
    // in a single step. Ignored for BFGS algorithm. Default: 1.
	double maxAngularErrorStep;

    // @brief armijoRuleSigma [double]: The sigma value used in armijo's
    // rule, for line search in the BFGS method. Default: 1e-5
	double armijoRuleSigma;


    // @brief armijoRuleBeta [double]: The beta value used in armijo's
    // rule, for line search in the BFGS method. Default: 0.5
	double armijoRuleBeta;

    // Constructor
	IKSolver(
        std::shared_ptr<Robot<DOF>> _R,
        int _iterMax = 100,
        ALGORITHM_t _algorithm = ALGORITHM_QUIK,
        double _exitTol = 1e-12,
        double _minStepSize = 1e-14,
        double _relImprovementTol = 0.05,
        int _maxGradFails = 5,
        int _maxGradFailsTotal = 20,
        double _lambda2 = 0,
        double _maxLinearErrorStep = .34,
        double _maxAngularErrorStep = 1,
        double _armijoRuleSigma = 1e-5,
        double _armijoRuleBeta = 0.5 )
        :   R(_R),
            iterMax(_iterMax),
            algorithm(_algorithm),
            exitTol(_exitTol),
            minStepSize(_minStepSize),
            relImprovementTol(_relImprovementTol),
            maxGradFails(_maxGradFails),
            maxGradFailsTotal(_maxGradFailsTotal),
            lambda2(_lambda2),
            maxLinearErrorStep(_maxLinearErrorStep),
            maxAngularErrorStep(_maxAngularErrorStep),
            armijoRuleSigma(_armijoRuleSigma),
            armijoRuleBeta(_armijoRuleBeta)
    {}

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
		BREAKREASON_t& breakReason) const
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
        iter = this->iterMax;
        breakReason = BREAKREASON_MAX_ITER; // Initialize to this, it will be overwritten if it doesn't reach max iter
        
        // Start IK iterations
        for (int i = 0; i < this->iterMax; i++){
            
            // Get error, forward kinematics and jacobian
            // Only do this for Newton and QuIK, or on first iteration
            if (this->algorithm != ALGORITHM_BFGS || i == 0){
                // Update T with forward kinematics
                this->R->FK( Q, T );
                
                // Get jacobian, store it in J
                this->R->jacobian(T, J, true);
                
                // Update error between target and current error, store in e
                Geometry::hgtDiff( T.template bottomRows<4>(), Twt, e );
            }
            
            // Calculate norm
            e_norm = e.norm();

            // Break, if exit tolerance has been reached
            if (e_norm < this->exitTol){
                breakReason = BREAKREASON_TOLERANCE; // Tolerance reached
                iter = i;
                break;
            }
            
            // Check relative improvement in error
            // We break if the relative improvement fails this->maxGradFails times in a row, or if
            // it fails this->maxGradFailsTotal total
            error_relImprovement = (e_prev_norm - e_norm) / e_prev_norm;
            if (error_relImprovement < this->relImprovementTol){
                // If relative improvement is below threshold, increment counters
                grad_fail_counter++;
                grad_fail_counter_total++;
                if (grad_fail_counter > this->maxGradFails) {
                    breakReason = BREAKREASON_GRAD_FAILS; // Grad consecutive fails reached
                    iter = i;
                    break;
                }
                if (grad_fail_counter_total > this->maxGradFailsTotal) {
                    breakReason = BREAKREASON_GRAD_FAILS; // Grad fails reached
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
                    
                    
                case ALGORITHM_QUIK:
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
                    
                    
                    
                case ALGORITHM_NR:
                    // Newton's method
                    this->lsolve( J, e, dQ);
                    dQ *= -1;
                    break;
                    
                    
                    
                case ALGORITHM_BFGS:
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
                    Geometry::hgtDiff( T.template bottomRows<4>(), Twt, e );
                    cost_ip1 = 0.5*e.array().square().sum();
                    
                    // Do line search
                    while ((cost_i - cost_ip1) < -this->armijoRuleSigma * grad_i.transpose()*(gamma*s0)){
                        // Reduce gamma
                        gamma = this->armijoRuleBeta * gamma;
                        
                        // Break if step size is too small (prevents infinite loops too)
                        if (gamma < this->minStepSize) break;
                        
                        // Recalculate cost
                        this->R->FK( Q + gamma*s0, T );
                        Geometry::hgtDiff( T.template bottomRows<4>(), Twt, e );
                        cost_ip1 = 0.5*e.array().square().sum();
                    }
                    
                    // Break out if step size is too small
                    if (gamma < this->minStepSize){
                        breakReason = BREAKREASON_MIN_STEP; // reached minimum step size
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
            if (dQ.array().square().sum() < this->minStepSize * this->minStepSize){
                breakReason = BREAKREASON_MIN_STEP; // minimum step sized reached
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
		BREAKREASON_t& breakReason) const
    {
        // Initialize and compute Twt
        Matrix4d Twt;
        Geometry::quatpos2hgt(quat, d, Twt);

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
		std::vector<BREAKREASON_t>& breakReason) const
    {
        // Get size of problem
        int N = (int) Q0.cols();

        // Ensure inputs are properly given
        assert(Twt.rows() == 4*N && "Number of rows in Twt should be 4*N (where N is the number of poses to solve).");  
        assert(Q_star.cols() == N && "Q_star must be a <DOFxN> matrix (where N is the number of poses to solve).");
        assert(e_star.cols() == N && "e_star must be a <6xN> matrix (where N is the number of poses to solve).");
        assert(iter.size() == N && "iter must be a <6xN> matrix (where N is the number of poses to solve).");
        assert(breakReason.size() == N && "breakReason must be a <6xN> matrix (where N is the number of poses to solve).");       

        // Start iterations over poses to solve
        for (int i = 0; i < N; i++){
            // Init variables to store answers
            Vector<double,DOF> Q_star_i;
            Vector<double,6> e_star_i;
            int iter_i;
            BREAKREASON_t breakReason_i;

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
		std::vector<BREAKREASON_t>& breakReason) const
    {
        // Convert to homogenous transform
        constexpr int DOF4 = DOF>0 ? (DOF+1)*4 : -1;
        Matrix<double,DOF4,4> Twt;
        Geometry::quatpos2hgt(quat, d, Twt);

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
        if (ei_lin_norm2 > (this->maxLinearErrorStep * this->maxLinearErrorStep))
            e.head<3>() *= this->maxLinearErrorStep / sqrt(ei_lin_norm2);
        
        if (ei_ang_norm2 > (this->maxAngularErrorStep * this->maxAngularErrorStep))
            e.tail<3>() *= this->maxAngularErrorStep / sqrt(ei_ang_norm2);
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
        if (this->lambda2 > 0) Astar.diagonal().array() += this->lambda2;
        
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
        cout << "IKSolver.iterMax: " << this->iterMax << endl;
        cout << "IKSolver.algorithm: " << this->algorithm << endl;
        cout << "IKSolver.exitTol: " << this->exitTol << endl;
        cout << "IKSolver.minStepSize: " << this->minStepSize << endl;
        cout << "IKSolver.relImprovementTol: " << this->relImprovementTol << endl;
        cout << "IKSolver.maxGradFails: " << this->maxGradFails << endl;
        cout << "IKSolver.lambda2: " << this->lambda2 << endl;
        cout << "IKSolver.maxLinearErrorStep: " << this->maxLinearErrorStep << endl;
        cout << "IKSolver.maxAngularErrorStep: " << this->maxAngularErrorStep << endl;
        cout << "IKSolver.armijoRuleSigma: " << this->armijoRuleSigma << endl;
        cout << "IKSolver.armijoRuleBeta: " << this->armijoRuleBeta << endl;
    }
};