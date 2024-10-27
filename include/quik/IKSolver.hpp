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

#include "Eigen/Dense"
#include "quik/Robot.hpp"

using namespace Eigen;

class IKSolver {
public:

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
    
    // @brief iterMax [int]: Maximum number of iterations of the algorithm. Default: 100
	int iterMax;
    
    // @brief algorithm [ALGORITHM_t]: The algorithm to use
    //     - ALGORITHM_QUIK - QuIK
    //     - ALGORITHM_NR - Newton-Raphson or Levenberg-Marquardt
    //     - ALGORITHM_BFGS - BFGS
    //     - Default: 0.
	IKSolver::ALGORITHM_t algorithm;

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
        : iterMax(_iterMax),
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
     * @param[in] R Robot& R: A Robot object to perform the IK on.
     * @param[in] Twt Matrix<double,4*N,4>& Twt: Tall matrix of all the N desired robot poses
     * vertically stacked.
     * @param[in] Q0 Matrix<double,DOF,N>& Q0: Initial guesses of the joint angles
     * @param[out] Q_star  Matrix<double,DOF,N>& Qstar: [DOFxN] The solved joint angles
     * @param[out] e_star Matrix<double,6,N>&e: The pose errors at the final solution.
     * @param[out] iter Vector<int,N> iter: The number of iterations the algorithm took.
     * @param[out] breakReason Vector<BREAKREASON_t,N> breakReason: The reason the algorithm stopped.
     *          See BREAKREASON_t for list of reasons.
     */
    template<int DOF=Dynamic>
    void IK(
        const Robot<DOF>& R,
		const Matrix<double,Dynamic,4>& Twt,
		const Matrix<double,DOF,Dynamic>& Q0,
		Matrix<double,DOF,Dynamic>& Q_star,
		Matrix<double,6,Dynamic>& e_star,
		VectorXi& iter,
		VectorXi& breakReason) const
    {
        // Get size of problem
        int N = (int) Q0.cols();
        
        // Define function variables
        Vector<double,DOF> Q_i, dQ_i, s0;
        constexpr int DOF4 = DOF>0 ? (DOF+1)*4 : -1;
        Matrix<double,DOF4,4> T_i((R.dof+1)*4, 4);
        Matrix4d Twt_i;
        Vector<double,6> e_i, Hg_i;
        Vector<double,DOF>  grad_i, grad_ip1, y;
        Matrix<double,6,DOF> J_i(6, R.dof), A_i;
        Matrix<double,DOF,DOF> H_i;
        int 	iter_i,
                breakReason_i,
                grad_fail_counter,
                grad_fail_counter_total;
        double 	e_i_norm = 0,
                e_i_prev_norm,
                error_relImprovement = 0,
                cost_i = 1e10,
                cost_ip1 = 1e10,
                gamma,
                rho,
                delta;
        

        // Start iterations over poses to solve
        for (int i = 0; i < N; i++){
        
            // Start solver
            Q_i = Q0.col(i);
            e_i.fill(0);
            dQ_i.fill(0);
            iter_i = this->iterMax;
            breakReason_i = IKSolver::BREAKREASON_MAX_ITER; // Initialize to this, it will be overwritten if it doesn't reach max iter
            e_i_prev_norm = 1e10;
            grad_fail_counter = 0;
            grad_fail_counter_total = 0;
            Twt_i = Twt.middleRows<4>(4*i);
            
            // Start IK iterations
            for (int i = 0; i < this->iterMax; i++){
                
                // Get error, forward kinematics and jacobian
                // Only do this for Newton and QuIK, or on first iteration
                if (this->algorithm != IKSolver::ALGORITHM_BFGS || i == 0){
                    // Forward kinematics
                    R.FK( Q_i, T_i );
                    
                    // Get jacobian (needed for all algorithms)
                    R.jacobian(T_i, J_i, true);
                    
                    // Get error
                    this->hgtDiff( T_i.template bottomRows<4>(), Twt_i, e_i );
                }
                
                // Calculate norm
                e_i_norm = e_i.norm();

                // Break, if exit tolerance has been reached
                if (e_i_norm < this->exitTol){
                    breakReason_i = IKSolver::BREAKREASON_TOLERANCE; // Tolerance reached
                    iter_i = i;
                    break;
                }
                
                // Check relative improvement in error
                // We break if the relative improvement fails this->maxGradFails times in a row, or if
                // it fails this->maxGradFailsTotal total
                error_relImprovement = (e_i_prev_norm - e_i_norm) / e_i_prev_norm;
                if (error_relImprovement < this->relImprovementTol){
                    // If relative improvement is below threshold, increment counters
                    grad_fail_counter++;
                    grad_fail_counter_total++;
                    if (grad_fail_counter > this->maxGradFails) {
                        breakReason_i = IKSolver::BREAKREASON_GRAD_FAILS; // Grad consecutive fails reached
                        iter_i = i;
                        break;
                    }
                    if (grad_fail_counter_total > this->maxGradFailsTotal) {
                        breakReason_i = IKSolver::BREAKREASON_GRAD_FAILS; // Grad fails reached
                        iter_i = i;
                        break;
                    }
                }else{
                    grad_fail_counter = 1;
                }

                // Store prev value
                e_i_prev_norm = e_i_norm;
                
                // Clamp error
                this->clampMag(e_i);
                
                // Go to switch statement to do work of each individual algorithm
                switch (this->algorithm){
                        
                        
                    case IKSolver::ALGORITHM_QUIK:
                        // Halley's method (QuIK Method)
                        
                        // First, store the newton step in dQ_i (note, it's negative)
                        this->lsolve<6>( J_i, e_i, dQ_i);
                        
                        // Then, negate it and divide by two
                        dQ_i *= -0.5;
                        
                        // Assign jacobian to A_i so that it gets added to it
                        A_i = J_i;

                        // Get gradient product
                        R.hessianProduct( J_i, dQ_i, A_i );
                                            
                        // Resolve
                        this->lsolve<6>(A_i, e_i, dQ_i);
                        dQ_i *= -1;
                        
                        break;
                        
                        
                        
                    case IKSolver::ALGORITHM_NR:
                        // Newton's method
                        this->lsolve<6>( J_i, e_i, dQ_i);
                        dQ_i *= -1;
                        break;
                        
                        
                        
                    case IKSolver::ALGORITHM_BFGS:
                        // BFGS
                        // On first iteration, initialize some variables
                        if (i == 0){
                            H_i = Matrix<double,DOF,DOF>::Identity(R.dof,R.dof);
                            grad_i = J_i.transpose() * e_i;
                            cost_i = 0.5*e_i.array().square().sum();
                        }
                        
                        // Get initial step
                        s0 = -H_i*grad_i;
                        
                        // Initialize line search
                        gamma = 1;
                        
                        // Recalculate cost and error
                        R.FK( Q_i + gamma*s0, T_i );
                        this->hgtDiff( T_i.template bottomRows<4>(), Twt_i, e_i );
                        cost_ip1 = 0.5*e_i.array().square().sum();
                        
                        // Do line search
                        while ((cost_i - cost_ip1) < -this->armijoRuleSigma * grad_i.transpose()*(gamma*s0)){
                            // Reduce gamma
                            gamma = this->armijoRuleBeta * gamma;
                            
                            // Break if step size is too small (prevents infinite loops too)
                            if (gamma < this->minStepSize) break;
                            
                            // Recalculate cost
                            R.FK( Q_i + gamma*s0, T_i );
                            this->hgtDiff( T_i.template bottomRows<4>(), Twt_i, e_i );
                            cost_ip1 = 0.5*e_i.array().square().sum();
                        }
                        
                        // Break out if step size is too small
                        if (gamma < this->minStepSize){
                            breakReason_i = IKSolver::BREAKREASON_MIN_STEP; // reached minimum step size
                            iter_i = i;
                            break;
                        }
                        
                        // Take step
                        dQ_i = gamma*s0;
                        
                        // Update gradient (T_i and e_i are already updated)
                        R.jacobian(T_i, J_i);
                        grad_ip1 = J_i.transpose() * e_i;
                        
                        // Update gradient
                        y = grad_ip1 - grad_i;
                        rho = dQ_i.transpose() * y;
                        delta = y.transpose() * H_i * y;
                        if (rho > delta && rho > numeric_limits<double>::epsilon())
                            H_i = H_i + ( (1 + delta/rho) * dQ_i*dQ_i.transpose() - dQ_i*y.transpose()*H_i - H_i*y*dQ_i.transpose())/rho;
                        else if (delta > numeric_limits<double>::epsilon() && rho > numeric_limits<double>::epsilon())
                            H_i = H_i + (dQ_i*dQ_i.transpose())/rho - H_i*(y*y.transpose())*H_i/delta;
                        
                        // Update variables for next time
                        grad_i = grad_ip1;
                        cost_i = cost_ip1;
                        
                        break;

                        
                    default:
                        // invalid input
                        cout << "Invalid algorithm specified!" << endl;
                        dQ_i.fill(0);
                        
                        
                } // end of algorithm switch statement
                            
                // Apply change
                Q_i += dQ_i;
                
                // Check grad tolerance, break if necessary
                if (dQ_i.array().square().sum() < this->minStepSize * this->minStepSize){
                    breakReason_i = IKSolver::BREAKREASON_MIN_STEP; // minimum step sized reached
                    iter_i = i;
                    break;
                }
                
            } // End of IK loop
            
            // Store solutions
            Q_star.col(i) = Q_i;
            e_star.col(i) = e_i;
            iter(i) = iter_i;
            breakReason(i) = breakReason_i;
            
        } // End of sample loop
        
    } // End of IK()

    /**
     * @brief Computes the inverse of a 4x4 homogenious transformation matrix
     * Much faster than actually inverting it since the computations are easy
     * The rotation portion of the transform is just transposed to invert it.
     * Then, the displacement section is just rotated and negated.
     * 
     * @param[in] T The matrix to invert (passed as reference)
     * @return Matrix4d 
     */
    static Matrix4d hgtInv( const Matrix4d& T )
    {
        Matrix4d Tinv;
        Tinv.topLeftCorner<3,3>() = T.topLeftCorner<3,3>().transpose();
        Tinv.topRightCorner<3,1>() = -Tinv.topLeftCorner<3,3>()*T.topRightCorner<3,1>();
        Tinv.bottomLeftCorner<1,3>().fill(0);
        Tinv(3,3) = 1;
        return Tinv;
    }

    /**
     * @brief Calculates the error between two homogeneous transforms.
     * 
     *  Algorithm used is as described in
     *  [1] T. Sugihara, “Solvability-Unconcerned Inverse Kinematics
     *  by the Levenberg–Marquardt Method,” IEEE Trans. Robot.,
     *  vol. 27, no. 5, pp. 984–991, Oct. 2011.
     * 
     * @param[in] T1 The first transform
     * @param[in] T2 The second transform
     * @param[out] e The error (passed as reference and transformed)
     */
    static void hgtDiff(const Matrix4d& T1, const Matrix4d& T2, Vector<double,6>& e)
    {
        Matrix3d R1, R2, Re;
        Vector3d d1, d2, eps;
        double eps_norm, t;
        
        // Break out values
        R1 = T1.topLeftCorner<3,3>();
        R2 = T2.topLeftCorner<3,3>();
        d1 = T1.topRightCorner<3,1>();
        d2 = T2.topRightCorner<3,1>();
        
        // Orientation error
        Re = R1*R2.transpose();
        
        // Assign linear error
        e.head<3>() = d1 - d2;
        
        // Extract diagonal and trace
        t = Re.trace();
        
        // Build l variable, and calculate norm
        eps <<	Re(2,1)-Re(1,2),
                Re(0,2)-Re(2,0),
                Re(1,0)-Re(0,1);
        eps_norm = eps.norm();

        // Different behaviour if rotations are near pi or not.
        if (t > -.99 || eps_norm > 1e-10){
            // Matrix is normal or near zero (not near pi)
            // If the eps_norm is small, then the first-order taylor
            // expansion results in no error at all
            if (eps_norm < 1e-3){
                // atan2( eps_norm, t - 1 ) / eps_norm ~= 0.5 - (t-3)/12
                // Should have zero machine precision error when eps_norm < 1e-3.
                //
                // w ~= theta/(2*sin(theta)) = acos((t-1)/2)/(2*sin(theta))
                //
                // taylor expansion of theta/(2*theta) ~= 1/2 + theta^2/12 (3rd
                // order)
                // taylor expansion of (acos(t-1)/2)^2 is (3-t) (2nd order).
                //
                // Subtituting:
                // w ~= (1/2 + (3-t)/12) * eps = (0.75 - t/12)*eps.
                e.tail<3>() = (0.75 - t/12) * eps;
            }else{
                // Just use normal formula
                e.tail<3>() = (atan2(eps_norm, t - 1) / eps_norm) * eps;
            }
        }else{
            // If we get here, the trace is either nearly -1, and the error is
            // close to zero.
            // This combination is only possible if R is nearly a rotation of pi
            // radians about the x, y, or z axes.
            //
            // Since at this point, any rotation vector will do since we could
            // rotate in any direction. However, we use the approximation below.
            e.tail<3>() = 1.570796326794897 * (Re.diagonal().array() + 1);
            
        } // End of if statements handling near-singular poses
    } // End of hgtDiff()
    
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
    template<int DOF=Dynamic>
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