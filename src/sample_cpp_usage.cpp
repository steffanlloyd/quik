/**
 * @file sample_cpp_usage.cpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief Demo code on how to define a robot, and call the forward and inverse kinematics functions
 * in pure C++ (outside of ros). This code can be compiled in ROS but does not depend on it in
 * any way.
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <chrono>
#include "Eigen/Dense"
#include <math.h>
#include "quik/geometry.hpp"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"

using namespace std;
using namespace Eigen;
using namespace quik;

// Define manipulator.
// This is the DH parameters for the KUKA KR6 robot
auto R = std::make_shared<Robot<6>>(
	// Given as DOFx4 table, in the following order: a_i, alpha_i, d_i, theta_i.
	(Matrix<double, 6, 4>() <<
		0.025,    -M_PI/2,   0.183,       0,
		-0.315,   0,         0,           0,
		-0.035,   M_PI/2,    0,           0,
		0,        -M_PI/2,   0.365,       0,
		0,        M_PI/2,    0,           0,
		0,        0,         0.08,        0).finished(),
					  
	// Second argument is a list of joint types
	// true is prismatic, false is revolute
	// KUKA KR6 only has revolute joints
	(Vector<JointType_t,6>() << 
        JOINT_REVOLUTE, JOINT_REVOLUTE, JOINT_REVOLUTE, JOINT_REVOLUTE, JOINT_REVOLUTE, JOINT_REVOLUTE
    ).finished(),

	// Third agument is a list of joint directions
	// Allows you to change the sign (direction) of
	// the joints.
	(Vector<double,6>(6) << 1, 1, 1, 1, 1, 1).finished(),

	// Fourth and fifth arguments are the base and tool transforms, respectively
	Matrix4d::Identity(4,4),
	Matrix4d::Identity(4,4)
);

constexpr int m = 6;
auto C = std::make_shared<WorldConstraint<m>>(WorldConstraint<m>::Partial({0,1,2,3,4,5}));

// Define the IK options
const IKSolver<6,m> IKS(
    R, // The robot object (pointer)
	C, // The constraint object (pointer)
    200, // max number of iterations
    ALGORITHM_QUIK, // algorithm (ALGORITHM_QUIK, ALGORITHM_NR)
    1e-12, // Exit tolerance
    1e-14, // Minimum step tolerance
    0.05, // iteration-to-iteration improvement tolerance (0.05 = 5% relative improvement)
    10, // max consequitive gradient fails
    80, // Max gradient fails
    1e-10, // lambda2 (lambda^2, the damping parameter for DQuIK and DNR)
    0.34, // Max linear error step
    1 // Max angular error step
);

int main()
{
	
	// Initilize variables
	int N = 5; // Number of poses to generate
	int DOF = R->dof;
	JointStateArray_t<6, Dynamic> 	Q(DOF, N),	    // True joint angles
                                	Q0(DOF, N),	    // Initial guess of joint angles
                                	Q_star(DOF, N);	// Solver's solution
	TwistArray_t<Dynamic>   		e_star(6,N);	// Error at solver pose
    std::vector<int>            	iter(N);	    // Store number of iterations of algorithm
	std::vector<BreakReason_t>  	breakReason(N);	// Store break out reason
	Hgt_t                    		T,		        // True forward kinematics transform
		                        	T_star;	        // Forward kinematics at solver solution
	
	HgtArray_t<Dynamic>    			Tn(N*4,4); 	    // 4N * 4 matrix of vertically stacked transforms to be solved.
							                   		// This is just a convenient way of sending in an array of transforms.
	
	// Generate some random joint configurations for the robot
	Q.setRandom(DOF, N);

	// Perturb true answers slightly to get initial "guess" (knock over by 0.1 radians)
	Q0 = Q.array() + 0.1;
		
	// Do forward kinematics of each Q sample and store in the "tall" matrix
	// of transforms, Tn
	for (int i = 0; i < N; i++){
		R->FKn( Q.col(i), T );
		Tn.middleRows<4>(i*4) = T;
	}

	R->print();

	// Start a timer, to get time
	auto startTime = chrono::high_resolution_clock::now();

	// Solve using the IK function, store results in Q_star, e_star, iter and breakreason
	IKS.IK(Tn, Q0, Q_star, e_star, iter, breakReason);

	// Get the time after calling the IK function
    chrono::duration<double, std::micro> elapsed = chrono::high_resolution_clock::now() - startTime;
	
	// Print out results
	cout << "Joint angles (start):" << endl;
	cout << Q0 << endl << endl;
	cout << "Joint angles (true):" << endl;
	cout << Q << endl << endl;
	cout << "The final joint angles are: " << endl;
	cout << Q_star << endl << endl;
	cout << "The errors are:" << endl;
	cout << e_star << endl << endl;
	cout << "Final normed error is: " << endl << e_star.array().square().colwise().sum().sqrt() << endl << endl;
	cout << "Break reason is: " << endl;
    for (const auto& reason : breakReason) cout << utilities::breakreason2str(reason) << ' ';
    cout << endl;
	cout << "Number of iterations: " << endl;
    for (const auto& iter_i : iter) cout << iter_i << ' ';
    cout << endl;
	cout << "Time elapsed for IK function: " << elapsed.count() << " us, " << elapsed.count()/N << " us per sample." << endl;
	cout << "Program finished!" << endl;
	
	return 0;
}