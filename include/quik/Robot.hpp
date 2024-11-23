/**
 * @file Robot.hpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief Defines the quik::Robot class, which stores the robot structure and can perform
 * various forward and velocity kinematic computations with very high efficiency.
 * Properties:
 *       - Matrix<double,DOF,4> DH: A DOFx4 array of the DH params in the following order:
 *          [a_1  alpha_1    d_1   theta_1;
 *           :       :        :       : 
 *           an   alpha_n    d_n   theta_n ];
 *
 *       - Vector<bool,DOF> linkTypes: A vector of link types. Should be true if joint is a
 *         prismatic joint, false otherwise.
 *
 *       - Vector<double,6> Qsign: A vector of link direction (-1 or 1). Allows you to
 *         change the sign of the joint variable.
 *
 *       - Matrix4d Tbase: The base transform of the robot (between world frame and first
 *         DH frame)
 *
 *       - Matrix4d Ttool: The tool transform of the robot (between DOF'th frame and tool
 *        frame)
 *
 * Methods:
 *
 *     - void FK( const MatrixQ& Q, MatrixTi& T) const;
 *       Computes the forward kinematics of the manipulator for a given set of joint angles.
 *       Result is stored in a Matrix<double,DOF*4,4> matrix, with each frame transform
 *       stacked vertically ontop of each other.
 *
 *     - void FKn( const MatrixQ& Q, Matrix4d& T) const;
 *       Computes the forward kinematics for a given set of joint angles, but only returns
 *       the final frame as a Matrix<double,4,4>.
 *
 *     - void jacobian( MatrixTi& T, MatrixJ& J) const;
 *       Computes the geometric jacobian of the manipulator, based on the stacked forward
 *       kinematics transforms given by the FK function.
 *
 *     - void hessianProduct( const MatrixJ& J, const MatrixQ& dQ, MatrixJ& A) const
 *       Computes the product H*dQ, where H is the geometric hessian of the robot. This
 *       function never computes H explicitely, and instead just reduces the computed values
 *	     through multiplication as they are computed. The result is added to A.
 *       Note, A must be pre-initialized with 0's before calling this function!
 * 
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "Eigen/Dense"
#include "quik/geometry.hpp"
#include "quik/utilities.hpp"

using namespace Eigen;
using namespace std;

namespace quik{

enum JOINTTYPE_t : bool {
	JOINT_REVOLUTE = false,
	JOINT_PRISMATIC = true
};

inline std::string jointtype2str(quik::JOINTTYPE_t jointType) {
    switch(jointType) {
        case quik::JOINT_REVOLUTE: return "JOINT_REVOLUTE";
        case quik::JOINT_PRISMATIC: return "JOINT_PRISMATIC";
        default: return "UNKNOWN_JOINTTYPE";
    }
};

inline quik::JOINTTYPE_t str2jointtype(std::string jointType) {
    if (jointType == "JOINT_REVOLUTE") return quik::JOINT_REVOLUTE;
    else if (jointType == "JOINT_PRISMATIC") return quik::JOINT_PRISMATIC;
    else throw std::runtime_error("Invalid JOINTTYPE string");
};

template<int DOF=Dynamic>
class Robot {
public:

	// @brief	Matrix<double,DOF,4> DH: A DOFx4 array of the DH params in the following order:
	//         [a_1  alpha_1    d_1   theta_1;
	//          :       :        :       :    
	//          an   alpha_n    d_n   theta_n ];
	//
    Array<double,DOF,4> DH;

	// @brief Vector<JOINTTYPE_t,DOF> linkTypes: A vector of link types. Specify JOINT_REVOLUTE
	// or JOINT_PRISMATIC
	Vector<quik::JOINTTYPE_t,DOF> linkTypes;
    
	// @brief Vector<double,6> Qsign: A vector of link direction (-1 or 1). Allows you to
	// change the sign of the joint variable.
	Vector<double,DOF> Qsign;

	// @brief Matrix4d Tbase: Base transform (between the first and world frame).
	Matrix4d Tbase;
	
	// @brief Matrix4d Ttool: The tool transform of the robot (between DOF'th frame and tool frame)
	Matrix4d Ttool;
    
	// The number of degrees of freedom. Assigned automatically.
	int dof;
	
	/**
	 * @brief Construct a new Robot object
	 * 
	 * @param _DH The DH Table
	 * @param _linkTypes Link types
	 * @param _Qsign Joint directions
	 * @param _Tbase Base transform (defaults to identity transform)
	 * @param _Ttool The tool transform (defaults to identity transform)
	 */
	Robot(
		Array<double,DOF,4> _DH,
		Vector<quik::JOINTTYPE_t,DOF> _linkTypes,
    	Vector<double,DOF> _Qsign = (DOF == -1 ? VectorXd::Ones(0) : VectorXd::Ones(DOF)),
		Matrix4d _Tbase = Matrix4d::Identity(4,4),
		Matrix4d _Ttool = Matrix4d::Identity(4,4))
		: DH(_DH), linkTypes(_linkTypes), Qsign(_Qsign), Tbase(_Tbase), Ttool(_Ttool)
	{
		this->dof = (int) this->DH.rows();

		// Handle case where DOF is Dynamic, and Qsign isn't given
		if(this->Qsign.size()==0) this->Qsign = VectorXd::Ones(this->dof);

		if(this->DH.cols() != 4) throw std::runtime_error("DH must be a DOFx4 matrix");
		if(this->linkTypes.size() != this->dof) throw std::runtime_error("linkTypes must be the same size as the DH matrix has rows.");
		if(this->Qsign.size() != this->dof) throw std::runtime_error("Qsign must be the same size as the DH matrix has rows.");
		if(!geometry::ishgt(this->Ttool)) throw std::runtime_error("Ttool must be a proper homogeneous transformation matrix");
		if(!geometry::ishgt(this->Tbase)) throw std::runtime_error("Tbase must be a proper homogeneous transformation matrix");
	}
	
	/**
	 * @brief Prints out the kinematic details of the robot
	 */
	void print() const
	{
		cout << "R.DH:" << endl << this->DH.format(quik::utilities::CleanFmt) << endl;
		cout << "R.Tbase: " << endl << this->Tbase.format(quik::utilities::CleanFmt) << endl;
		cout << "R.Ttool: " << endl << this->Ttool.format(quik::utilities::CleanFmt) << endl;
		cout << "R.linkTypes: [";
		for(int i = 0; i < this->linkTypes.size(); i++){
			if (i!=0) cout << ", ";
			if(this->linkTypes(i)) cout << "PRISMATIC";
			else cout << "REVOLUTE";
		}
		cout << "]" << endl;
		cout << "R.Qsign: " << this->Qsign.transpose() << endl;
		cout << "R.dof: " << this->dof << endl << endl << endl;
	}
    
	/**
	 * @brief Computes the forward kinematics of the manipulator for a given set of joint angles.
	 * Result is stored in a Matrix<double,DOF*4,4> matrix, with each frame transform
	 * stacked vertically ontop of each other.
	 * 
	 * INPUTS:
	 * @param Q The joint variables to compute the forward kinematics at.
	 * 
	 * OUTPUTS:
	 * @param T The ouput transformations.
	 */
    void FK( const Vector<double,DOF>& Q, Matrix<double,(DOF>0?4*(DOF+1):-1),4>& T) const
	{
		// Initialize data
		Array<double,1,4> DH_k;
		Matrix4d Ak;
		double stk, ctk, sak, cak, ak, dk;
			
		// Iterate over joints
		for (int k=0; k<this->dof; k++){
			
			// Get DH variables for row
			DH_k = this->DH.row(k);
			if (this->linkTypes(k)) DH_k(2) += Q(k) * this->Qsign(k);
			else DH_k(3) += Q(k) * this->Qsign(k);
			
			// break out sin and cos
			stk = sin(DH_k(3));
			ctk = cos(DH_k(3));
			sak = sin(DH_k(1));
			cak = cos(DH_k(1));
			ak = DH_k(0);
			dk = DH_k(2);
			
			// Assign frame transform
			Ak << 	ctk, 	-stk*cak, 	stk*sak, 	ak*ctk,
					stk, 	ctk*cak, 	-ctk*sak, 	ak*stk,
					0,		sak,		cak,		dk,
					0, 		0,			0,			1;
			
			// Collect transforms
			if (k==0){
				T.template middleRows<4>(0) = this->Tbase * Ak;
			}else{
				T.template middleRows<4>(4*k) = T.template middleRows<4>(4*(k-1)) * Ak;
			}
				
		} // End of joint for loop
		
		// Apply tool transform
		T.template bottomRows<4>() = ((T.template middleRows<4>(4*(this->dof-1))) * this->Ttool).eval();
		
	} // End of FK function
	

	/**
	 * @brief Computes the forward kinematics for a given set of joint angles, but only returns
	 * the final frame as a Matrix<double,4,4>. Note, this just calls FK internally.
	 * 
	 * @param[in] Q The input joint angles
	 * @param[out] T The tranformation matrices for the final link
	 */
	void FKn(const Vector<double,DOF> &Q, Matrix4d& Tn, int frame=-1) const
	{
		constexpr int DOF4 = (DOF>0) ? (DOF+1)*4 : -1;
		Matrix<double,DOF4,4> Ti((this->dof+1)*4,4);
		this->FK( Q, Ti );

		// If frame is -1 (default), return tool frame
		if(frame==-1) frame = this->dof+1;

		// Validate input
		if(frame < 1 || frame > this->dof+1) {
			throw std::runtime_error("Requested frame must be -1 (for tool frame) or an integer between 1 and DOF+1");
		}

		Tn = Ti.template middleRows<4>((frame-1)*4);
	}

	/**
	 * @brief Computes the forward kinematics for a given set of joint angles, and returns the result
	 * as a quaternion and a point
	 * 
	 * @param[in] Q The input joint angles
	 * @param[out] quat Vector4d The quaternion out
	 * @param[out] d Vector3d the point out
	 * @param[in] frame=-1 int The frame to compute the transform for. Put a negative value for the tool
	 * frame.
	 */
	void FKn(const Vector<double,DOF> &Q, Vector4d& quat, Vector3d& d, int frame=-1) const
	{
		// Call forward kinematics to a transform
		Matrix4d T;
		this->FKn(Q, T, frame);

		// Convert to a quaternion and point
        quik::geometry::hgt2quatpos(T, quat, d);
	}

	/**
	 * @brief Computes the geometric jacobian of the manipulator, based on joint angles.
	 * 
	 * If you already have computed the forward kinematics, this function is inneficient. Use the second
	 * calling syntax below!
	 * 
	 * @param[in] Q The joint variables for the robot
	 * @param[out] J The jacobian matrix
	 * @param[in] includeTool  Whether or not to include the tool transform in the calculations
	 * Default: true.
	 */
	void jacobian( const Vector<double, DOF>& Q, Matrix<double, 6, DOF>& J, bool includeTool = true) const
	{
		// Initialize the outputs
        constexpr int DOF4 = DOF>0 ? (DOF+1)*4 : -1;
        Matrix<double,DOF4,4> T((this->dof+1)*4, 4); // Holds the forward kinematics for each joint

		// Update T with forward kinematics
		this->FK( Q, T );
		
		// Get jacobian, store it in J
		this->jacobian(T, J, includeTool);
	}
	
	/**
	 * @brief Computes the geometric jacobian of the manipulator, based on the stacked forward
	 * kinematics transforms given by the FK function.
	 * 
	 * If you've already computed the forward kinematics, this function is much more efficent
	 * that the calling syntax based on joint angles, since it needs to just call the forward
	 * kinematics first anyway.
	 * 
	 * @param[in] T The forward transformation matrices for all the links of the robot
	 * As computed with the FK function
	 * @param[out] J The Matrix J, passed as reference, in which to store the results.
	 * @param[in] includeTool Whether or not to include the tool transform in the calculations
	 * Default: true.
	 */
	void jacobian( const Matrix<double,(DOF>0?4*(DOF+1):-1),4>& T, Matrix<double,6,DOF>& J, bool includeTool = true) const
	{
		// Initialize some variables
		Vector3d z_im1, o_im1, o_n;
		
		// Get position of end effector
		o_n = T.template block<3,1>( 4*(this->dof-1+int(includeTool)), 3);
		// o_n = T.template block<3,1>( 4*(this->dof-1), 3);

		// Loop through joints
		for (int i = 0; i < this->dof; i++){
			
			// Get z_{i-1}, o_{i-1}
			// z_{i-1} is the unit vector along the z-axis of the previous joint
			// o_{i-1} is the position of the center of the previous joint frame
			if (i > 0){
				z_im1 = T.template block<3,1>(4*(i-1), 2);
				o_im1 = T.template block<3,1>(4*(i-1), 3);
			}else{
				z_im1 = this->Tbase.block<3,1>(0,2);
				o_im1 = this->Tbase.block<3,1>(0,3);
			}
			
			// Assign the appropriate blocks of the Jacobian
			// These formulas are from Spong.
			if (this->linkTypes(i)){
				// Prismatic joint
				J.template block<3,1>(0, i) = z_im1;
				J.template block<3,1>(3, i).fill(0);
			}else{
				// Revolute joint
				J.template block<3,1>(0, i) = z_im1.cross( o_n - o_im1 );
				J.template block<3,1>(3, i) = z_im1;
			}
		} // end of joint for loop

	} // end of jacobian()
	
	/**
	 * @brief void hessianProduct( const MatrixJ& J, const MatrixQ& dQ, MatrixJ& A) const
	 * Computes the product H*dQ, where H is the geometric hessian of the robot. This
	 * function never computes H explicitely, and instead just reduces the computed values
	 * through multiplication as they are computed. The result is added to A.
	 * Note, A must be pre-initialized with 0's before calling this function!
	 * 
	 * @param[in] J The Jacobian J
	 * @param[in] dQ The delta joint variables vector
	 * @param[out] A The resulting Hessian product, to store values into (will be modified).
	 */
	void hessianProduct( const Matrix<double,6,DOF>& J, const Vector<double,DOF>& dQ, Matrix<double,6,DOF>& A) const
	{
		Vector3d cp, jvk, jwk, Aw_k_sum;

		// Uncomment this line if A will not contain anything. 
		// If A is unititialized, then this function will not give meaningful answers.
		// However, usually this function is called with A already initialized 
		// So it is just a waste of time to fill it with zeros.
		// A.fill(0); 
		
		if (!this->linkTypes.any()){
			// Special code for revolute joint robots only (most common, can
			// skip branched coding which is slow)
			
			for (int k = 0; k < this->dof; k++){
				// First, iterate over off-diagonal terms
				jvk = J.template block<3,1>(0,k);
				jwk = J.template block<3,1>(3,k);
				
				// Rotational terms can be simplified by avoiding half the cross products
				// Initiate summer, then sum in loop before doing cross product.
				Aw_k_sum << 0,0,0;
				
				for (int i = 0; i < k; i++){
					// A(4:6, k) += jwi x jwk * dQi
					//           += (jwi * dQi) x jwk
					// Can sum the first term in cross product first before computing cross product.
					Aw_k_sum += J.template block<3,1>(3,i) * dQ(i);
					
					// A(1:3, k) += jwi x jvk*dQi
					cp = J.template block<3,1>(3, i).cross( jvk );
					A.template block<3,1>(0, k) += cp * dQ(i);
					A.template block<3,1>(0, i) += cp * dQ(k); // Symmetry
				}
				
				// Do final cross product for rotational term
				A.template block<3,1>(3, k) += Aw_k_sum.cross( jwk );
				
				// For diagonal entries, can skip the omega term since jwk x jwk = 0
				// A(4:6, k) += jwi x jwk * dQi = 0
				//
				// A(1:3, k) += jwi x jvk*dQi
				A.template block<3,1>(0, k) += jwk.cross( jvk ) * dQ(k);
			}
			
		}else{
			// General case
			
			for (int k = 0; k < this->dof; k++){
				// First, iterate over off-diagonal terms
				jvk = J.template block<3,1>(0,k);
				jwk = J.template block<3,1>(3,k);
				
				for (int i = 0; i < k; i++){
					if (! this->linkTypes(i)){ // link i is revolute
						if (! this->linkTypes(k)){	// link k is revolute
							// A(4:6, k) += jwi x jwk * dQi
							A.template block<3,1>(3, k) += (J.template block<3,1>(3, i)).cross( jwk ) * dQ(i);
						}
						
						// A(1:3, k) += jwi x jvk*dQi
						cp = J.template block<3,1>(3, i).cross( jvk );
						A.template block<3,1>(0, k) += cp * dQ(i);
						A.template block<3,1>(0, i) += cp * dQ(k); // Symmetry
					}
					
					if (! this->linkTypes(k)){
						// For diagonal entries, can skip the omega term since jwk x jwk = 0
						// A(4:6, k) += jwi x jwk * dQi = 0
						//
						// A(1:3, k) += jwi x jvk*dQi
						A.template block<3,1>(0, k) += jwk.cross( jvk ) * dQ(k);
					}
				} // end of inner for loop
			} // end of outer for loop
		} // end of branched if
	} // end of hessianProduct()

	/**
	 * @brief Returns a length that is characteristic of the robot.
	 * Found by summing the pythagorian distance of each link
	 * e.g. sum( sqrt(a_i^2 + d_i^2 ))
	 * 
	 * This quantity can be useful for ensuring that inverse kinematics quantities
	 * are appropriately scaled.
	 * 
	 * @return double 
	 */
	double characteristicLength() const
	{
		Vector<double,DOF> a = DH.col(0);
		Vector<double,DOF> d = DH.col(2);
		return (a.array().square() + d.array().square()).sqrt().sum();
	}

}; // End of class definition quik::Robot

} // End of namespace quik