/**
 * @file geometry.cpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief Defines the c++ source code for several geometry functions in the
 * quik::geometry namespace, such as:
 *  - quik::geometry::hgtDiff: Computes the twist error between any two homogeneous tranforms
 *  - quik::geometry::hgtInv: Computes the inverse of a homogeneous transform without inverting
 *    the matrix (for speed).
 * - quik::geometry::hgt2quatpos: converts a homogeneous transform to a 
 *   quaternion and a point.
 * - quik::geometry::quatpos2hgt: converts a quaternion and a point to a homogeneous
 *   transform.
 * - quik::geometry::isRotation: checks if a 3x3 matrix is a rotation matrix.
 * - quik::geometry::ishgt: Checks if a 4x4 matrix is a homogeneous transform.
 * 
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "quik/geometry.hpp"
#include "Eigen/Dense"
#include <iostream>

using namespace Eigen;

namespace quik{
namespace geometry{

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
void hgtDiff(const Matrix4d& T1, const Matrix4d& T2, Vector<double,6>& e)
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
 * @brief Computes the inverse of a 4x4 homogenious transformation matrix
 * Much faster than actually inverting it since the computations are easy
 * The rotation portion of the transform is just transposed to invert it.
 * Then, the displacement section is just rotated and negated.
 * 
 * @param[in] T The matrix to invert (passed as reference)
 * @return Matrix4d 
 */
Matrix4d hgtInv( const Matrix4d& T )
{
    Matrix4d Tinv;
    Tinv.topLeftCorner<3,3>() = T.topLeftCorner<3,3>().transpose();
    Tinv.topRightCorner<3,1>() = -Tinv.topLeftCorner<3,3>()*T.topRightCorner<3,1>();
    Tinv.bottomLeftCorner<1,3>().fill(0);
    Tinv(3,3) = 1;
    return Tinv;
}

/**
 * @brief Converts a 4x4 homogeneous transformation matrix into a 4-vector quaternion
 * and a 3-vector displacement vector
 * 
 * @param[in] T Matrix4d T, the homogeneous transformation matrix. 
 * @param[out] quat The output quaterneon (x,y,z,w)
 * @param[out] d The output displacement vector (x,y,z)
 */
void hgt2quatpos( const Matrix4d& T, Vector4d& quat, Vector3d& d)
{
    // Extract the rotation matrix from the homogeneous transformation matrix
    Matrix3d R = T.block<3,3>(0,0);

    // Convert the rotation matrix to a quaternion, normalize it
    Quaterniond q(R);
    q.normalize();

    // Store the quaternion in the output variable, assign the output displacement
    quat = q.coeffs();
    d = T.block<3,1>(0,3);
}

/**
 * @brief Converts several 4x4 homogeneous transformation matrix into a matrix of 
 * 4-vector quaternions and 3-vector displacement vectors
 * 
 * @param[in] T Matrix<double,4*N,4> T, the homogeneous transformation matrix. 
 * @param[out] quat Matrix<double,4,N> The output quaterneon (x,y,z,w)
 * @param[out] d Matrix<double,3,N> The output displacement vector (x,y,z)
 */
void hgt2quatpos( const Matrix<double,Dynamic,4>& T, Matrix<double,4,Dynamic>& quat, Matrix<double,3,Dynamic>& d)
{
    // Get the number of transformations
    int N = T.rows() / 4;

    if(quat.cols() != N) throw std::runtime_error("Number of columns in quat should be equal to the number of rows in T / 4.");
    if(d.cols() != N) throw std::runtime_error("Number of columns in d should be equal to the number of rows in T / 4.");

    // Iterate over each transformation
    for(int i = 0; i < N; ++i) {
        // Init and compute results
        Vector3d d_i; Vector4d quat_i;
        quik::geometry::hgt2quatpos( T.middleRows<4>(4*i), quat_i, d_i);

        // Store the results
        quat.col(i) = quat_i;
        d.col(i) = d_i;
    }	
}



/**
 * @brief Converts a 4-vector quaternion and a 3-vector displacement vector
 * to a 4x4 homogeneous transformation matrix.
 * 
 * @param[in] quat The input quaterneon (x,y,z,w)
 * @param[in] d The input displacement vector (x,y,z)
 * @param[out] T Matrix4d T, the output homogeneous transformation matrix. 
 */
void quatpos2hgt( const Vector4d& quat, const Vector3d& d, Matrix4d& T)
{
    // Convert quaternion to rotation matrix
    Quaterniond quaternion(quat(3), quat(0), quat(1), quat(2));
    Matrix3d rotation = quaternion.normalized().toRotationMatrix();

    // Assign transform
    T.block<3,3>(0,0) = rotation;
    T.block<3,1>(0,3) = d;
    T.row(3) << 0, 0, 0, 1;
}

/**
 * @brief Converts several 4-vector quaternions and 3-vector displacement vectors
 * to 4x4 homogeneous transformation matrices (vertically stacked)..
 * 
 * @param[in] quat Matrix<double,4,N> The input quaterneon (x,y,z,w)
 * @param[in] d Matrix<double,3,N> The input displacement vector (x,y,z)
 * @param[out] T Matrix<double,4*N,4> T, the output homogeneous transformation matrix. 
 */
void quatpos2hgt( const Matrix<double,4,Dynamic>& quat, const Matrix<double,3,Dynamic>& d, Matrix<double,Dynamic,4>& T)
{
    int N = quat.cols();

    if(T.rows()/4 != N) throw std::runtime_error("Number of columns in quat should be equal to the number of rows in T / 4.");
    if(d.cols() != N) throw std::runtime_error("Number of columns in d should be equal to the number of rows in T / 4.");

    for(int i = 0; i < N; ++i) {
        Matrix4d T_i;

        quik::geometry::quatpos2hgt( quat.col(i), d.col(i), T_i);
        T.middleRows<4>(4*i) = T_i;
    }
}

/**
 * @brief Checks if a matrix is a homogeneous transformation matrix
 * 
 * @param T The matrix
 * @return bool
 */
bool isRotationMatrix(const Matrix3d &R, double tolerance) {

    // Check if is orthogonal (R*R_transpose = I)
    if (! (R * R.transpose()).isApprox(Matrix3d::Identity(), tolerance)) return false;

    // Check if rotation part has determinant 1 (right-hand rule system)
    if (std::abs(R.determinant() - 1.0) > 1e-6) return false;

    // Passed all checks, return true
    return true;
}

/**
 * @brief Checks if a matrix is a homogeneous transformation matrix
 * 
 * @param T The matrix
 * @return bool
 */
bool ishgt(const Matrix4d &T, double tolerance) {
    // Check if last row is [0, 0, 0, 1]
    if (!T.row(3).transpose().isApprox(Vector4d(0, 0, 0, 1), tolerance)) return false;

    // Check that rotation part is rotation matrix
    return quik::geometry::isRotationMatrix(T.block<3,3>(0,0));
}

} // End of namespace geometry
} // End of namespace quik