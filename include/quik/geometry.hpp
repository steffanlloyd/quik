/**
 * @file geometry.hpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief Defines the header code for several geometry functions in the
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
#pragma once

#include "Eigen/Dense"

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
void hgtDiff(const Matrix4d& T1, const Matrix4d& T2, Vector<double,6>& e);

/**
 * @brief Computes the inverse of a 4x4 homogenious transformation matrix
 * Much faster than actually inverting it since the computations are easy
 * The rotation portion of the transform is just transposed to invert it.
 * Then, the displacement section is just rotated and negated.
 * 
 * @param[in] T The matrix to invert (passed as reference)
 * @return Matrix4d 
 */
Matrix4d hgtInv( const Matrix4d& T );

/**
 * @brief Converts a 4x4 homogeneous transformation matrix into a 4-vector quaternion
 * and a 3-vector displacement vector
 * 
 * @param[in] T Matrix4d T, the homogeneous transformation matrix. 
 * @param[out] quat The output quaterneon (x,y,z,w)
 * @param[out] d The output displacement vector (x,y,z)
 */
void hgt2quatpos( const Matrix4d& T, Vector4d& quat, Vector3d& d);

/**
 * @brief Converts several 4x4 homogeneous transformation matrix into a matrix of 
 * 4-vector quaternions and 3-vector displacement vectors
 * 
 * @param[in] T Matrix<double,4*N,4> T, the homogeneous transformation matrix. 
 * @param[out] quat Matrix<double,4,N> The output quaterneon (x,y,z,w)
 * @param[out] d Matrix<double,3,N> The output displacement vector (x,y,z)
 */
void hgt2quatpos( const Matrix<double,Dynamic,4>& T, Matrix<double,4,Dynamic>& quat, Matrix<double,3,Dynamic>& d);

/**
 * @brief Converts a 4-vector quaternion and a 3-vector displacement vector
 * to a 4x4 homogeneous transformation matrix.
 * 
 * @param[in] quat The input quaterneon (x,y,z,w)
 * @param[in] d The input displacement vector (x,y,z)
 * @param[out] T Matrix4d T, the output homogeneous transformation matrix. 
 */
void quatpos2hgt( const Vector4d& quat, const Vector3d& d, Matrix4d& T);

/**
 * @brief Converts several 4-vector quaternions and 3-vector displacement vectors
 * to 4x4 homogeneous transformation matrices (vertically stacked)..
 * 
 * @param[in] quat Matrix<double,4,N> The input quaterneon (x,y,z,w)
 * @param[in] d Matrix<double,3,N> The input displacement vector (x,y,z)
 * @param[out] T Matrix<double,4*N,4> T, the output homogeneous transformation matrix. 
 */
void quatpos2hgt( const Matrix<double,4,Dynamic>& quat, const Matrix<double,3,Dynamic>& d, Matrix<double,Dynamic,4>& T);

/**
 * @brief Checks if a matrix is a homogeneous transformation matrix
 * 
 * @param T The matrix
 * @return bool
 */
bool isRotationMatrix(const Matrix3d &R, double tolerance = 1e-6);

/**
 * @brief Checks if a matrix is a homogeneous transformation matrix
 * 
 * @param T The matrix
 * @return bool
 */
bool ishgt(const Matrix4d &T, double tolerance = 1e-6);

} // End of namespace Geometry
} // End of namespace quik