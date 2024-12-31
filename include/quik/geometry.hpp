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
#include "quik/types.hpp"

using namespace Eigen;
using namespace quik;

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
 * @param[in] T1 Hgt_t The first transform
 * @param[in] T2 Hgt_t The second transform
 * @param[out] e Twist_t The error (passed as reference and transformed)
 */
void hgtDiff(const Hgt_t& T1, const Hgt_t& T2, Twist_t& e);

/**
 * @brief Computes the inverse of a 4x4 homogenious transformation matrix
 * Much faster than actually inverting it since the computations are easy
 * The rotation portion of the transform is just transposed to invert it.
 * Then, the displacement section is just rotated and negated.
 * 
 * @param[in] T Hgt_t The matrix to invert (passed as reference)
 * @return Hgt_t 
 */
Hgt_t hgtInv( const Hgt_t& T );

/**
 * @brief Converts a 4x4 homogeneous transformation matrix into a 4-vector quaternion
 * and a 3-vector displacement vector
 * 
 * @param[in] T Hgt_t T, the homogeneous transformation matrix. 
 * @param[out] quat Quaternion_t The output quaterneon (x,y,z,w)
 * @param[out] d Point3_t The output displacement vector (x,y,z)
 */
void hgt2quatpos( const Hgt_t& T, Quaternion_t& quat, Point3_t& d);

/**
 * @brief Converts several 4x4 homogeneous transformation matrix into a matrix of 
 * 4-vector quaternions and 3-vector displacement vectors
 * 
 * @param[in] T HgtArray_t<N> T, the homogeneous transformation matrix. 
 * @param[out] quat QuaternionArray_t<N> The output quaterneon (x,y,z,w)
 * @param[out] d Point3Array_t<N> The output displacement vector (x,y,z)
 */
void hgt2quatpos( const HgtArray_t<Dynamic>& T, QuaternionArray_t<Dynamic>& quat, Point3Array_t<Dynamic>& d);

/**
 * @brief Converts a 4-vector quaternion and a 3-vector displacement vector
 * to a 4x4 homogeneous transformation matrix.
 * 
 * @param[in] quat The input quaterneon (x,y,z,w)
 * @param[in] d The input displacement vector (x,y,z)
 * @param[out] T Hgt_t T, the output homogeneous transformation matrix. 
 */
void quatpos2hgt( const Quaternion_t& quat, const Point3_t& d, Hgt_t& T);

/**
 * @brief Converts several 4-vector quaternions and 3-vector displacement vectors
 * to 4x4 homogeneous transformation matrices (vertically stacked)..
 * 
 * @param[in] quat QuaternionArray_t<N> The input quaterneon (x,y,z,w)
 * @param[in] d Point3Array_t<N> The input displacement vector (x,y,z)
 * @param[out] T HgtArray_t<N> T, the output homogeneous transformation matrix. 
 */
void quatpos2hgt( const QuaternionArray_t<Dynamic>& quat, const Point3Array_t<Dynamic>& d, HgtArray_t<Dynamic>& T);

/**
 * @brief Checks if a matrix is a homogeneous transformation matrix
 * 
 * @param T The matrix
 * @return bool
 */
bool isRotationMatrix(const Rotation_t &R, double tolerance = 1e-6);

/**
 * @brief Checks if a matrix is a homogeneous transformation matrix
 * 
 * @param T The matrix
 * @return bool
 */
bool ishgt(const Hgt_t &T, double tolerance = 1e-6);

/**
 * @brief Computes the skew-symmetric 3x3 matrix for a vector
 * 
 * @param w The vector
 * @return Matrix3d The skew-symmetric matrix
 */
Matrix3d skew(Vector3d w);

/**
 * @brief Computes the 3-vector corresponding to a 3x3 skew-symmetric matrix
 * 
 * @param S The skew-symmetric matrix
 * @param check_tolerance The tolerance for checking that the input is actually skew-symmetric.
 * Default 1e-6. Give a negative value to skip checking (faster). 
 * @return Vector3d The 3-vector
 */
Vector3d vex(Matrix3d S, double check_tolerance = 1e-6);

/**
 * @brief Creates an adjoint matrix from a rotation matrix. Optionaly inverts the transform
 * 
 * @param R The rotation matrix
 * @param invert Flag to invert. Default: false.
 * @return Adjoint_t The adjoint matrix
 */
Adjoint_t adjoint(const Rotation_t& R, bool invert = false);

/**
 * @brief Creates an adjoint matrix from a pure displacement. Optionally inverts it.
 * 
 * @param d The displacement vector
 * @param invert Flag to invert (default: false)
 * @return Adjoint_t 
 */
Adjoint_t adjoint(const Vector3d& d, bool invert = false);

/**
 * @brief Creates an adjoint matrix from a transformation matrix. Optionally inverts it.
 * 
 * @param T The homogeneous transform
 * @param invert The invert flag (default: false)
 * @return Adjoint_t 
 */
Adjoint_t adjoint(const Hgt_t& T, bool invert = false);

} // End of namespace Geometry
} // End of namespace quik