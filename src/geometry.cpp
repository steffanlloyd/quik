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
#include "quik/types.hpp"
#include <iostream>

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
 * @param[in] T1 The first transform
 * @param[in] T2 The second transform
 * @param[out] e The error (passed as reference and transformed)
 */
void hgtDiff(const Hgt_t& T1, const Hgt_t& T2, Twist_t& e)
{
    Rotation_t R1, R2, Re;
    Point3_t d1, d2, eps;
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
 * @return Hgt_t 
 */
Hgt_t hgtInv( const Hgt_t& T )
{
    Hgt_t Tinv;
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
 * @param[in] T Hgt_t T, the homogeneous transformation matrix. 
 * @param[out] quat The output quaterneon (x,y,z,w)
 * @param[out] d The output displacement vector (x,y,z)
 */
void hgt2quatpos( const Hgt_t& T, Quaternion_t& quat, Point3_t& d)
{
    // Extract the rotation matrix from the homogeneous transformation matrix
    Rotation_t R = T.block<3,3>(0,0);

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
 * @param[in] T HgtArray_t T, the homogeneous transformation matrix. 
 * @param[out] quat QuaternionArray_t The output quaterneon (x,y,z,w)
 * @param[out] d Point3Array_t The output displacement vector (x,y,z)
 */
void hgt2quatpos( const HgtArray_t<Dynamic>& T, QuaternionArray_t<Dynamic>& quat, Point3Array_t<Dynamic>& d)
{
    // Get the number of transformations
    int N = T.rows() / 4;

    if(quat.cols() != N) throw std::runtime_error("Number of columns in quat should be equal to the number of rows in T / 4.");
    if(d.cols() != N) throw std::runtime_error("Number of columns in d should be equal to the number of rows in T / 4.");

    // Iterate over each transformation
    for(int i = 0; i < N; ++i) {
        // Init and compute results
        Point3_t d_i; 
        Quaternion_t quat_i;
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
 * @param[in] quat Quaternion_t The input quaterneon (x,y,z,w)
 * @param[in] d Point3_t The input displacement vector (x,y,z)
 * @param[out] T Hgt_t T, the output homogeneous transformation matrix. 
 */
void quatpos2hgt( const Quaternion_t& quat, const Point3_t& d, Hgt_t& T)
{
    // Convert quaternion to rotation matrix
    Quaterniond quaternion(quat(3), quat(0), quat(1), quat(2));
    Rotation_t rotation = quaternion.normalized().toRotationMatrix();

    // Assign transform
    T.block<3,3>(0,0) = rotation;
    T.block<3,1>(0,3) = d;
    T.row(3) << 0, 0, 0, 1;
}

/**
 * @brief Converts several 4-vector quaternions and 3-vector displacement vectors
 * to 4x4 homogeneous transformation matrices (vertically stacked)..
 * 
 * @param[in] quat QuaternionArray_t<N> The input quaterneon (x,y,z,w)
 * @param[in] d Point3Array_t<N> The input displacement vector (x,y,z)
 * @param[out] T HgtArray_t<N> T, the output homogeneous transformation matrix. 
 */
void quatpos2hgt( const QuaternionArray_t<Dynamic>& quat, const Point3Array_t<Dynamic>& d, HgtArray_t<Dynamic>& T)
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
bool isRotationMatrix(const Rotation_t &R, double tolerance)
{

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
bool ishgt(const Hgt_t &T, double tolerance)
{
    // Check if last row is [0, 0, 0, 1]
    if (!T.row(3).transpose().isApprox(Vector4d(0, 0, 0, 1), tolerance)) return false;

    // Check that rotation part is rotation matrix
    return quik::geometry::isRotationMatrix(T.block<3,3>(0,0));
}

/**
 * @brief Computes the skew-symmetric 3x3 matrix for a vector
 * 
 * @param w The vector
 * @return Matrix3d The skew-symmetric matrix
 */
Matrix3d skew(Vector3d w) 
{
    Matrix3d S;

    S << 0, -w(2), w(1),
         w(2), 0, -w(0),
         -w(1), w(0), 0;

    return S;
}

/**
 * @brief Computes the 3-vector corresponding to a 3x3 skew-symmetric matrix
 * 
 * @param S The skew-symmetric matrix
 * @param check_tolerance The tolerance for checking that the input is actually skew-symmetric.
 * Default 1e-6. Give a negative value to skip checking (faster). 
 * @return Vector3d The 3-vector
 */
Vector3d vex(Matrix3d S, double check_tolerance){

    if (S.rows() != S.cols()) throw std::invalid_argument("S must be square!");
    if (S.rows() != 3) throw std::invalid_argument("S must be 3x3");

    if (check_tolerance > 0) {
        Matrix3d sum = S + S.transpose();
        if (sum.array().abs().maxCoeff() > check_tolerance) {
            throw std::runtime_error("Input matrix is not skew symmetric within a tolerance of " + std::to_string(check_tolerance) + "! Be careful with results!");
        }
    }

    Vector3d w1(S(2, 1), S(0, 2), S(1, 0));
    Vector3d w2(S(1, 2), S(2, 0), S(0, 1));

    return (w1 - w2) / 2;
}


/**
 * @brief Creates an adjoint matrix from a rotation matrix. Optionaly inverts the transform
 * 
 * @param R The rotation matrix
 * @param invert Flag to invert. Default: false.
 * @return Adjoint_t The adjoint matrix
 */
Adjoint_t adjoint(const Rotation_t& R, bool invert)
{
    Adjoint_t Ad;
    Matrix3d Z = Matrix3d::Zero();
    Rotation_t Rt = (invert) ? R.transpose() : R;

    Ad <<   Rt, Z,
            Z, Rt;

    return Ad;
}

/**
 * @brief Creates an adjoint matrix from a pure displacement. Optionally inverts it.
 * 
 * @param d The displacement vector
 * @param invert Flag to invert (default: false)
 * @return Adjoint_t 
 */
Adjoint_t adjoint(const Vector3d& d, bool invert)
{
    Adjoint_t Ad;
    Matrix3d Z = Matrix3d::Zero();
    Matrix3d I = Matrix3d::Identity();
    Matrix3d S = (invert) ? geometry::skew(-d) : geometry::skew(d);

    Ad <<   I, S,
            Z, I;

    return Ad;
}

/**
 * @brief Creates an adjoint matrix from a transformation matrix. Optionally inverts it.
 * 
 * @param T The homogeneous transform
 * @param invert The invert flag (default: false)
 * @return Adjoint_t 
 */
Adjoint_t adjoint(const Hgt_t& T, bool invert)
{
    Adjoint_t Ad;
    Matrix3d R = T.topLeftCorner(3, 3);
    Vector3d p = T.topRightCorner(3, 1);
    Matrix3d S = geometry::skew(p);

    if (invert) {
        Matrix3d Rt = R.transpose();
        Ad << Rt, -Rt * S,
              Matrix3d::Zero(), Rt;
    } else {
        Ad << R, S * R,
              Matrix3d::Zero(), R;
    }

    return Ad;
}

} // End of namespace geometry
} // End of namespace quik