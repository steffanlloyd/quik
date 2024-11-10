#pragma once

#include "Eigen/Dense"

using namespace Eigen;

namespace quik{
namespace Geometry{

void hgtDiff(const Matrix4d& T1, const Matrix4d& T2, Vector<double,6>& e);
Matrix4d hgtInv( const Matrix4d& T );
void hgt2quatpos( const Matrix4d& T, Vector4d& quat, Vector3d& d);
void hgt2quatpos( const Matrix<double,Dynamic,4>& T, Matrix<double,4,Dynamic>& quat, Matrix<double,3,Dynamic>& d);
void quatpos2hgt( const Vector4d& quat, const Vector3d& d, Matrix4d& T);
void quatpos2hgt( const Matrix<double,4,Dynamic>& quat, const Matrix<double,3,Dynamic>& d, Matrix<double,Dynamic,4>& T);
bool isRotationMatrix(const Matrix3d &R, double tolerance = 1e-6);
bool ishgt(const Matrix4d &T, double tolerance = 1e-6);

} // End of namespace Geometry
} // End of namespace quik