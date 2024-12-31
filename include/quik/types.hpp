#pragma once

#include "Eigen/Dense"

using namespace Eigen;

namespace quik{

template<int n=Dynamic, int N=Dynamic>
using VectorArray_t = Matrix<double, n, N>;

template<int n=Dynamic, int m=Dynamic, int N=Dynamic>
using MatrixArray_t = Matrix<double, (N>0 ? N*n : -1), m>;

using Hgt_t = Matrix4d;

using Adjoint_t = Matrix<double,6,6>;

using Rotation_t = Matrix3d;

template<int N=Dynamic>
using HgtArray_t = MatrixArray_t<4, 4, N>;

template<int DOF=Dynamic>
using JointHgtArray_t = MatrixArray_t<4,4,(DOF>0 ? (DOF+1) : -1)>;

template <int DOF=Dynamic>
using JointState_t = Vector<double,DOF>;

template <int DOF=Dynamic, int N=Dynamic>
using JointStateArray_t = VectorArray_t<DOF, N>;

template <int DOF=Dynamic>
using Jacobian_t = Matrix<double,6,DOF>;

using Twist_t = Vector<double,6>;

template <int N=Dynamic>
using TwistArray_t = VectorArray_t<6, N>;

using Quaternion_t = Vector<double,4>;

template <int N=Dynamic>
using QuaternionArray_t = VectorArray_t<4, N>;

using Point3_t = Vector<double,3>;

template <int N=Dynamic>
using Point3Array_t = VectorArray_t<3, N>;

};