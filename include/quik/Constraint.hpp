#pragma once

#include <memory>
#include <iostream>
#include "Eigen/Dense"
#include "quik/Robot.hpp"
#include "quik/geometry.hpp"

using namespace Eigen;
using namespace std;

namespace quik{

template<int m=Dynamic>
class Constraint
{
public:
    Constraint(){}
    
    template<d>
    virtual Matrix<double,m,d> transform(Matrix<double,6,d> A) = 0;
};

class WorldConstraint : public Constraint<6>
{
public:
    WorldConstraint(){}

    template<d>
    Matrix<double,6,d> transform(Matrix<double,6,d> A) override
    {
        return A;
    }
};

};