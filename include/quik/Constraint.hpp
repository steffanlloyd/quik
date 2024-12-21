#pragma once

#include "Eigen/Dense"
#include "quik/types.hpp"
#include "quik/Robot.hpp"

using namespace Eigen;
using namespace quik;
using namespace std;

namespace quik{

template<int m=Dynamic>
class Constraint
{
public:
    Constraint(){}
    
    virtual Matrix<double,m,6> get_transform(const Hgt_t& Tn) = 0;
};

class WorldConstraint : public Constraint<6>
{
public:
    WorldConstraint(){}

    Matrix<double,6,6> get_transform(const Hgt_t&) override
    {
        return Matrix<double,6,6>::Identity();
    }
};

};