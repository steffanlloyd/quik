#pragma once

#include "Eigen/Dense"
#include "quik/types.hpp"
#include "quik/Robot.hpp"
#include <unordered_set>
#include <vector>
#include <stdexcept>

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

template<int m = Dynamic>
class WorldConstraint : public Constraint<m>
{
private:
    Matrix<double,m,6> B_; 

    static void validate_axes_(const vector<int>& axes) {
        // Check that the size of axes is correct
        if (m != Dynamic && axes.size() != m) {
            throw invalid_argument("Number of axes must match the template parameter m.");
        }

        // Check that all values are between 0, 5 with no repeated values
        unordered_set<int> unique_axes;
        for (int axis : axes) {
            if (axis < 0 || axis >= 6) {
                throw out_of_range("Axis index out of valid range [0, 5].");
            }
            if (!unique_axes.insert(axis).second) {
                throw invalid_argument("Duplicate axis values are not allowed.");
            }
        }
    }

public:
    explicit WorldConstraint(
        const vector<int>& axes =  {0, 1, 2, 3, 4, 5}, 
        const Hgt_t& transform = Hgt_t::Identity())
    {
        this->validate_axes_(axes);

        Adjoint_t Ad = geometry::adjoint(transform);

        // Assign correct axes from adjoint
        for (size_t i = 0; i < axes.size(); ++i) {
            this->B_(i, axes[i]) = 1.0;
            this->B_.row(i) = Ad.row(axes[i]);
        }
    }

    static WorldConstraint World() {
        return WorldConstraint({0,1,2,3,4,5}, Hgt_t::Identity());
    }

    static WorldConstraint Partial(const vector<int>& axes) {
        return WorldConstraint(axes, Hgt_t::Identity());
    }

    // Constructor for a full 6-DOF constraint with a custom transform
    static WorldConstraint TransformedWorld(const Hgt_t& transform) {
        return WorldConstraint({0, 1, 2, 3, 4, 5}, transform);
    }

    // Constructor for a partial constraint with a custom transform
    static WorldConstraint TransformedPartial(const vector<int>& axes, const Hgt_t& transform) {
        return WorldConstraint(axes, transform);
    }

    Matrix<double,m,6> get_transform(const Hgt_t&) override
    {
        return this->B_;
    }
};

};