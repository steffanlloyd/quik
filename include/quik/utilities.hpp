/**
 * @file utilities.hpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief Defines helper functions that are not ROS-related.
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include "Eigen/Dense"
#include <Eigen/Core>
#include "quik/Robot.hpp"
// #include "quik/IKSolver.hpp"
#include <sstream>
#include <string>

namespace quik{
namespace utilities{

/**
 * @brief Helper function that allows eigen objects to be cast to strings
 * 
 * @tparam Derived 
 * @param m 
 * @return std::string 
 */
inline Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
template<typename Derived>
std::string eigen2str(const Eigen::MatrixBase<Derived>& m){
    std::stringstream ss;
    ss << m.format(CleanFmt);
    return ss.str();
};


} // end of namespace quik::utilities
} // end of namespace quik