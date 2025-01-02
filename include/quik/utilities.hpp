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

/**
 * @brief Converts a string to a BreakReason_t
 * 
 * @param breakReason 
 * @return BreakReason_t 
 */
inline BreakReason_t str2breakreason(std::string breakReason)
{
    if (breakReason == "BREAKREASON_TOLERANCE") return BREAKREASON_TOLERANCE;
    else if (breakReason == "BREAKREASON_MIN_STEP") return BREAKREASON_MIN_STEP;
    else if (breakReason == "BREAKREASON_MAX_ITER") return BREAKREASON_MAX_ITER;
    else if (breakReason == "BREAKREASON_GRAD_FAILS") return BREAKREASON_GRAD_FAILS;
    else throw std::runtime_error("Invalid BREAKREASON string");
};

/**
 * @brief Converts a BreakReason_t to a string (for printing)
 * 
 * @param breakReason 
 * @return std::string 
 */
inline std::string breakreason2str(BreakReason_t breakReason)
{
    switch(breakReason) {
        case BREAKREASON_TOLERANCE: return "BREAKREASON_TOLERANCE";
        case BREAKREASON_MIN_STEP: return "BREAKREASON_MIN_STEP";
        case BREAKREASON_MAX_ITER: return "BREAKREASON_MAX_ITER";
        case BREAKREASON_GRAD_FAILS: return "BREAKREASON_GRAD_FAILS";
        default: return "UNKNOWN_BREAKREASON";
    }
};

/**
 * @brief Converts a string to an Algorithm_t (for parsing yaml files)
 * 
 * @param algorithm 
 * @return Algorithm_t 
 */
inline Algorithm_t str2algorithm(const std::string& algorithm)
{
    if (algorithm == "ALGORITHM_QUIK") return ALGORITHM_QUIK;
    else if (algorithm == "ALGORITHM_NR") return ALGORITHM_NR;
    else throw std::runtime_error("Invalid Algorithm_t string");
};


/**
 * @brief Converts an Algorithm_t to a string (for printing)
 * 
 * @param algorithm 
 * @return std::string 
 */
inline std::string algorithm2str(Algorithm_t algorithm) {
    switch(algorithm) {
        case ALGORITHM_QUIK: return "ALGORITHM_QUIK";
        case ALGORITHM_NR: return "ALGORITHM_NR";
        default: return "UNKNOWN_ALGORITHM";
    }
};

/**
 * @brief Converts a JointType_t to a string (for printing)
 * 
 * @param jointType 
 * @return std::string 
 */
inline std::string jointtype2str(JointType_t jointType) {
    switch(jointType) {
        case JOINT_REVOLUTE: return "JOINT_REVOLUTE";
        case JOINT_PRISMATIC: return "JOINT_PRISMATIC";
        default: return "UNKNOWN_JOINTTYPE";
    }
};

/**
 * @brief Converts a string to a JointType_t (for parsing yaml)
 * 
 * @param jointType 
 * @return JointType_t 
 */
inline JointType_t str2jointtype(std::string jointType) {
    if (jointType == "JOINT_REVOLUTE") return JOINT_REVOLUTE;
    else if (jointType == "JOINT_PRISMATIC") return JOINT_PRISMATIC;
    else throw std::runtime_error("Invalid JOINTTYPE string");
};

} // end of namespace quik::utilities
} // end of namespace quik