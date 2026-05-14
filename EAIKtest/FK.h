#pragma once

#include <vector>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
class RobotConfig;

class RobotFK
{
private:
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive fk_solver;

public:
    RobotFK(const RobotConfig& config);
    void debugPrintChainZero();
    bool solve(
        const std::vector<double>& q_robot,
        KDL::Frame& T_out
    );

    bool extractEAIK_HP(
        Eigen::Matrix<double, 3, 6>& H,
        Eigen::Matrix<double, 3, 7>& P
    );

    void printFrame(const KDL::Frame& T);
};