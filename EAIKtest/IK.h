#pragma once

#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

class RobotConfig;

struct IKCandidate
{
    bool success = false;
    std::vector<double> q;
    double score = 1e100;
};

class RobotIK
{
private:
    KDL::Chain chain;
    KDL::JntArray q_min_kdl;
    KDL::JntArray q_max_kdl;
    std::vector<double> q_min_robot;
    std::vector<double> q_max_robot;

    // 单 seed solve() 使用。多 seed 并行时不能共享这个 solver。
    KDL::ChainFkSolverPos_recursive fk_solver;
    KDL::ChainIkSolverVel_pinv ik_vel_solver;
    KDL::ChainIkSolverPos_NR_JL ik_pos_solver;

    int max_iter;
    double eps;

public:
    explicit RobotIK(const RobotConfig& config);

    bool solve(
        const KDL::Frame& T_target,
        const std::vector<double>& q_init_robot,
        std::vector<double>& q_result_robot
    );

    // 同一个 T_target，多 seed 并行 IK；最后返回离 q_reference_robot 最近的成功解。
    IKCandidate solveMultiSeedParallel(
        const KDL::Frame& T_target,
        const std::vector<std::vector<double>>& seeds_robot,
        const std::vector<double>& q_reference_robot
    ) const;

    unsigned int getNrOfJoints() const;

    const std::vector<double>& getQMinRobot() const;
    const std::vector<double>& getQMaxRobot() const;

    static void printQ(const std::vector<double>& q);
};
