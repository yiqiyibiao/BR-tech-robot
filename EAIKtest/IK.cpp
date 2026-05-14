#include "IK.h"
#include "robot_config.h"

#include <iostream>
#include <future>
#include <cmath>
#include <algorithm>

namespace
{
    double jointDistanceScore(
        const std::vector<double>& q,
        const std::vector<double>& q_ref
    )
    {
        if (q.size() != q_ref.size())
            return 1e100;

        double s = 0.0;
        for (size_t i = 0; i < q.size(); ++i)
        {
            double d = q[i] - q_ref[i];
            s += d * d;
        }
        return std::sqrt(s);
    }

    std::vector<double> makeLimitVector(const RobotConfig& config, bool use_min)
    {
        std::vector<double> q(6);
        for (int i = 0; i < 6; ++i)
            q[i] = use_min ? config.limits[i].q_min : config.limits[i].q_max;
        return q;
    }

    IKCandidate solveOneSeedThreadSafe(
        const KDL::Chain& chain,
        const KDL::JntArray& q_min_kdl,
        const KDL::JntArray& q_max_kdl,
        const KDL::Frame& T_target,
        const std::vector<double>& seed_robot,
        const std::vector<double>& q_reference_robot,
        int max_iter,
        double eps
    )
    {
        IKCandidate out;

        if (seed_robot.size() != chain.getNrOfJoints())
            return out;

        // 关键：每个线程独立创建 KDL solver，避免共享 solver 的线程安全问题。
        KDL::ChainFkSolverPos_recursive fk(chain);
        KDL::ChainIkSolverVel_pinv vel(chain);
        KDL::ChainIkSolverPos_NR_JL ik(
            chain,
            q_min_kdl,
            q_max_kdl,
            fk,
            vel,
            max_iter,
            eps
        );

        KDL::JntArray q_init = robotRadToKDLRad(seed_robot);
        KDL::JntArray q_result(chain.getNrOfJoints());

        int ret = ik.CartToJnt(q_init, T_target, q_result);
        if (ret < 0)
            return out;

        out.success = true;
        out.q = KDLRadToRobotRad(q_result);
        out.score = jointDistanceScore(out.q, q_reference_robot);
        return out;
    }
}

RobotIK::RobotIK(const RobotConfig& config)
    : chain(config.buildChain()),
    q_min_kdl(robotRadToKDLRad(makeLimitVector(config, true))),
    q_max_kdl(robotRadToKDLRad(makeLimitVector(config, false))),
    q_min_robot(makeLimitVector(config, true)),
    q_max_robot(makeLimitVector(config, false)),
    fk_solver(chain),
    ik_vel_solver(chain),
    ik_pos_solver(
        chain,
        q_min_kdl,
        q_max_kdl,
        fk_solver,
        ik_vel_solver,
        15,
        1e-5
    ),
    max_iter(15),
    eps(1e-5)
{
}

bool RobotIK::solve(
    const KDL::Frame& T_target,
    const std::vector<double>& q_init_robot,
    std::vector<double>& q_result_robot
)
{
    if (q_init_robot.size() != chain.getNrOfJoints())
        return false;

    KDL::JntArray q_init = robotRadToKDLRad(q_init_robot);
    KDL::JntArray q_result(chain.getNrOfJoints());

    int ret = ik_pos_solver.CartToJnt(q_init, T_target, q_result);

    if (ret < 0)
        return false;

    q_result_robot = KDLRadToRobotRad(q_result);
    return true;
}

IKCandidate RobotIK::solveMultiSeedParallel(
    const KDL::Frame& T_target,
    const std::vector<std::vector<double>>& seeds_robot,
    const std::vector<double>& q_reference_robot
) const
{
    IKCandidate best;

    if (seeds_robot.empty())
        return best;

    std::vector<std::future<IKCandidate>> futures;
    futures.reserve(seeds_robot.size());

    for (const auto& seed : seeds_robot)
    {
        futures.emplace_back(std::async(
            std::launch::async,
            solveOneSeedThreadSafe,
            std::cref(chain),
            std::cref(q_min_kdl),
            std::cref(q_max_kdl),
            std::cref(T_target),
            std::cref(seed),
            std::cref(q_reference_robot),
            max_iter,
            eps
        ));
    }

    for (auto& f : futures)
    {
        IKCandidate r = f.get();
        if (r.success && r.score < best.score)
            best = r;
    }

    return best;
}

unsigned int RobotIK::getNrOfJoints() const
{
    return chain.getNrOfJoints();
}

const std::vector<double>& RobotIK::getQMinRobot() const
{
    return q_min_robot;
}

const std::vector<double>& RobotIK::getQMaxRobot() const
{
    return q_max_robot;
}

void RobotIK::printQ(const std::vector<double>& q)
{
    for (size_t i = 0; i < q.size(); ++i)
    {
        std::cout << "J" << i + 1 << " = "
            << q[i] << " rad, "
            << q[i] * 180.0 / mymath::MY_PI
            << " deg" << std::endl;
    }
}
