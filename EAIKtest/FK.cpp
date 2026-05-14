#include "FK.h"
#include "robot_config.h"

#include <iostream>
#include <Eigen/Dense>

RobotFK::RobotFK(const RobotConfig& config)
    : chain(config.buildChain()),
    fk_solver(chain)
{
}

bool RobotFK::solve(
    const std::vector<double>& q_robot,
    KDL::Frame& T_out
)
{
    if (q_robot.size() != chain.getNrOfJoints())
        return false;

    KDL::JntArray q_kdl = robotRadToKDLRad(q_robot);

    int ret = fk_solver.JntToCart(q_kdl, T_out);
    return ret >= 0;
}

bool RobotFK::extractEAIK_HP(
    Eigen::Matrix<double, 3, 6>& H,
    Eigen::Matrix<double, 3, 7>& P
)
{
    std::vector<KDL::Vector> O;
    std::vector<KDL::Vector> Z;

    KDL::Frame T = KDL::Frame::Identity();

    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
    {
        const KDL::Segment& seg = chain.getSegment(i);
        const KDL::Joint& joint = seg.getJoint();

        if (joint.getType() != KDL::Joint::None)
        {
            KDL::Vector origin = T * joint.JointOrigin();
            KDL::Vector axis = T.M * joint.JointAxis();

            O.push_back(origin);
            Z.push_back(axis);
        }

        T = T * seg.pose(0.0);
    }

    if (O.size() != 6 || Z.size() != 6)
    {
        std::cout << "extractEAIK_HP failed: joint size = "
            << O.size() << std::endl;
        return false;
    }

    std::vector<double> q0(6, 0.0);
    KDL::Frame T_tcp;

    if (!solve(q0, T_tcp))
    {
        std::cout << "extractEAIK_HP failed: FK q0 failed" << std::endl;
        return false;
    }

    for (int i = 0; i < 6; ++i)
    {
        H(0, i) = Z[i].x();
        H(1, i) = Z[i].y();
        H(2, i) = Z[i].z();
    }

    KDL::Vector base(0.0, 0.0, 0.0);
    KDL::Vector tcp = T_tcp.p;

    std::vector<KDL::Vector> pvec(7);

    pvec[0] = O[0] - base;
    pvec[1] = O[1] - O[0];
    pvec[2] = O[2] - O[1];
    pvec[3] = O[3] - O[2];
    pvec[4] = O[4] - O[3];
    pvec[5] = O[5] - O[4];
    pvec[6] = tcp - O[5];

    for (int i = 0; i < 7; ++i)
    {
        P(0, i) = pvec[i].x();
        P(1, i) = pvec[i].y();
        P(2, i) = pvec[i].z();
    }

    std::cout << "===== EAIK H =====" << std::endl;
    std::cout << H << std::endl;

    std::cout << "===== EAIK P =====" << std::endl;
    std::cout << P << std::endl;

    return true;
}
void RobotFK::debugPrintChainZero()
{
    KDL::Frame T = KDL::Frame::Identity();

    std::cout << "\n===== KDL Chain Debug =====\n";

    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
    {
        const KDL::Segment& seg = chain.getSegment(i);
        const KDL::Joint& joint = seg.getJoint();

        std::cout << "\nSegment " << i << std::endl;
        std::cout << "Joint name = " << joint.getName() << std::endl;
        std::cout << "Joint type = " << joint.getType() << std::endl;

        if (joint.getType() != KDL::Joint::None)
        {
            KDL::Vector local_axis = joint.JointAxis();
            KDL::Vector local_origin = joint.JointOrigin();

            KDL::Vector world_axis = T.M * local_axis;
            KDL::Vector world_origin = T * local_origin;

            std::cout << "local origin = "
                << local_origin.x() << ", "
                << local_origin.y() << ", "
                << local_origin.z() << std::endl;

            std::cout << "local axis = "
                << local_axis.x() << ", "
                << local_axis.y() << ", "
                << local_axis.z() << std::endl;

            std::cout << "world origin before pose = "
                << world_origin.x() << ", "
                << world_origin.y() << ", "
                << world_origin.z() << std::endl;

            std::cout << "world axis before pose = "
                << world_axis.x() << ", "
                << world_axis.y() << ", "
                << world_axis.z() << std::endl;
        }

        KDL::Frame pose0 = seg.pose(0.0);

        std::cout << "seg.pose(0) p = "
            << pose0.p.x() << ", "
            << pose0.p.y() << ", "
            << pose0.p.z() << std::endl;

        T = T * pose0;

        std::cout << "T after segment p = "
            << T.p.x() << ", "
            << T.p.y() << ", "
            << T.p.z() << std::endl;
    }
}
void RobotFK::printFrame(const KDL::Frame& T)
{
    std::cout << "Position XYZ = "
        << T.p.x() << ", "
        << T.p.y() << ", "
        << T.p.z() << std::endl;

    std::cout << "Rotation matrix:" << std::endl;

    for (int i = 0; i < 3; ++i)
    {
        std::cout << T.M(i, 0) << "  "
            << T.M(i, 1) << "  "
            << T.M(i, 2) << std::endl;
    }
}