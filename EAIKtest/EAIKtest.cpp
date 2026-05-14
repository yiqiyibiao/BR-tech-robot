#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "EAIK.h"

#include "robot_config.h"
#include "FK.h"

static bool isValidSolution(const std::vector<double>& q)
{
    for (double v : q)
    {
        if (!std::isfinite(v))
            return false;
    }
    return true;
}

int main()
{
    RobotConfig config;
    RobotFK fk(config);

    Eigen::Vector3d ex(1, 0, 0);
    Eigen::Vector3d ey(0, 1, 0);
    Eigen::Vector3d ez(0, 0, 1);

    // 正确：按这个机器人的理论解析结构
    Eigen::Matrix<double, 3, 6> H;
    H << ez, ey, ey, ey, ez, ey;

    Eigen::Matrix<double, 3, 7> P;
    P << Eigen::Vector3d(0.0, 0.0, 117.587347293043),
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(424.978480552240, 0.0, 140.260398528734),
        Eigen::Vector3d(391.572138818323, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 100.274726688559),
        Eigen::Vector3d(1.51, 4.21, 418.63);

    std::cout << "\n===== H =====\n" << H << std::endl;
    std::cout << "\n===== P =====\n" << P << std::endl;

    EAIK::Robot robot(H, P);

    std::cout << "\n===== Kinematic Family =====\n";
    std::cout << robot.get_kinematic_family() << std::endl;

    std::vector<double> q =
    {
        0.1,
        0.2,
        -0.3,
        0.5,
        -0.2,
        0.4
    };

    KDL::Frame T_kdl;

    if (!fk.solve(q, T_kdl))
    {
        std::cout << "KDL FK failed\n";
        return 0;
    }

    std::cout << "\n===== KDL FK =====\n";
    std::cout << "XYZ = "
        << T_kdl.p.x() << ", "
        << T_kdl.p.y() << ", "
        << T_kdl.p.z() << std::endl;

    auto T_eaik = robot.fwdkin(q);

    std::cout << "\n===== EAIK FK =====\n";
    std::cout << T_eaik << std::endl;

    try
    {
        auto sol = robot.calculate_IK(T_eaik);

        std::cout << "\nraw solution num = "
            << sol.Q.size()
            << std::endl;

        int valid_count = 0;

        for (size_t i = 0; i < sol.Q.size(); ++i)
        {
            if (!isValidSolution(sol.Q[i]))
                continue;

            std::cout << "\nIK valid [" << valid_count << "] = ";

            for (double v : sol.Q[i])
                std::cout << v << " ";

            std::cout << std::endl;
            valid_count++;
        }

        std::cout << "\nvalid solution num = "
            << valid_count
            << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cout << "\n===== IK ERROR =====\n";
        std::cout << e.what() << std::endl;
    }

    return 0;
}