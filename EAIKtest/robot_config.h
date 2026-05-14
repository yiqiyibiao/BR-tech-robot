#pragma once

#include <vector>
#include <string>
#include <cmath>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace mymath
{
    constexpr double MY_PI = 3.14159265358979323846;

    inline double deg2rad(double deg)
    {
        return deg * MY_PI / 180.0;
    }

    inline double rad2deg(double rad)
    {
        return rad * 180.0 / MY_PI;
    }
}

struct JointLimit
{
    double q_min;
    double q_max;
    double dq_max;
    double ddq_max;
};

struct MDHParam
{
    std::string name;
    double a;
    double alpha;
    double d;
    double offset;
};

struct ToolTCP
{
    double x;
    double y;
    double z;

    double rx_deg;
    double ry_deg;
    double rz_deg;
};

inline KDL::Frame MDHPre(double a, double alpha)
{
    return KDL::Frame(KDL::Vector(a, 0, 0)) *
        KDL::Frame(KDL::Rotation::RotX(alpha));
}

inline KDL::Frame MDHPost(double d)
{
    return KDL::Frame(KDL::Vector(0, 0, d));
}

class RobotConfig
{
public:

    std::vector<MDHParam> dh;
    std::vector<JointLimit> limits;

    ToolTCP tcp;

    RobotConfig()
    {
        initDH();
        initLimits();
        initTCP();
    }

    void initDH()
    {
        dh =
        {
            {
                "J1",
                0.0,
                0.0,
                117.587347293043,
                0.0
            },

            {
                "J2",
                -0.017361238490,
                -mymath::MY_PI / 2.0 - 0.001605302335,
                140.260398528734,
                -mymath::MY_PI / 2.0
            },

            {
                "J3",
                424.978480552240,
                0.000688646638,
                0.0,
                0.0
            },

            {
                "J4",
                391.572138818323,
                0.004086909316,
                0.0,
                mymath::MY_PI / 2.0
            },

            {
                "J5",
                -0.033016446594,
                mymath::MY_PI / 2.0 + 0.001538988065,
                100.274726688559,
                0.0
            },

            {
                "J6",
                -0.875583335793,
                -mymath::MY_PI / 2.0 - 0.001698851635,
                103.120944418157,
                0.0
            }
        };
    }

    void initLimits()
    {
        limits =
        {
            {
                mymath::deg2rad(-360),
                mymath::deg2rad(360),
                2.0,
                5.0
            },

            {
                mymath::deg2rad(-360),
                mymath::deg2rad(360),
                2.0,
                5.0
            },

            {
                mymath::deg2rad(-360),
                mymath::deg2rad(360),
                2.0,
                5.0
            },

            {
                mymath::deg2rad(-360),
                mymath::deg2rad(360),
                2.0,
                5.0
            },

            {
                mymath::deg2rad(-360),
                mymath::deg2rad(360),
                2.0,
                5.0
            },

            {
                mymath::deg2rad(-360),
                mymath::deg2rad(360),
                2.0,
                5.0
            }
        };
    }

    void initTCP()
    {
        tcp.x = 1.51;
        tcp.y = 4.21;
        tcp.z = 418.63;

        tcp.rx_deg = 0.0;
        tcp.ry_deg = -45.0;
        tcp.rz_deg = 180.0;
    }

    void setTCP(
        double x,
        double y,
        double z,
        double rx_deg,
        double ry_deg,
        double rz_deg
    )
    {
        tcp.x = x;
        tcp.y = y;
        tcp.z = z;

        tcp.rx_deg = rx_deg;
        tcp.ry_deg = ry_deg;
        tcp.rz_deg = rz_deg;
    }

    ToolTCP getTCP() const
    {
        return tcp;
    }

    KDL::Frame getToolFrame() const
    {
        double rx = mymath::deg2rad(tcp.rx_deg);
        double ry = mymath::deg2rad(tcp.ry_deg);
        double rz = mymath::deg2rad(tcp.rz_deg);

        KDL::Rotation R_tool =
            KDL::Rotation::RotZ(rz) *
            KDL::Rotation::RotY(ry) *
            KDL::Rotation::RotX(rx);

        return KDL::Frame(
            R_tool,
            KDL::Vector(
                tcp.x,
                tcp.y,
                tcp.z
            )
        );
    }

    KDL::Chain buildChain() const
    {
        KDL::Chain chain;

        for (size_t i = 0; i < dh.size(); ++i)
        {
            std::string pre_name =
                "L" + std::to_string(i + 1) + "_pre";

            std::string joint_name =
                "J" + std::to_string(i + 1);

            std::string link_name =
                "L" + std::to_string(i + 1);

            chain.addSegment(
                KDL::Segment(
                    pre_name,
                    KDL::Joint(KDL::Joint::None),
                    MDHPre(
                        dh[i].a,
                        dh[i].alpha
                    )
                )
            );

            chain.addSegment(
                KDL::Segment(
                    link_name,
                    KDL::Joint(
                        joint_name,
                        KDL::Joint::RotZ
                    ),
                    MDHPost(dh[i].d)
                )
            );
        }

        chain.addSegment(
            KDL::Segment(
                "tool",
                KDL::Joint(KDL::Joint::None),
                getToolFrame()
            )
        );

        return chain;
    }

    KDL::JntArray getQMin() const
    {
        KDL::JntArray q_min(6);

        for (int i = 0; i < 6; ++i)
            q_min(i) = limits[i].q_min;

        return q_min;
    }

    KDL::JntArray getQMax() const
    {
        KDL::JntArray q_max(6);

        for (int i = 0; i < 6; ++i)
            q_max(i) = limits[i].q_max;

        return q_max;
    }
};

inline KDL::JntArray robotRadToKDLRad(
    const std::vector<double>& q_in
)
{
    KDL::JntArray q(6);

    q(0) = q_in[0];

    q(1) = q_in[1] - mymath::MY_PI / 2.0;

    q(2) = q_in[2];

    q(3) = q_in[3] + mymath::MY_PI / 2.0;

    q(4) = q_in[4];

    q(5) = q_in[5];

    return q;
}

inline std::vector<double> KDLRadToRobotRad(
    const KDL::JntArray& q
)
{
    std::vector<double> q_out(6);

    q_out[0] = q(0);

    q_out[1] = q(1) + mymath::MY_PI / 2.0;

    q_out[2] = q(2);

    q_out[3] = q(3) - mymath::MY_PI / 2.0;

    q_out[4] = q(4);

    q_out[5] = q(5);

    return q_out;
}