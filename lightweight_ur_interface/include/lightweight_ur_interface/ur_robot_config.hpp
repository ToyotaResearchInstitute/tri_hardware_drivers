#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>

#ifndef UR_ROBOT_CONFIG_HPP
#define UR_ROBOT_CONFIG_HPP

namespace lightweight_ur_interface
{
    class PIDParams
    {
    protected:

        double kp_;
        double ki_;
        double kd_;
        double i_clamp_;

    public:

        PIDParams(const double kp, const double ki, const double kd, const double i_clamp) : kp_(kp), ki_(ki), kd_(kd), i_clamp_(i_clamp) {}

        PIDParams() : kp_(0.0), ki_(0.0), kd_(0.0), i_clamp_(0.0) {}

        inline double Kp() const
        {
            return kp_;
        }

        inline double Ki() const
        {
            return ki_;
        }

        inline double Kd() const
        {
            return kd_;
        }

        inline double Iclamp() const
        {
            return i_clamp_;
        }
    };

    inline std::ostream& operator<<(std::ostream& strm, const PIDParams& params)
    {
        strm << "Kp: " << params.Kp() << " Ki: " << params.Ki() << " Kd: " << params.Kd() << " iclamp: " << params.Iclamp();
        return strm;
    }

    class JointLimits
    {
    protected:

        double min_position_;
        double max_position_;
        double max_velocity_;
        double max_acceleration_;

    public:

        JointLimits(const double min_position, const double max_position, const double max_velocity, const double max_acceleration)
        {
            assert(min_position <= max_position);
            min_position_ = min_position;
            max_position_ = max_position;
            max_velocity_ = std::abs(max_velocity);
            max_acceleration_ = std::abs(max_acceleration);
        }

        JointLimits() : min_position_(0.0), max_position_(0.0), max_velocity_(0.0), max_acceleration_(0.0) {}

        inline double MinPosition() const
        {
            return min_position_;
        }

        inline double MaxPosition() const
        {
            return max_position_;
        }

        inline std::pair<double, double> PositionLimits() const
        {
            return std::make_pair(min_position_, max_position_);
        }

        inline double MaxVelocity() const
        {
            return max_velocity_;
        }

        inline double MaxAcceleration() const
        {
            return max_acceleration_;
        }
    };

    inline std::ostream& operator<<(std::ostream& strm, const JointLimits& limits)
    {
        strm << "Position: [" << limits.MinPosition() << "," << limits.MaxPosition() << ") Velocity: " << limits.MaxVelocity() << " Acceleration: " << limits.MaxAcceleration();
        return strm;
    }

    inline std::vector<std::string> GetOrderedJointNames()
    {
        return std::vector<std::string>{"shoulder_pan_joint",
                                        "shoulder_lift_joint",
                                        "elbow_joint",
                                        "wrist_1_joint",
                                        "wrist_2_joint",
                                        "wrist_3_joint"};
    }

    inline std::map<std::string, JointLimits> GetDefaultLimits()
    {
        std::map<std::string, JointLimits> joint_limits;
        joint_limits["shoulder_pan_joint"] = JointLimits(-2.0 * M_PI, 2.0 * M_PI, 2.0, 2.0);
        joint_limits["shoulder_lift_joint"] = JointLimits(-2.0 * M_PI, 2.0 * M_PI, 2.0, 2.0);
        joint_limits["elbow_joint"] = JointLimits(-2.0 * M_PI, 2.0 * M_PI, 3.0, 3.0);
        joint_limits["wrist_1_joint"] = JointLimits(-2.0 * M_PI, 2.0 * M_PI, 3.0, 3.0);
        joint_limits["wrist_2_joint"] = JointLimits(-2.0 * M_PI, 2.0 * M_PI, 3.0, 3.0);
        joint_limits["wrist_3_joint"] = JointLimits(-2.0 * M_PI, 2.0 * M_PI, 3.0, 3.0);
        return joint_limits;
    }

    inline std::map<std::string, JointLimits> GetLimits(const double velocity_scaling=1.0, const double acceleration_scaling=1.0)
    {
        assert(velocity_scaling >= 0.0);
        assert(velocity_scaling <= 1.0);
        assert(acceleration_scaling >= 0.0);
        assert(acceleration_scaling <= 1.0);
        const std::map<std::string, JointLimits> default_joint_limits = GetDefaultLimits();
        std::map<std::string, JointLimits> joint_limits;
        const auto default_limit_pairs = arc_helpers::GetKeysAndValues(default_joint_limits);
        for (size_t idx = 0; idx < default_limit_pairs.size(); idx++)
        {
            const std::string& joint_name = default_limit_pairs[idx].first;
            const JointLimits& default_joint_limit = default_limit_pairs[idx].second;
            const JointLimits scaled_joint_limit(default_joint_limit.MinPosition(), default_joint_limit.MaxPosition(), (default_joint_limit.MaxVelocity() * velocity_scaling), (default_joint_limit.MaxAcceleration() * acceleration_scaling));
            joint_limits[joint_name] = scaled_joint_limit;
        }
        return joint_limits;
    }

    inline std::map<std::string, PIDParams> GetDefaultPositionControllerParams(const double base_kp, const double base_ki, const double base_kd, const double base_i_clamp)
    {
        std::map<std::string, PIDParams> joint_controller_params;
        joint_controller_params["shoulder_pan_joint"] = PIDParams(base_kp, base_ki, base_kd, base_i_clamp);
        joint_controller_params["shoulder_lift_joint"] = PIDParams(base_kp, base_ki, base_kd, base_i_clamp);
        joint_controller_params["elbow_joint"] = PIDParams(base_kp, base_ki, base_kd, base_i_clamp);
        joint_controller_params["wrist_1_joint"] = PIDParams(base_kp, base_ki, base_kd, base_i_clamp);
        joint_controller_params["wrist_2_joint"] = PIDParams(base_kp, base_ki, base_kd, base_i_clamp);
        joint_controller_params["wrist_3_joint"] = PIDParams(base_kp, base_ki, base_kd, base_i_clamp);
        return joint_controller_params;
    }

    inline std::vector<PIDParams> GetDefaultPoseControllerParams(const double translation_stiffness, const double translation_damping, const double rotation_stiffness, const double rotation_damping)
    {
        std::vector<PIDParams> pose_controller_params(6);
        pose_controller_params[0] = PIDParams(translation_stiffness, 0.0, translation_damping, 0.0);
        pose_controller_params[1] = PIDParams(translation_stiffness, 0.0, translation_damping, 0.0);
        pose_controller_params[2] = PIDParams(translation_stiffness, 0.0, translation_damping, 0.0);
        pose_controller_params[3] = PIDParams(rotation_stiffness, 0.0, rotation_damping, 0.0);
        pose_controller_params[4] = PIDParams(rotation_stiffness, 0.0, rotation_damping, 0.0);
        pose_controller_params[5] = PIDParams(rotation_stiffness, 0.0, rotation_damping, 0.0);
        return pose_controller_params;
    }

    inline bool IsSubset(const std::vector<std::string>& candidate_set, const std::vector<std::string>& candidate_subset)
    {
        if (candidate_set.size() < candidate_subset.size())
        {
            return false;
        }
        std::map<std::string, uint8_t> names_map;
        for (size_t idx = 0; idx < candidate_set.size(); idx++)
        {
            const std::string& name = candidate_set[idx];
            names_map[name] = 0x01;
        }
        for (size_t idx = 0; idx < candidate_subset.size(); idx++)
        {
            const std::string& name = candidate_subset[idx];
            const auto found_itr = names_map.find(name);
            if (found_itr == names_map.end())
            {
                return false;
            }
        }
        return true;
    }

    inline bool SetsEqual(const std::vector<std::string>& set1, const std::vector<std::string>& set2)
    {
        if (IsSubset(set1, set2) && IsSubset(set2, set1))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
#endif // UR_ROBOT_CONFIG_HPP
