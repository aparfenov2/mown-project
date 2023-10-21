#pragma once

#include <ros/ros.h>
#include <tracking_pid/traj_point.h>
#include <nav_msgs/Path.h>

namespace tracking_pid
{
    class SectionInterpolation
    {
    public:
        // Interpolate over the given section with some x and yaw velocities
        // :param from_: start of the section
        // :type from_: PoseStamped
        // :param to: end of the section
        // :type to: PoseStamped
        // :param x_vel: translational velocity to move the trajectory target pose
        // :type x_vel: float
        // :param x_acc: translational acceleration to move the trajectory target pose
        // :type x_acc: float
        // :param yaw_vel: rotational velocity to rotate the trajectory target pose
        // :type yaw_vel: float
        // :param yaw_acc: rotational acceleration to rotate the trajectory target pose
        // :type yaw_acc: float
        double _x_vel;
        double _x_acc_decc;
        double _yaw_vel;
        double _yaw_acc_decc;
        double _start_yaw;
        double _end_yaw;
        ros::Duration duration_on_section;
        geometry_msgs::PoseStamped section_start_pose_stamped;
        geometry_msgs::PoseStamped section_end_pose_stamped;
        double _start_xyz[3];
        double _end_xyz[3];
        double _delta[3];
        double _delta_yaw;
        double length_of_section;
        double length_of_section_ang;
        double time_x_acc_decc;
        double length_x_acc_decc;
        double length_x_vel;
        double time_x_vel;
        double _x_vel_adjusted;
        double time_yaw_acc_decc;
        double length_yaw_acc_decc;
        double length_yaw_vel;
        double time_yaw_vel;
        double _yaw_vel_adjusted;
        double duration_for_section_x;
        double duration_for_section_yaw;
        ros::Duration duration_for_section;
        ros::Time section_start_time;
        double x_progress;
        double yaw_progress;
        double current_x_vel;
        double current_yaw_vel;
        ros::Time section_end_time;

        SectionInterpolation(
            geometry_msgs::PoseStamped from_,
            geometry_msgs::PoseStamped to,
            ros::Time start_time,
            double x_vel,
            double x_acc,
            double yaw_vel,
            double yaw_acc);

        geometry_msgs::PoseStamped interpolate(double progress_ratio);
        tracking_pid::traj_point interpolate_with_acceleration(ros::Time current_time);
    };

    class InterpolatorNode
    {
    public:
        nav_msgs::Path _latest_path_msg;
        std::vector<std::vector<geometry_msgs::PoseStamped>> _sections;
        double _target_x_vel;
        double _target_x_acc;
        double _target_yaw_vel;
        double _target_yaw_acc;
        std::vector<geometry_msgs::PoseStamped> _path_poses;
        std::unique_ptr<SectionInterpolation> _current_section;
        double progress_on_section;
        geometry_msgs::PoseStamped _latest_subgoal_pose;
        /**
         * """Accept and store the path"""
         * */
        void _process_path(nav_msgs::Path path_msg);
        void _update_target(tracking_pid::traj_point &tp);
    };
}