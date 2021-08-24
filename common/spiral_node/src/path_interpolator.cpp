#include <math.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <tracking_pid/traj_point.h>
#include <nav_msgs/Path.h>

namespace tracking_pid
{
    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
    // SectionInterpolation keeps track and interpolates poses between a start and end PoseStamped, with given
    // x and yaw velocities Based on the difference between the start and end pose and the velocities,
    // a duration for the section is calculated

    // the interpolate method then determines an intermediate point along the section given some progress along the section

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
            double yaw_acc)
        {

            _x_vel = x_vel; // type: float
            _x_acc_decc = x_acc;
            _yaw_vel = yaw_vel;
            _yaw_acc_decc = yaw_acc;

            duration_on_section = ros::Duration(0);

            section_start_pose_stamped = from_; //  // type: PoseStamped
            section_end_pose_stamped = to;      //  // type: PoseStamped

            _start_xyz[0] = section_start_pose_stamped.pose.position.x;
            _start_xyz[1] = section_start_pose_stamped.pose.position.y;
            _start_xyz[2] = section_start_pose_stamped.pose.position.z;

            _end_xyz[0] = section_end_pose_stamped.pose.position.x;
            _end_xyz[1] = section_end_pose_stamped.pose.position.y;
            _end_xyz[2] = section_end_pose_stamped.pose.position.x;

            {
                tf::Quaternion quaternion = tf::Quaternion(
                    section_start_pose_stamped.pose.orientation.x,
                    section_start_pose_stamped.pose.orientation.y,
                    section_start_pose_stamped.pose.orientation.z,
                    section_start_pose_stamped.pose.orientation.w);

                // euler = tf.transformations.euler_from_quaternion(quaternion)
                _start_yaw = tf::getYaw(quaternion);
            }
            {
                tf::Quaternion quaternion = tf::Quaternion(
                    section_end_pose_stamped.pose.orientation.x,
                    section_end_pose_stamped.pose.orientation.y,
                    section_end_pose_stamped.pose.orientation.z,
                    section_end_pose_stamped.pose.orientation.w);

                _end_yaw = tf::getYaw(quaternion);
            }

            //// Warning! These calculations are only valid for yaw. So not to be used in 3D

            for (int i = 0; i < 3; i++)
            {
                _delta[i] = _end_xyz[i] - _start_xyz[i];
            }

            _delta_yaw = _end_yaw - _start_yaw;
            _delta_yaw = std::fmod((_delta_yaw + M_PI), (2 * M_PI)) - M_PI;

            length_of_section = std::sqrt(_delta[0] * _delta[0] + _delta[1] * _delta[1] + _delta[2] * _delta[2]);
            length_of_section_ang = _delta_yaw;

            time_x_acc_decc = _x_vel / _x_acc_decc;                                      // Time during (de) and acceleration phases t = v/a
            length_x_acc_decc = 0.5 * _x_acc_decc * (time_x_acc_decc * time_x_acc_decc); // Translation during acceleration phase x = 0.5a*t^2
            length_x_vel = length_of_section - length_x_acc_decc - length_x_acc_decc;    // Translation during constant velocity phase
            time_x_vel = length_x_vel / _x_vel;                                          // Time during constant velocity phase t = v/a
            _x_vel_adjusted = _x_vel;

            if (time_x_vel < 0)
            { // No constant acceleration phase. Recompute (de)-acceleration phase
                length_x_acc_decc = 0.5 * length_of_section;
                time_x_acc_decc = std::sqrt(2 * length_x_acc_decc / _x_acc_decc); // x = 0.5a*t^2 -> 2x/a = t^2 -> t = sqrt(2x/a)
                _x_vel_adjusted = _x_acc_decc * time_x_acc_decc;
                length_x_vel = 0.0;
                time_x_vel = 0.0;
            }

            time_yaw_acc_decc = _yaw_vel / _yaw_acc_decc;                                        // Time during acceleration phase t = v/a
            length_yaw_acc_decc = 0.5 * _yaw_acc_decc * (time_yaw_acc_decc * time_yaw_acc_decc); // Translation during acceleration phase x = 0.5a*t^2
            length_yaw_vel = length_of_section_ang - length_yaw_acc_decc - length_yaw_acc_decc;  // Translation during constant velocity phase
            time_yaw_vel = length_yaw_vel / _yaw_vel;                                            // Time during constant velocity phase t = v/a
            _yaw_vel_adjusted = _yaw_vel;

            if (time_yaw_vel < 0)
            { // No constant acceleration phase. Recompute (de)-acceleration phase
                length_yaw_acc_decc = 0.5 * length_of_section_ang;
                time_yaw_acc_decc = std::sqrt(2 * length_yaw_acc_decc / _yaw_acc_decc); // x = 0.5a*t^2 -> 2x/a = t^2 -> t = sqrt(2x/a)
                _yaw_vel_adjusted = _yaw_acc_decc * time_yaw_acc_decc;
                length_yaw_vel = 0.0;
                time_yaw_vel = 0.0;
            }

            duration_for_section_x = time_x_acc_decc + time_x_vel + time_x_acc_decc;
            duration_for_section_yaw = time_yaw_acc_decc + time_yaw_vel + time_yaw_acc_decc;
            duration_for_section = ros::Duration(std::max(duration_for_section_x, duration_for_section_yaw));

            ROS_DEBUG("_x_acc_decc: %f", _x_acc_decc);
            ROS_DEBUG("_x_vel: %f", _x_vel);
            ROS_DEBUG("length_of_section: %f", length_of_section);
            ROS_DEBUG("time_x_acc_decc: %f", time_x_acc_decc);
            ROS_DEBUG("time_x_vel: %f", time_x_vel);
            ROS_DEBUG("length_x_acc_decc: %f", length_x_acc_decc);
            ROS_DEBUG("length_x_vel: %f", length_x_vel);
            ROS_DEBUG("duration_for_section_x: %f", duration_for_section_x);

            ROS_DEBUG("_yaw_acc_decc: %f", _yaw_acc_decc);
            ROS_DEBUG("_yaw_vel: %f", _yaw_vel);
            ROS_DEBUG("length_of_section_ang: %f", length_of_section_ang);
            ROS_DEBUG("time_yaw_acc_decc: %f", time_yaw_acc_decc);
            ROS_DEBUG("time_yaw_vel: %f", time_yaw_vel);
            ROS_DEBUG("length_yaw_acc_decc: %f", length_yaw_acc_decc);
            ROS_DEBUG("length_yaw_vel: %f", length_yaw_vel);
            ROS_DEBUG("duration_for_section_yaw: %f", duration_for_section_yaw);

            ROS_DEBUG("duration_for_section: %f", duration_for_section.toSec());

            section_start_time = start_time;
            x_progress = 0.0;
            yaw_progress = 0.0;
            current_x_vel = 0.0;
            current_yaw_vel = 0.0;
            section_end_time = section_start_time + duration_for_section;
        }

        geometry_msgs::PoseStamped interpolate(double progress_ratio)
        {
            // """
            // Calculate where we should be along the section given a ratio of progress.
            // 0.0 means we're at the start, 1.0 means finished
            // :param progress_ratio: How far along the section are we?
            // :type progress_ratio: float
            // :return: an interpolation between the Section's start and end
            // :rtype: PoseStamped
            // """
            //# target_x = start_x + delta_x * progress_on_section
            double next_xyz[3];
            next_xyz[0] = _start_xyz[0] + _delta[0] * progress_ratio;
            next_xyz[1] = _start_xyz[1] + _delta[1] * progress_ratio;
            next_xyz[2] = _start_xyz[2] + _delta[2] * progress_ratio;
            double next_yaw = _start_yaw + _delta_yaw * progress_ratio;
            geometry_msgs::PoseStamped next_pose;
            //# next_pose.header.stamp is to be filled by the caller
            next_pose.header.frame_id = section_start_pose_stamped.header.frame_id;
            next_pose.pose.position.x = next_xyz[0];
            next_pose.pose.position.y = next_xyz[1];
            next_pose.pose.position.z = next_xyz[2];
            //# Compute orientation. PID can use it for holonomic robots
            tf::Quaternion quaternion;
            quaternion.setRPY(0, 0, next_yaw);
            quaternion = quaternion.normalize();
            next_pose.pose.orientation.x = quaternion.getX();
            next_pose.pose.orientation.y = quaternion.getY();
            next_pose.pose.orientation.z = quaternion.getZ();
            next_pose.pose.orientation.w = quaternion.getW();
            return next_pose;
        }

        tracking_pid::traj_point interpolate_with_acceleration(ros::Time current_time)
        {

            // """
            // Calculate where we should be along the section given a ratio of progress.
            // 0.0 means we're at the start, 1.0 means finished
            // :param progress_ratio: How far along the section are we?
            // :type progress_ratio: float
            // :return: an interpolation between the Section's start and end
            // :rtype: PoseStamped
            // """

            ros::Duration current_section_time = (current_time - section_start_time);
            double t = current_section_time.toSec();

            if (t < time_x_acc_decc)
            {
                double tr = t;
                x_progress = 0.5 * _x_acc_decc * tr * tr;
                current_x_vel = _x_acc_decc * tr;
                if (x_progress > length_x_acc_decc)
                {
                    current_x_vel = _x_vel_adjusted;
                    x_progress = length_x_acc_decc;
                }
            }
            else if (t < (time_x_acc_decc + time_x_vel))
            {
                double tr = (t - time_x_acc_decc);
                x_progress = length_x_acc_decc + current_x_vel * tr;
                current_x_vel = _x_vel_adjusted;
                if (x_progress > (length_x_acc_decc + length_x_vel))
                {
                    x_progress = (length_x_acc_decc + length_x_vel);
                }
            }
            else if (t < (time_x_acc_decc + time_x_vel + time_x_acc_decc))
            {
                double tr = (t - time_x_acc_decc - time_x_vel);
                x_progress = (length_x_acc_decc + length_x_vel) + _x_vel_adjusted * tr - 0.5 * _x_acc_decc * tr * tr;
                current_x_vel = _x_vel_adjusted - _x_acc_decc * tr;
                if (x_progress > length_of_section)
                {
                    current_x_vel = 0.0;
                    x_progress = length_of_section;
                }
            }
            else
            {
                current_x_vel = 0.0;
                x_progress = length_of_section;
            }

            if (t < time_yaw_acc_decc)
            {
                double tr = t;
                yaw_progress = 0.5 * _yaw_acc_decc * tr * tr;
                current_yaw_vel = _yaw_acc_decc * tr;
                if (yaw_progress > length_yaw_acc_decc)
                {
                    current_yaw_vel = _yaw_vel_adjusted;
                    yaw_progress = length_yaw_acc_decc;
                }
            }
            else if (t < (time_yaw_acc_decc + time_yaw_vel))
            {
                double tr = (t - time_yaw_acc_decc);
                yaw_progress = length_yaw_acc_decc + current_yaw_vel * tr;
                current_yaw_vel = _yaw_vel_adjusted;
                if (yaw_progress > (length_yaw_acc_decc + length_yaw_vel))
                {
                    yaw_progress = (length_yaw_acc_decc + length_yaw_vel);
                }
            }
            else if (t < (time_yaw_acc_decc + time_yaw_vel + time_yaw_acc_decc))
            {
                double tr = (t - time_yaw_acc_decc - time_yaw_vel);
                yaw_progress = (length_yaw_acc_decc + length_yaw_vel) + _yaw_vel_adjusted * tr - 0.5 * _yaw_acc_decc * tr * tr;
                current_yaw_vel = _yaw_vel_adjusted - _yaw_acc_decc * tr;
                if (yaw_progress > length_of_section_ang)
                {
                    current_yaw_vel = 0.0;
                    yaw_progress = length_of_section_ang;
                }
            }
            else
            {
                current_yaw_vel = 0.0;
                yaw_progress = length_of_section_ang;
            }

            double x_progress_ratio;
            double yaw_progress_ratio;

            if (length_of_section > 0)
            {
                x_progress_ratio = x_progress / length_of_section;
            }
            else
            {
                x_progress_ratio = 1.0;
            }

            if (length_of_section_ang > 0)
            {
                yaw_progress_ratio = yaw_progress / length_of_section_ang;
            }
            else
            {
                yaw_progress_ratio = 1.0;
            }
            //# target_x = start_x + delta_x * progress_on_section
            double next_xyz[3];
            next_xyz[0] = _start_xyz[0] + _delta[0] * x_progress_ratio;
            next_xyz[1] = _start_xyz[1] + _delta[1] * x_progress_ratio;
            next_xyz[2] = _start_xyz[2] + _delta[2] * x_progress_ratio;
            double next_yaw = _start_yaw + _delta_yaw * yaw_progress_ratio;
            tracking_pid::traj_point tp;
            tp.pose = geometry_msgs::PoseStamped();
            //# next_pose.header.stamp is to be filled by the caller
            tp.pose.header.frame_id = section_start_pose_stamped.header.frame_id;
            tp.pose.pose.position.x = next_xyz[0];
            tp.pose.pose.position.y = next_xyz[1];
            tp.pose.pose.position.z = next_xyz[2];
            tp.velocity.linear.x = current_x_vel;
            //# Compute orientation. PID can use it for holonomic robots
            tf::Quaternion quaternion;
            quaternion.setRPY(0, 0, next_yaw);
            tp.pose.pose.orientation.x = quaternion.x();
            tp.pose.pose.orientation.y = quaternion.y();
            tp.pose.pose.orientation.z = quaternion.z();
            tp.pose.pose.orientation.w = quaternion.w();
            tp.velocity.angular.z = sgn(_delta_yaw) * current_yaw_vel;

            return tp;
        }
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
        SectionInterpolation *_current_section;
        double progress_on_section;
        geometry_msgs::PoseStamped _latest_subgoal_pose;
        /**
         * """Accept and store the path"""
         * */
        void _process_path(nav_msgs::Path path_msg)
        {
            int path_size = path_msg.poses.size();
            ROS_DEBUG("_process_path(...). Path has %d poses.", path_size);

            // # Assert that all poses are defined in the same frame
            // # TODO: it would be nice to interpolate between poses defines in different frames but that is out of scope
            // # TODO: Tracking_pid completely disregards the frame_id and just does everything in the frame
            // #   it was configured for, even though it listens to Pose*Stamped*s

            if (!path_size)
            {
                ROS_WARN("There are no poses in the given path with header {}");
                return;
            }
            // # If empty frame_ids are supplied, use the global headers frame_id
            for (auto it = path_msg.poses.begin(); it != path_msg.poses.end(); ++it)
            {
                if (it->header.frame_id.empty())
                {
                    it->header.frame_id = path_msg.header.frame_id;
                }
            }
            // _path_pub.publish(path_msg)
            //  # Path is valid, so lets store it
            _latest_path_msg = path_msg;

            double target_x_vel;
            ros::param::param<double>("~target_x_vel", target_x_vel, 1.0);
            double target_x_acc;
            ros::param::param<double>("~target_x_acc", target_x_acc, 1.0);
            double target_yaw_vel;
            ros::param::param<double>("~target_yaw_vel", target_yaw_vel, 1.0);
            double target_yaw_acc;
            ros::param::param<double>("~target_yaw_acc", target_yaw_acc, 1.0);

            if (target_x_vel == 0.0 || target_yaw_vel == 0.0)
            {
                ROS_WARN("Ignoring ~target_x_vel of %f, ~target_yaw_vel of %f, keeping %f, %f, consider using the pause function", target_x_vel, target_yaw_vel, _target_x_vel, _target_yaw_vel);
            }
            else
            {
                _target_x_vel = target_x_vel;
                _target_x_acc = target_x_acc;
                _target_yaw_vel = target_yaw_vel;
                _target_yaw_acc = target_yaw_acc;
            }
            _path_poses = path_msg.poses;
            // _sections = list(zip(_path_poses, _path_poses[1:]))
            _sections.clear();
            for (int i = 0; i < _path_poses.size() - 1; i++)
            {
                std::vector<geometry_msgs::PoseStamped> pair;
                pair.push_back(_path_poses[i]);
                pair.push_back(_path_poses[i + 1]);
                _sections.push_back(pair);
            }
        }

        void _update_target(tracking_pid::traj_point &tp)
        {
            // """
            // Called by _timer and determines & publishes a interpolated pose along the received Path

            // :param event: supplied by rospy.Timer, the time at which the callback method is called
            // :type event: rospy.TimerEvent (see https://wiki.ros.org/rospy/Overview/Time#Timer)
            // :return:
            // """
            // # event is supplied by the timer and contains info like the current time and time since last tick
            // # based on the time, the start time of the subsection and the target_velocities, we can calculate where the target should be:
            // # >>> duration_on_section = (current_time - section_start_time)
            // # >>> duration_for_section = (length_of_section / target_(x_yaw)velocity)
            // # >>> progress_on_section = (duration_on_section / duration_for_section)
            // # >>> target_x = start_x + delta_x * progress_on_section

            ros::Time current_real = ros::Time::now();

            if (_path_poses.empty())
            {
                ROS_DEBUG_THROTTLE(1.0, "No path poses set");
                return;
            }

            if (!_current_section or ros::Time::now() > _current_section->section_end_time)
            { //  # or when past end time of current section, go to next
                if (_sections.size() > 0)
                {
                    auto start_end = _sections[0];
                    geometry_msgs::PoseStamped start = start_end[0];
                    geometry_msgs::PoseStamped end = start_end[1];
                    _sections.erase(_sections.begin());
                    if (_current_section)
                    {
                        delete _current_section;
                    }
                    _current_section = new SectionInterpolation(start, end, current_real, _target_x_vel, _target_x_acc, _target_yaw_vel, _target_yaw_acc);
                    ROS_INFO("Starting new section. duration_for_section = %f", _current_section->duration_for_section.toSec());

                    // if flip_for_axis:
                    //     # Have the control point in front if we are to drive forwards, control point in the back when driving backwards
                    //     # This makes the robot always keep point up, for Emma that always needs to keep pointing up
                    //     _set_controller_direction(sign=np.sign(_current_section->delta[axes[flip_for_axis]]))
                }
                else
                {
                    ROS_DEBUG("Path ended");
                    // stop_path();
                    return;

                    // loop = rospy.get_param("~loop", 0)
                    // if (loop != 0) {
                    //     rospy.loginfo("~loop = {}, starting path again".format(loop))
                    //     rospy.set_param("~loop", loop-1)
                    //     _process_path(_latest_path_msg)
                    // } else {
                    //     rospy.logdebug("No loop requested or remaining, finishing up")
                    //     _pub_finished.publish(True)
                    //     if _as.is_active():
                    //         _as.set_succeeded()
                    //     stop_path()
                    // return;
                    // }
                }
            }

            ros::Duration duration_on_section = current_real - _current_section->section_start_time;

            // # Distance between duplicated poses is 0, so we can't do the division below.
            // # If the duration is 0, then we're done immediately.
            // # This will still be valid when this node starts taking rotation and angular velocity into account
            // # Then, even when the pose.position does not change while pose.orientation does change,
            // #  the angular velocity will make the section have a nonzero duration
            if (!_current_section->duration_for_section.isZero())
            {
                progress_on_section = (duration_on_section.toSec() / _current_section->duration_for_section.toSec());
            }
            else
            {
                ROS_INFO("Instantaneous completion of 0-length section");
                progress_on_section = 1;
            }
            tp = _current_section->interpolate_with_acceleration(ros::Time::now());
            tp.pose.header.stamp = current_real;

            // # TODO: Rotate in the corners, using controller mode 3 tp.controller.data = 3

            // # Remember the last interpolated sub-goal on our way to the next waypoint in the Path
            _latest_subgoal_pose = tp.pose;

            // __publish_marker(tp.pose)
            // trajectory_pub.publish(tp)
        }
    };
}
