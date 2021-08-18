
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_local_planner.h>

#include "controller/controller.h"
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <stdio.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tracking_pid/PidConfig.h>
#include <tracking_pid/PidDebug.h>
#include <tracking_pid/traj_point.h>
#include <tracking_pid/trajectory.h>
#include <vector>
#include <visualization_msgs/Marker.h>

namespace tracking_pid
{
    class TrackingPidLocalPlanner : public nav_core::BaseLocalPlanner
    {

    public:
        // Generic pose variables
        geometry_msgs::Transform tfCurPose;
        tf::Transform tfControlPose = tf::Transform::getIdentity();
        tf::Transform tfGoalPose = tf::Transform::getIdentity();
        tf::Transform tfErrorPose = tf::Transform::getIdentity();
        tf::StampedTransform tfMapToOdom;
        tf2_ros::Buffer *tf_buffer;

        geometry_msgs::Pose controlPose;
        geometry_msgs::Pose goalPose;
        tracking_pid::traj_point goalPoint;
        tf::Vector3 newOrigin;

        // Frame names
        std::string map_frame;
        std::string base_link_frame;

        // Errors
        double error_x = 0;
        double error_y = 0;
        double error_th = 0;

        // For timing
        ros::Time prev_time;
        ros::Duration delta_t;

        // Controller output
        double control_effort_long = 0.0; // output of pid controller
        double control_effort_lat = 0.0;  // output of pid controller
        double control_effort_ang = 0.0;  // output of pid controller
        geometry_msgs::Twist output_combined;

        // Controller logic
        int controlType = 0;
        bool controller_enabled = true;
        bool enabled_on_boot = true;
        bool waiting_for_setpoint = false;
        bool feedback_long_enabled = false;
        bool feedback_lat_enabled = false;
        bool feedback_ang_enabled = false;
        bool feedforward_long_enabled = false;
        bool feedforward_lat_enabled = false;
        bool feedforward_ang_enabled = false;
        bool controller_debug_enabled = false;
        bool holonomic_robot = false;
        bool track_base_link = false;

        // Primary feedback controller parameters
        double Kp_long = 0, Ki_long = 0, Kd_long = 0;
        double Kp_lat = 0, Ki_lat = 0, Kd_lat = 0;
        double Kp_ang = 0, Ki_ang = 0, Kd_ang = 0;
        double l = 0.0;

        // feedforward controller

        double feedforward_long = 0;
        double feedforward_lat = 0;
        double feedforward_ang = 0;
        double xvel = 0.0;
        double yvel = 0.0;
        double thvel = 0.0;
        double theta_cp = 0.0;

        // Parameters for error calc. with disconinuous input
        bool angle_error = false;
        double angle_wrap = 2.0 * 3.14159;

        // Cutoff frequency for the derivative calculation in Hz.
        // Negative -> Has not been set by the user yet, so use a default.
        double cutoff_frequency_long = -1;
        double cutoff_frequency_lat = -1;
        double cutoff_frequency_ang = -1;

        // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
        // 1/4 of the sample rate.
        double c_long = 1.;
        double c_lat = 1.;
        double c_ang = 1.;

        // Used to check for tan(0)==>NaN in the filter calculation
        double tan_filt = 1.;

        // Upper and lower saturation limits
        double upper_limit = 1000, lower_limit = -1000;

        // Anti-windup term. Limits the absolute value of the integral term.
        double windup_limit = 1000;

        // Initialize filter data with zeros
        std::vector<double> error_long = std::vector<double>(3, 0);
        std::vector<double> filtered_error_long = std::vector<double>(3, 0);
        std::vector<double> error_deriv_long = std::vector<double>(3, 0);
        std::vector<double> filtered_error_deriv_long = std::vector<double>(3, 0);
        std::vector<double> error_lat = std::vector<double>(3, 0);
        std::vector<double> filtered_error_lat = std::vector<double>(3, 0);
        std::vector<double> error_deriv_lat = std::vector<double>(3, 0);
        std::vector<double> filtered_error_deriv_lat = std::vector<double>(3, 0);
        std::vector<double> error_ang = std::vector<double>(3, 0);
        std::vector<double> filtered_error_ang = std::vector<double>(3, 0);
        std::vector<double> error_deriv_ang = std::vector<double>(3, 0);
        std::vector<double> filtered_error_deriv_ang = std::vector<double>(3, 0);

        // Temporary variables
        double proportional_long = 0;          // proportional term of output
        double integral_long = 0;              // integral term of output
        double derivative_long = 0;            // derivative term of output
        double proportional_lat = 0;           // proportional term of output
        double integral_lat = 0;               // integral term of output
        double derivative_lat = 0;             // derivative term of output
        double proportional_ang = 0;           // proportional term of output
        double integral_ang = 0;               // integral term of output
        double derivative_ang = 0;             // derivative term of output
        double steadySetPointThreshold = 0.01; // Threshold in degrees on the linear velocity output
        double error_integral_lat = 0;
        double error_integral_long = 0;
        double error_integral_ang = 0;
        double theta_rp = 0.0;

        // Topic and node names and message objects
        // ros::Publisher control_effort_pub;
        ros::Subscriber sub_trajectory;
        // ros::Subscriber subs_odom;
        ros::ServiceServer enable_service;
        ros::ServiceServer enable_and_wait_service;

        // Debugging of controller
        ros::Publisher debug_pub;
        // Rviz visualization
        ros::Publisher marker_pub;
        visualization_msgs::Marker mkCurPose, mkControlPose, mkGoalPose;
        geometry_msgs::Point p;

        // For Screen output
        std::string topic_from_controller, topic_from_plant, setpoint_topic, pid_enable_topic, node_name = "pid_node";

        // Diagnostic objects
        double min_loop_frequency = 1, max_loop_frequency = 1000;
        int measurements_received = 0;

        tracking_pid::Controller pid_controller;
        dynamic_reconfigure::Server<tracking_pid::PidConfig> *dsrv_;

        void publishMarkers()
        {
            p.x = tfCurPose.translation.x;
            p.y = tfCurPose.translation.y;
            p.z = tfCurPose.translation.z;
            mkCurPose.points.push_back(p);

            p.x = tfControlPose.getOrigin().x();
            p.y = tfControlPose.getOrigin().y();
            p.z = tfControlPose.getOrigin().z();
            mkControlPose.points.push_back(p);

            p.x = tfGoalPose.getOrigin().x();
            p.y = tfGoalPose.getOrigin().y();
            p.z = tfGoalPose.getOrigin().z();
            mkGoalPose.points[0] = p;

            marker_pub.publish(mkCurPose);
            marker_pub.publish(mkControlPose);
            marker_pub.publish(mkGoalPose);
        }

        bool enableCallback(std_srvs::SetBool::Request &req,  // NOLINT
                            std_srvs::SetBool::Response &res) // NOLINT
        {
            controller_enabled = req.data;
            waiting_for_setpoint = false;
            // TODO(Nobleo): Add wait dfor new Setp
            pid_controller.setEnabled(controller_enabled);
            res.success = true;
            if (controller_enabled)
                res.message = "Controller enabled";
            else
                res.message = "Controller disabled";
            return true;
        }

        bool enableAndWaitCallback(std_srvs::SetBool::Request &req,  // NOLINT
                                   std_srvs::SetBool::Response &res) // NOLINT
        {
            controller_enabled = req.data;
            waiting_for_setpoint = true;
            // TODO(Nobleo): Add wait dfor new Setp
            pid_controller.setEnabled(controller_enabled);
            res.success = true;
            if (controller_enabled)
                res.message = "Controller enabled. Wait for Trajectory";
            else
                res.message = "Controller disabled";
            return true;
        }

        void trajectory_callback(const tracking_pid::traj_point &goalPointMsg)
        {
            goalPoint = goalPointMsg;
            try
            {
                goalPoint.pose = tf_buffer->transform(goalPoint.pose, map_frame);
                waiting_for_setpoint = false;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                return;
            }
            tfGoalPose = tf::Transform(
                tf::Quaternion(goalPoint.pose.pose.orientation.x, goalPoint.pose.pose.orientation.y,
                               goalPoint.pose.pose.orientation.z, goalPoint.pose.pose.orientation.w),
                tf::Vector3(goalPoint.pose.pose.position.x, goalPoint.pose.pose.position.y, goalPoint.pose.pose.position.z));
            if (goalPoint.controller.data != controlType)
            {
                controlType = goalPoint.controller.data;
                pid_controller.selectMode((tracking_pid::ControllerMode)controlType);
            }
        }

        void poseCallback(geometry_msgs::Twist &cmd_vel)
        {
            try
            {
                tf_buffer->canTransform(base_link_frame, map_frame, ros::Time(0), ros::Duration(10.0));
            }
            catch (tf2::TransformException ex)
            {
                ROS_ERROR("Received an exception trying to transform: %s", ex.what());
            }

            // calculate delta_t
            if (!prev_time.isZero()) // Not first time through the program
            {
                delta_t = ros::Time::now() - prev_time;
                prev_time = ros::Time::now();
                if (0 == delta_t.toSec())
                {
                    ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
                    return;
                }
            }
            else
            {
                ROS_DEBUG("prev_time is 0, doing nothing");
                prev_time = ros::Time::now();
                return;
            }

            if (!waiting_for_setpoint)
            {
                tracking_pid::PidDebug pid_debug;
                geometry_msgs::TransformStamped tfCurPoseStamped;
                tfCurPoseStamped = tf_buffer->lookupTransform(map_frame, base_link_frame, ros::Time(0));
                tfCurPose = tfCurPoseStamped.transform;
                cmd_vel = pid_controller.update(tfCurPose, tfGoalPose, delta_t, &pid_debug);

                // if (controller_enabled)
                // {
                //     control_effort_pub.publish(cmd_vel);
                // }

                if (controller_debug_enabled)
                {
                    debug_pub.publish(pid_debug);
                }
            }
        }

        void reconfigure_callback(const tracking_pid::PidConfig &config)
        {
            pid_controller.configure(config);
            controller_debug_enabled = config.controller_debug_enabled;
        }

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros)
        {
            ros::NodeHandle node_priv("~/" + name);
            ros::NodeHandle node;

            tf_buffer = tf;
            // costmap_ros_ = costmap_ros;

            // Get params if specified in launch file or as params on command-line, set defaults
            node_priv.param<std::string>("map_frame", map_frame, "map");
            node_priv.param<std::string>("base_link_frame", base_link_frame, "base_link");
            node_priv.param<bool>("holonomic_robot", holonomic_robot, false);
            pid_controller.setHolonomic(holonomic_robot);
            node_priv.param<bool>("track_base_link", track_base_link, false);
            pid_controller.setTrackBaseLink(track_base_link);
            node_priv.param<bool>("enabled_on_boot", enabled_on_boot, true);
            controller_enabled = enabled_on_boot;
            waiting_for_setpoint = true;

            // instantiate publishers & subscribers
            // control_effort_pub = node.advertise<geometry_msgs::Twist>("move_base/cmd_vel", 1);
            sub_trajectory = node.subscribe("local_trajectory", 1, &TrackingPidLocalPlanner::trajectory_callback, this);
            enable_service = node.advertiseService("enable_control", &TrackingPidLocalPlanner::enableCallback, this);
            enable_and_wait_service = node.advertiseService("enable_control_and_wait", &TrackingPidLocalPlanner::enableAndWaitCallback, this);

            marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

            debug_pub = node.advertise<tracking_pid::PidDebug>("debug", 1);

            // configure dynamic reconfiguration
            //   dynamic_reconfigure::Server<tracking_pid::PidConfig> config_server;
            //   dynamic_reconfigure::Server<tracking_pid::PidConfig>::CallbackType f;
            //   f = boost::bind(&reconfigure_callback, _1);
            //   config_server.setCallback(f);

            dsrv_ = new dynamic_reconfigure::Server<tracking_pid::PidConfig>(node_priv);
            dsrv_->setCallback(boost::bind(&TrackingPidLocalPlanner::reconfigure_callback, this, _1));

            // configure rviz visualization
            mkCurPose.header.frame_id = mkControlPose.header.frame_id = mkGoalPose.header.frame_id = map_frame;
            mkCurPose.header.stamp = mkControlPose.header.stamp = mkGoalPose.header.stamp = ros::Time::now();
            mkCurPose.ns = "axle point";
            mkControlPose.ns = "control point";
            mkGoalPose.ns = "goal point";
            mkCurPose.action = mkControlPose.action = mkGoalPose.action = visualization_msgs::Marker::ADD;
            mkCurPose.pose.orientation.w = mkControlPose.pose.orientation.w = mkGoalPose.pose.orientation.w = 1.0;
            mkCurPose.id = 0;
            mkControlPose.id = 1;
            mkGoalPose.id = 2;
            mkCurPose.type = mkControlPose.type = mkGoalPose.type = visualization_msgs::Marker::POINTS;
            mkCurPose.scale.x = 0.1;
            mkCurPose.scale.y = 0.1;
            mkControlPose.scale.x = 0.02;
            mkControlPose.scale.y = 0.02;
            mkGoalPose.scale.x = 0.1;
            mkGoalPose.scale.y = 0.1;
            mkCurPose.color.b = 1.0;
            mkCurPose.color.a = 1.0;
            mkControlPose.color.g = 1.0f;
            mkControlPose.color.a = 1.0;
            mkGoalPose.color.r = 1.0;
            mkGoalPose.color.a = 1.0;
            mkGoalPose.points.resize(1);

            // tf2_ros::TransformListener tf_listener(tf_buffer);
        }

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
        {
            return true;
        }

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
        {
            // Get current robot pose and spin controller
            poseCallback(cmd_vel);

            // Publish some poses as markers to be visualized in Rviz
            publishMarkers();
            return true;
        }

        bool isGoalReached()
        {
            return false;
        }

        ~TrackingPidLocalPlanner()
        {
            //make sure to clean things up
            delete dsrv_;
        }
    };
}

PLUGINLIB_EXPORT_CLASS(tracking_pid::TrackingPidLocalPlanner, nav_core::BaseLocalPlanner)
