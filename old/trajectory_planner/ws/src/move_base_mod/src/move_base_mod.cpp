/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: aparfenov
 *********************************************************************/
#include <move_base/move_base_mod.h>
 // #include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>

#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base_mod {

    MoveBase::MoveBase(tf2_ros::Buffer & tf):
        tf_(tf),
        as_(NULL),
        controller_costmap_ros_(NULL),
        blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
        new_global_plan_(false) {

            as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base_mod", boost::bind( & MoveBase::executeCb, this, _1), false);

            ros::NodeHandle private_nh("~");
            ros::NodeHandle nh;

            //get some parameters that will be global to the move base node
            std::string local_planner;
            private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
            private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
            private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
            private_nh.param("controller_frequency", controller_frequency_, 20.0);

            //for commanding the base
            vel_pub_ = nh.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
            // current_path_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_path", 0 );

            // ros::NodeHandle action_nh("move_base_mod");
            // action_goal_pub_ = action_nh.advertise < move_base_mod::TrajectoryControllerGoal > ("goal", 1);

            //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
            //they won't get any useful information back about its status, but this is useful for tools
            //like nav_view and rviz
            // ros::NodeHandle simple_nh("move_base_mod_simple");
            // goal_sub_ = simple_nh.subscribe < geometry_msgs::PoseStamped > ("goal", 1, boost::bind( & MoveBase::goalCB, this, _1));

            //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
            controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
            controller_costmap_ros_->pause();

            //create a local planner
            try {
                tc_ = blp_loader_.createInstance(local_planner);
                ROS_INFO("Created local_planner %s", local_planner.c_str());
                tc_->initialize(blp_loader_.getName(local_planner), & tf_, controller_costmap_ros_);
            } catch (const pluginlib::PluginlibException & ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
                exit(1);
            }

            // Start actively updating costmaps based on sensor data
            controller_costmap_ros_->start();

            as_->start();

            // dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
            // dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
            // dsrv_->setCallback(cb);
        }

    // void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr & goal) {
    //     ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    //     move_base_mod::TrajectoryControllerGoal action_goal;
    //     geometry_msgs::PoseStamped target_pose = * goal;

    //     target_pose.header.stamp = ros::Time::now();
    //     std::vector < geometry_msgs::PoseStamped > path;
    //     // TODO: make path from goal
    //     path.push_back(target_pose);
    //     action_goal.poses = path;

    //     action_goal_pub_.publish(action_goal);
    // }

    MoveBase::~MoveBase() {

        if (as_ != NULL)
            delete as_;

        if (controller_costmap_ros_ != NULL)
            delete controller_costmap_ros_;

        tc_.reset();
    }

    void MoveBase::publishZeroVelocity() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }

    bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion & q) {
        //first we need to check if the quaternion has nan's or infs
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

        //next, we need to check if the length of the quaternion is close to zero
        if (tf_q.length2() < 1e-6) {
            ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
            return false;
        }

        //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        tf_q.normalize();

        tf2::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        if (fabs(dot - 1) > 1e-3) {
            ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
            return false;
        }

        return true;
    }

    // std::vector<geometry_msgs::PoseStamped> MoveBase::pathToGlobalFrame(const std::vector<geometry_msgs::PoseStamped> &path) {
    //     std::vector<geometry_msgs::PoseStamped> ret;
    //     for (auto it = path.begin(); it != path.end(); ++it) {
    //       ret.push_back(this->goalToGlobalFrame(*it));
    //   }
    // }

    // geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    //     std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    //     geometry_msgs::PoseStamped goal_pose, global_pose;
    //     goal_pose = goal_pose_msg;

    //     //just get the latest available transform... for accuracy they should send
    //     //goals in the frame of the planner
    //     goal_pose.header.stamp = ros::Time();

    //     try{
    //         tf_.transform(goal_pose_msg, global_pose, global_frame);
    //     }
    //     catch(tf2::TransformException& ex){
    //       ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
    //           goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
    //       return goal_pose_msg;
    //     }
    //   return global_pose;
    // }

    void MoveBase::executeCb(const move_base_mod::TrajectoryControllerGoalConstPtr & move_base_goal) {
        // if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
        //   as_->setAborted(move_base_mod::TrajectoryControllerResult(), "Aborting on goal because it was sent with an invalid quaternion");
        //   return;
        // }

        //TODO: why do pathToGlobalFrame ?
        // std::vector<geometry_msgs::PoseStamped> goal = pathToGlobalFrame(move_base_goal->poses);
        std::vector < geometry_msgs::PoseStamped > goal = move_base_goal->poses;

        publishZeroVelocity();

        ros::Rate r(controller_frequency_);

        new_global_plan_ = true;

        ros::NodeHandle n;
        while (n.ok()) {

            if (as_->isPreemptRequested()) {
                if (as_->isNewGoalAvailable()) {
                    //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                    move_base_mod::TrajectoryControllerGoal new_goal = * as_->acceptNewGoal();

                    // if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
                    //   as_->setAborted(move_base_mod::TrajectoryControllerResult(), "Aborting on goal because it was sent with an invalid quaternion");
                    //   return;
                    // }

                    // goal = pathToGlobalFrame(new_goal.poses);
                    goal = new_goal.poses;
                    new_global_plan_ = true;

                    //publish the goal point to the visualizer
                    // ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
                    // current_path_pub_.publish(goal);
                } else {
                    //if we've been preempted explicitly we need to shut things down
                    resetState();

                    //notify the ActionServer that we've successfully preempted
                    ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");
                    as_->setPreempted();

                    //we'll actually return from execute after preempting
                    return;
                }
            }

            //for timing that gives real time even in simulation
            ros::WallTime start = ros::WallTime::now();

            //the real work on pursuing a goal is done here
            bool done = executeCycle(goal);

            //if we're done, then we'll return from execute
            if (done)
                return;

            ros::WallDuration t_diff = ros::WallTime::now() - start;
            ROS_DEBUG_NAMED("move_base", "Full control cycle time: %.9f\n", t_diff.toSec());

            r.sleep();
            //make sure to sleep for the remainder of our cycle time
            if (r.cycleTime() > ros::Duration(1 / controller_frequency_))
                ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
        } // while
        //if the node is killed then we'll abort and return
        as_->setAborted(move_base_mod::TrajectoryControllerResult(), "Aborting on the goal because the node has been killed");
        return;
    }

    // double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
    // {
    //   return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    // }

    bool MoveBase::executeCycle(std::vector < geometry_msgs::PoseStamped > & goal) {
        // boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        //we need to be able to publish velocity commands
        geometry_msgs::Twist cmd_vel;

        //TODO: update feedback to correspond to our curent position
        // geometry_msgs::PoseStamped global_pose;
        // getRobotPose(global_pose, planner_costmap_ros_);
        // const geometry_msgs::PoseStamped& current_position = global_pose;

        // //push the feedback out
        // move_base_mod::TrajectoryControllerFeedback feedback;
        // feedback.base_position = current_position;
        // as_->publishFeedback(feedback);

        //if we have a new plan then grab it and give it to the controller
        if (new_global_plan_) {
            //make sure to set the new plan flag to false
            new_global_plan_ = false;

            ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

            if (!tc_->setPlan(goal)) {
                // //ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();
                as_->setAborted(move_base_mod::TrajectoryControllerResult(), "Failed to pass global plan to the controller.");
                return true;
            }

        }

        if (tc_->isGoalReached()) {
            ROS_DEBUG_NAMED("move_base", "Goal reached!");
            resetState();

            as_->setSucceeded(move_base_mod::TrajectoryControllerResult(), "Goal reached.");
            return true;
        }

        {
            boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock( * (controller_costmap_ros_->getCostmap()->getMutex()));

            if (tc_->computeVelocityCommands(cmd_vel)) {
                ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
                // last_valid_control_ = ros::Time::now();
                //make sure that we send the velocity command to the base
                vel_pub_.publish(cmd_vel);
            } else {
                ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                // ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
                resetState();

                as_->setAborted(move_base_mod::TrajectoryControllerResult(), "The local planner could not find a valid plan.");
                return true;
            }

            //we aren't done yet
            return false;
        }
    }

    void MoveBase::resetState() {
        publishZeroVelocity();
    }

    bool MoveBase::getRobotPose(geometry_msgs::PoseStamped & global_pose, costmap_2d::Costmap2DROS * costmap) {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = ros::Time(); // latest available
        ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

        // get robot pose on the given costmap frame
        try {
            tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        } catch (tf2::LookupException & ex) {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        } catch (tf2::ConnectivityException & ex) {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        } catch (tf2::ExtrapolationException & ex) {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        // check if global_pose time stamp is within costmap transform tolerance
        if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance()) {
            ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. "\
                "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
            return false;
        }

        return true;
    }
};
