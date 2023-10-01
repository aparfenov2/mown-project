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
#ifndef NAV_MOVE_BASE_MOD_ACTION_H_
#define NAV_MOVE_BASE_MOD_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_mod/TrajectoryControllerAction.h>

#include <nav_core/base_local_planner.h>
// #include <nav_core/base_global_planner.h>
// #include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

// #include <dynamic_reconfigure/server.h>
// #include "move_base/MoveBaseConfig.h"

namespace move_base_mod {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_mod::TrajectoryControllerAction> MoveBaseActionServer;

  class MoveBase {
  public:
    MoveBase(tf2_ros::Buffer& tf);
    virtual ~MoveBase();
    bool executeCycle(std::vector<geometry_msgs::PoseStamped> &goal);

private:
    void publishZeroVelocity();

    void resetState();

    // void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

    void executeCb(const move_base_mod::TrajectoryControllerGoalConstPtr& move_base_goal);

    bool isQuaternionValid(const geometry_msgs::Quaternion& q);

    bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      // double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

    // geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
    // std::vector<geometry_msgs::PoseStamped> pathToGlobalFrame(const std::vector<geometry_msgs::PoseStamped> &path);

    tf2_ros::Buffer& tf_;

    MoveBaseActionServer* as_;

    boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
    costmap_2d::Costmap2DROS* controller_costmap_ros_;

    std::string robot_base_frame_, global_frame_;

    double controller_frequency_;
    ros::Publisher vel_pub_;
    // ros::Subscriber goal_sub_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
    bool new_global_plan_;
};
};
#endif

