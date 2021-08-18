
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_local_planner.h>

namespace tracking_pid {
    class TrackingPidLocalPlanner : public nav_core::BaseLocalPlanner {

    public:
      /**
       * @brief  Default constructor for the ros wrapper
       */
      TrackingPidLocalPlanner();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      TrackingPidLocalPlanner(std::string name,
                           tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~TrackingPidLocalPlanner();
      
      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    };
}

PLUGINLIB_EXPORT_CLASS(tracking_pid::TrackingPidLocalPlanner, nav_core::BaseLocalPlanner)

namespace tracking_pid {

}