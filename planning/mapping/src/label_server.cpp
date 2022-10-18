#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <grid_map_ros/grid_map_ros.hpp>

class MappingServer {
public:
    MappingServer(ros::NodeHandle& _nh, ros::NodeHandle& _pnh) : nh(_nh), pnh(_pnh), map_(std::vector<std::string>({"occupancy"})) {
        // Parameters
        ROS_ASSERT(pnh.getParam("fixed_frame_id", FIXED_FRAME_ID));
        ROS_ASSERT(pnh.getParam("resolution", RESOLUTION));
        ROS_ASSERT(pnh.getParam("max_range", MAXIMUM_RANGE));
        ROS_ASSERT(pnh.getParam("hit_logprob", HIT_LOGPROB));
        ROS_ASSERT(pnh.getParam("miss_logprob", MISS_LOGPROB));
        ROS_ASSERT(pnh.getParam("max_logprob", MAX_LOGPROB));
        ROS_ASSERT(pnh.getParam("min_logprob", MIN_LOGPROB));
        // HIT_PROB  = std::max(0.0, std::min(HIT_PROB,  1.0));
        // MISS_PROB = std::max(0.0, std::min(MISS_PROB, 1.0));

        std::string lidar_topic_name;
        std::string ocupancy_grid_topic_name;

        ROS_ASSERT(pnh.getParam("rgbPoints", lidar_topic_name));
        ROS_ASSERT(pnh.getParam("labelmap", ocupancy_grid_topic_name));
        ROS_ERROR_STREAM("labels: " << lidar_topic_name << " labels_map: " <<  ocupancy_grid_topic_name);
        ROS_ASSERT(!lidar_topic_name.empty() && !ocupancy_grid_topic_name.empty());

        occupied_cells_publisher = nh.advertise<nav_msgs::OccupancyGrid>(ocupancy_grid_topic_name, 1);

        nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic_name, 1, &MappingServer::update_occupancy_map, this);
        // pointcloud_subscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, lidar_topic_name, 1);
        // tf_pointcloud_subscriber = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pointcloud_subscriber, tf_listener, FIXED_FRAME_ID, 1);
        // tf_pointcloud_subscriber->registerCallback(boost::bind(&MappingServer::update_occupancy_map, this, _1));

        map_.setGeometry(grid_map::Length(MAXIMUM_RANGE, MAXIMUM_RANGE), RESOLUTION, grid_map::Position(0.0, 0.0));
        map_.setFrameId(FIXED_FRAME_ID);
    }

    ~MappingServer() {
        delete pointcloud_subscriber;
    }

    /*
     * Update the occupancies of grid map from given pointcloud.
     * A pointcloud subscriber registers this callback function to update the map
     * when a new sensor measurement is observed.
     *
     * @param src_pc: a pointcloud in the sensor coordinate
     */
    void update_occupancy_map(const sensor_msgs::PointCloud2ConstPtr& _src_pc) {
        parse_valid_sensor_measurement(*_src_pc);
        publish_occupied_cells();
    }

    /*
     * Publish the occupied cells for visualization in RViz.
     * As a simple message type, the function publishes a set of centers of occupied cells.
     *
     */
    void publish_occupied_cells() {
        nav_msgs::OccupancyGrid occupancyGrid;
        grid_map::GridMapRosConverter::toOccupancyGrid(map_, "occupancy", MIN_LOGPROB, MAX_LOGPROB, occupancyGrid);
        occupancyGrid.header.frame_id = FIXED_FRAME_ID;
        occupied_cells_publisher.publish(occupancyGrid);
    }


protected:
    // Node handle
    ros::NodeHandle &nh;
    ros::NodeHandle &pnh;
    // Transform and pointcloud subscribers
    message_filters::Subscriber<sensor_msgs::PointCloud2>*  pointcloud_subscriber;
    tf::MessageFilter<sensor_msgs::PointCloud2>*            tf_pointcloud_subscriber;
    tf::TransformListener                                   tf_listener;

    // Options =========================================================================================================
protected:
    std::string  FIXED_FRAME_ID;
    double RESOLUTION;
    double MAXIMUM_RANGE;
    double HIT_LOGPROB;
    double MISS_LOGPROB;
    double MAX_LOGPROB;
    double MIN_LOGPROB;
    ros::Publisher occupied_cells_publisher;

    //! Grid map data.
    grid_map::GridMap map_;

    /*
     * Parse the valid points in the sensor measurement
     * and transform the data from the sensor coordinate to the world coordinate.
     *
     * @param ros_pc: [input]  point cloud in the sensor coordinate
     * @param pc:     [output] point cloud consisting of valid points in the world coordinate
     * @param origin: [output] sensor origin in the world coordinate
     * @return validity of the sensor measurement
     */
    void parse_valid_sensor_measurement(const sensor_msgs::PointCloud2& _ros_pc) {
        // Pose of the sensor frame
        tf::StampedTransform sensor_to_world;
        try{
            tf_listener.lookupTransform(FIXED_FRAME_ID, _ros_pc.header.frame_id, _ros_pc.header.stamp, sensor_to_world);
        }
        catch(tf::TransformException& e) {
            ROS_ERROR_STREAM("Cannot find a transform from sensor to world: " << _ros_pc.header.frame_id << " --> " << FIXED_FRAME_ID);
            // return false;
        }

        // in the sensor coordinate ====================================================================================
        pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud;
        pcl::fromROSMsg(_ros_pc, pcl_pointcloud);

        pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud_in_sensor_coordinate;
        for(const auto& point : pcl_pointcloud) {
            // Remove the invalid data: NaN
            if(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
                continue;

            // Remove the invalid data: out of sensing range
            double l = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if(l > MAXIMUM_RANGE)
                continue;

            pcl_pointcloud_in_sensor_coordinate.push_back(point);
        }

        // in the world coordinate =====================================================================================
        Eigen::Matrix4f transform;
        pcl_ros::transformAsMatrix(sensor_to_world, transform);
        pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud_in_world_coordinate;
        pcl::transformPointCloud(pcl_pointcloud_in_sensor_coordinate, pcl_pointcloud_in_world_coordinate, transform);

        double or_x = sensor_to_world.getOrigin().x();
        double or_y = sensor_to_world.getOrigin().y();

        for(const auto& point : pcl_pointcloud_in_world_coordinate) {
            updateMap(point, or_x, or_y);
        }

    }

    void updateMap(const pcl::PointXYZRGB &point_pose, double origin_x, double origin_y) {

        grid_map::Position end(point_pose.x, point_pose.y);

        grid_map::Index subIndex;
        if (!map_.getIndex(end, subIndex)) {
            return;
        }
        float lab = point_pose.r;
        if (lab < 0 || lab > 100) lab = NAN;
        map_.at("occupancy", subIndex) = lab;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "label_server");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    MappingServer mapping_server(nh,nhp);

    try{
        ros::spin();
    }
    catch(std::runtime_error& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    return 0;
}