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

class MappingServer
{
public:
    MappingServer(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) :
        nh(_nh), pnh(_pnh), map_(std::vector<std::string>({"occupancy", "color"}))
    {
        // Parameters
        ROS_ASSERT(pnh.getParam("fixed_frame_id", FIXED_FRAME_ID));
        ROS_ASSERT(pnh.getParam("resolution", RESOLUTION));
        ROS_ASSERT(pnh.getParam("max_range", MAXIMUM_RANGE));
        ROS_ASSERT(pnh.getParam("hit_logprob", HIT_LOGPROB));
        ROS_ASSERT(pnh.getParam("miss_logprob", MISS_LOGPROB));
        ROS_ASSERT(pnh.getParam("max_logprob", MAX_LOGPROB));
        ROS_ASSERT(pnh.getParam("min_logprob", MIN_LOGPROB));

        // read list of allowed colors from segmented point cloud
        // XmlRpc::XmlRpcValue colors;
        // ROS_ASSERT(pnh.getParam("allowed_colors", colors));
        // ROS_ASSERT(symbols.getType()==XmlRpc::XmlRpcValue::TypeArray);
        // for (XmlRpc::XmlRpcValue::iterator i=colors.begin(); i!=colors.end(); ++i) {
        //     ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
        //     ROS_ASSERT(i->second.size() == 3);
        //     for(int j=0; j<i->second.size(); ++j) {
        //         ROS_ASSERT(i->second[j].getType()==XmlRpc::XmlRpcValue::TypeInt);
        //     }
        //     allowed_colors_.push_back(std::make_tuple(i->second[0], i->second[1], i->second[2]));
        // }

        // HIT_PROB  = std::max(0.0, std::min(HIT_PROB,  1.0));
        // MISS_PROB = std::max(0.0, std::min(MISS_PROB, 1.0));
        // map_(std::vector<std::string>({"occupancy"}));
        std::string lidar_topic_name;
        std::string segm_points_topic_name;
        std::string occupancy_grid_topic_name;
        std::string gridmap_topic_name;

        ROS_ASSERT(pnh.getParam("/planner/topics/mapping_server/segm_points", segm_points_topic_name));
        ROS_ASSERT(pnh.getParam("/planner/topics/mapping_server/lidar", lidar_topic_name));
        ROS_ASSERT(pnh.getParam("/planner/topics/mapping_server/costmap", occupancy_grid_topic_name));
        ROS_ASSERT(pnh.getParam("/planner/topics/mapping_server/gridmap", gridmap_topic_name));
        ROS_ASSERT(
            !segm_points_topic_name.empty()
            && !lidar_topic_name.empty()
            && !occupancy_grid_topic_name.empty()
            && !gridmap_topic_name.empty()
            );

        occupied_cells_publisher = nh.advertise<nav_msgs::OccupancyGrid>(occupancy_grid_topic_name, 1);
        grid_map_publisher = nh.advertise<grid_map_msgs::GridMap>(gridmap_topic_name, 1);

        color_cloud_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>(segm_points_topic_name, 10, &MappingServer::update_color_map, this);
        point_cloud_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic_name, 10, &MappingServer::update_occupancy_map, this);

        // pointcloud_subscriber =
        //     new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, lidar_topic_name, 1);

        // tf_pointcloud_subscriber =
        //     new tf::MessageFilter<sensor_msgs::PointCloud2>(*pointcloud_subscriber,
        //                                                     tf_listener,
        //                                                     FIXED_FRAME_ID,
        //                                                     1);

        // tf_pointcloud_subscriber->registerCallback(boost::bind(&MappingServer::update_occupancy_map,
        //                                                        this, _1));

        map_.setGeometry(grid_map::Length(MAXIMUM_RANGE, MAXIMUM_RANGE), RESOLUTION, grid_map::Position(0.0, 0.0));
        map_.add("occupancy", MIN_LOGPROB);
        map_.add("color");
        map_.setFrameId(FIXED_FRAME_ID);
    }

    ~MappingServer()
    {
        delete pointcloud_subscriber;
        delete tf_pointcloud_subscriber;
    }

    /*
     * Update the occupancies of grid map from given pointcloud.
     * A pointcloud subscriber registers this callback function to update the map
     * when a new sensor measurement is observed.
     *
     * @param src_pc: a pointcloud in the sensor coordinate
     */
    void update_occupancy_map(const sensor_msgs::PointCloud2ConstPtr &_src_pc)
    {
        parse_valid_sensor_measurement(*_src_pc);
        publish_occupied_cells();
    }


    void update_color_map(const sensor_msgs::PointCloud2ConstPtr &_src_pc)
    {
        parse_color_sensor_measurement(*_src_pc);
        publish_occupied_cells();
    }

    /*
     * Publish the occupied cells for visualization in RViz.
     * As a simple message type, the function publishes a set of centers of occupied cells.
     *
     */
    void publish_occupied_cells()
    {
        ros::Time now = ros::Time::now();
        map_.setTimestamp(now.toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map_, message);
        grid_map_publisher.publish(message);
        static double last_update_s = 0;
        ROS_INFO_THROTTLE(5.0, "Grid map (freq %f) published.", now.toSec() - last_update_s);
        last_update_s = now.toSec();

        nav_msgs::OccupancyGrid occupancyGrid;
        grid_map::GridMapRosConverter::toOccupancyGrid(map_,
                                                       "occupancy",
                                                       MIN_LOGPROB,
                                                       MAX_LOGPROB,
                                                       occupancyGrid);
        occupancyGrid.header.frame_id = "odom";
        occupied_cells_publisher.publish(occupancyGrid);
    }

protected:
    // Node handle
    ros::NodeHandle &nh;
    ros::NodeHandle &pnh;
    // Transform and pointcloud subscribers
    message_filters::Subscriber<sensor_msgs::PointCloud2> *pointcloud_subscriber;
    tf::MessageFilter<sensor_msgs::PointCloud2> *tf_pointcloud_subscriber;
    tf::TransformListener tf_listener;

    // Options ====================================================================================
protected:
    std::string FIXED_FRAME_ID;
    double RESOLUTION;
    double MAXIMUM_RANGE;
    double HIT_LOGPROB;
    double MISS_LOGPROB;
    double MAX_LOGPROB;
    double MIN_LOGPROB;
    // std::vector<std::tuple<int,int,int>> allowed_colors_;
    ros::Publisher occupied_cells_publisher;
    ros::Publisher grid_map_publisher;
    ros::Subscriber point_cloud_subscriber_;
    ros::Subscriber color_cloud_subscriber_;

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
    void parse_valid_sensor_measurement(const sensor_msgs::PointCloud2 &_ros_pc)
    {
        // Pose of the sensor frame
        tf::StampedTransform sensor_to_world;
        try
        {
            tf_listener.lookupTransform(FIXED_FRAME_ID,
                                        _ros_pc.header.frame_id,
                                        _ros_pc.header.stamp,
                                        sensor_to_world);
        }
        catch (tf::TransformException &e)
        {
            ROS_DEBUG_STREAM("Cannot find a transform from sensor to world: "
                             << _ros_pc.header.frame_id << " --> "
                             << FIXED_FRAME_ID);
            // return false;
        }

        // in the sensor coordinate ===============================================================
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
        pcl::fromROSMsg(_ros_pc, pcl_pointcloud);

        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud_in_sensor_coordinate;
        for (const auto &point : pcl_pointcloud)
        {
            // Remove the invalid data: NaN
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
                continue;

            // Remove the invalid data: out of sensing range
            double l = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (l > MAXIMUM_RANGE)
                continue;

            pcl_pointcloud_in_sensor_coordinate.push_back(point);
        }

        // in the world coordinate ================================================================
        Eigen::Matrix4f transform;
        pcl_ros::transformAsMatrix(sensor_to_world, transform);
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud_in_world_coordinate;
        pcl::transformPointCloud(pcl_pointcloud_in_sensor_coordinate,
                                 pcl_pointcloud_in_world_coordinate,
                                 transform);

        double or_x = sensor_to_world.getOrigin().x();
        double or_y = sensor_to_world.getOrigin().y();

        for (const auto &point : pcl_pointcloud_in_world_coordinate)
        {
            updateMap(point, or_x, or_y);
        }
    }

    void updateMap(const pcl::PointXYZ &point_pose, double origin_x, double origin_y)
    {

        grid_map::Position start(origin_x, origin_y);
        grid_map::Position end(point_pose.x, point_pose.y);

        if (!map_.isInside(end) || !map_.isInside(start))
        {
            ROS_DEBUG_STREAM("End or Start point is not inside map boundary. Start: "
                             << origin_x << ", " << origin_y
                             << ", end point: " << point_pose.x << "," << point_pose.y);
            return;
        }

        for (grid_map::LineIterator iterator(map_, start, end); !iterator.isPastEnd();)
        {
            const grid_map::Index subIndex = *iterator;
            ++iterator;

            // if (!map_.isValid(subIndex)) {
            //     continue;
            // }
            if (std::isnan(map_.at("occupancy", subIndex)))
            {
                map_.at("occupancy", subIndex) = MIN_LOGPROB;
            }

            if (iterator.isPastEnd())
            {
                if (map_.at("occupancy", subIndex) < MAX_LOGPROB)
                {
                    map_.at("occupancy", subIndex) =
                        std::max(MAX_LOGPROB, map_.at("occupancy", subIndex) + HIT_LOGPROB);
                }
            }
            else
            {
                if (map_.at("occupancy", subIndex) > MIN_LOGPROB)
                {
                    map_.at("occupancy", subIndex) =
                        std::min(MIN_LOGPROB, map_.at("occupancy", *iterator) + MISS_LOGPROB);
                }
            }
        }
    }


    void parse_color_sensor_measurement(const sensor_msgs::PointCloud2& _ros_pc) {
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
            updateColorMap(point, or_x, or_y);
        }
    }

    void updateColorMap(const pcl::PointXYZRGB &point_pose, double origin_x, double origin_y) {

        grid_map::Position end(point_pose.x, point_pose.y);

        grid_map::Index subIndex;
        if (!map_.getIndex(end, subIndex)) {
            return;
        }

        union {
            unsigned long longColor;
            float floatColor;
        } colors;

        colors.longColor = ((int)point_pose.r << 16) + ((int)point_pose.g << 8) + (int)point_pose.b;
        map_.at("color", subIndex) = colors.floatColor;

        // TODO: convert label to occupancy
        // float lab = point_pose.r;
        // if (lab < 0 || lab > 100) lab = NAN;
        // map_.at("occupancy", subIndex) = lab;
    }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_server");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MappingServer mapping_server(nh, pnh);

    try
    {
        ros::spin();
    }
    catch (std::runtime_error &e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    return 0;
}
