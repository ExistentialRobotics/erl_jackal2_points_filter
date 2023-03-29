#ifndef _POINT_CLOUD_TRANSFORMER_H
#define _POINT_CLOUD_TRANSFORMER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>

class Jackal2_Cloud_Filter
{
    private:
        ros::NodeHandle nh_;
        tf::TransformListener tl_;

        ros::Subscriber point_cloud_subscriber_;
        ros::Publisher  robo_filtered_cloud_publisher_;
        ros::Publisher  world_filtered_cloud_publisher_;

        std::string world_frame_id;
        std::string lidar_frame_id;

        double x_min;
        double x_max;
        double y_min;
        double y_max;

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud);

    public:
        Jackal2_Cloud_Filter();
        ~Jackal2_Cloud_Filter(){}
};

Jackal2_Cloud_Filter::Jackal2_Cloud_Filter() : nh_("~")
{
    std::string input_cloud_name;
    std::string output_cloud_robo_name;
    std::string output_cloud_world_name;
    

    nh_.param("input_cloud_topic_name", input_cloud_name, std::string("/ouster/points"));
    nh_.param("output_cloud_topic_name_in_roboframe" , output_cloud_robo_name,  std::string("/jackal2_cloudfilter/points_robo"));
    nh_.param("output_cloud_topic_name_in_worldframe", output_cloud_world_name, std::string("/jackal2_cloudfilter/points_world"));
    nh_.param("world_frame_id", world_frame_id, std::string("/map"));
    nh_.param("lidar_frame_id", lidar_frame_id, std::string("/os_sensor"));
    nh_.param("x_filter_min", x_min, -0.5);
    nh_.param("x_filter_max", x_max,  0.0);
    nh_.param("y_filter_min", y_min, -0.3);
    nh_.param("y_filter_min", y_max,  0.3);

    point_cloud_subscriber_          = nh_.subscribe(input_cloud_name, 10, &Jackal2_Cloud_Filter::pointCloudCallback, this);
    robo_filtered_cloud_publisher_   = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_robo_name,  10);
    world_filtered_cloud_publisher_  = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_world_name, 10);
}

void Jackal2_Cloud_Filter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Create PassThrough filter object and set parameters for x-dimension
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud.makeShared());
    pass_x.setFilterFieldName("x"); // filter based on x-axis
    pass_x.setFilterLimits(x_max,x_min); // set range limits (-0.5m to 0m)
    pass_x.filter(cloud); // apply filter

    // Create PassThrough filter object and set parameters for y-dimension
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud.makeShared());
    pass_y.setFilterFieldName("y"); // filter based on y-axis
    pass_y.setFilterLimits(y_min, y_max); // set range limits (-0.3m to 0.3m)
    pass_y.filter(cloud); // apply filter

    // Convert pcl::PointCloud to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(cloud, filtered_cloud_msg);

    // Publish filtered point cloud
    robo_filtered_cloud_publisher_.publish(filtered_cloud_msg);
    
    tf::StampedTransform transform;
    try{
        listener.lookupTransform(world_frame_id, lidar_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return;
    }

    sensor_msgs::PointCloud2 transformed_point_cloud;
    pcl_ros::transformPointCloud(filtered_cloud_msg, transformed_point_cloud, transform);
    world_filtered_cloud_publisher_.publish(transformed_point_cloud);
}
ros::Publisher pub;

#endif