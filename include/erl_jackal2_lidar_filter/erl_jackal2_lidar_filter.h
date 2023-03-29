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
    nh_.param("world_frame_id", world_frame_id, std::string("map"));
    nh_.param("lidar_frame_id", lidar_frame_id, std::string("os_sensor"));
    nh_.param("x_filter_min", x_min, -0.5);
    nh_.param("x_filter_max", x_max,  0.0);
    nh_.param("y_filter_min", y_min, -0.3);
    nh_.param("y_filter_max", y_max,  0.3);

    point_cloud_subscriber_          = nh_.subscribe(input_cloud_name, 10, &Jackal2_Cloud_Filter::pointCloudCallback, this);
    robo_filtered_cloud_publisher_   = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_robo_name,  10);
    world_filtered_cloud_publisher_  = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_world_name, 10);
}

void Jackal2_Cloud_Filter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    // Convert the input point cloud to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // Create a new PCL point cloud for the filtered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    filtered_cloud->header = pcl_cloud->header;
    filtered_cloud->points.reserve(pcl_cloud->points.size());

    // Filter the points one by one
    for (size_t i = 0; i < pcl_cloud->points.size(); i++)
    {
      pcl::PointXYZI& point = pcl_cloud->points[i];

      // Check if the point meets the filtering condition
      if (!((point.x > x_min && point.x < x_max) && (point.y > y_min && point.y < y_max)))
      {
        // Add the point to the filtered point cloud
        filtered_cloud->points.push_back(point);
      }
    }

    // convert filtered PCL point cloud to PointCloud2 and publish
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;
    robo_filtered_cloud_publisher_.publish(filtered_cloud_msg);
    
    // tf::TransformListener listener;
    // tf::StampedTransform transform;
    // try{
    //     listener.lookupTransform(lidar_frame_id, world_frame_id, ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    //     return;
    // }
    // sensor_msgs::PointCloud2 transformed_point_cloud;
    // pcl_ros::transformPointCloud(world_frame_id, filtered_cloud_msg, transformed_point_cloud, listener);
    // world_filtered_cloud_publisher_.publish(transformed_point_cloud);
}

#endif