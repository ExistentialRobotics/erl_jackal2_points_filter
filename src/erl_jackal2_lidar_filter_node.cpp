#include <ros/ros.h>
#include "erl_jackal2_lidar_filter/erl_jackal2_lidar_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "erl_jacksl2_lidar_filter_node", ros::init_options::AnonymousName);

  Jackal2_Cloud_Filter jackal2_cloud_filter;

  ros::spin();

  return 0;
}