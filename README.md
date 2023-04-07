# ERL Jackal2 points filter
ERL Jackal2 3D lidar filter. Filter the raw OS1-32 lidar data of robo-body/antennas. 

# How to use
1. This package was already installed in ERL Jackal 2 robot under the work space <i>"catkin_ws"</i>.
2. ```cd catkin_ws```
3. ```source devel/setup.bash```
4. ```roslaunch erl_jackal2_filter points_filter.launch``` OR integrate the launch content with your own launch file.  

# How to modify the launch file
``` 
<launch>

  <node pkg="erl_jackal2_filter" type="erl_jackal2_filter" name="jackal2_filter_node" output="screen">
    <param name="input_cloud_topic_name"                   value="/ouster/points" />
    <param name="output_cloud_topic_name_in_roboframe"     value="/jackal2_filtered/points" />
    <param name="output_cloud_topic_name_in_worldframe"    value="/jackal2_filtered/points_in_map" />
    <param name="world_frame_id"                           value="map" />
    <param name="lidar_frame_id"                           value="os_sensor" />
    <param name="x_filter_min"                             value=" -0.8" />
    <param name="x_filter_max"                             value="  0.0" />
    <param name="y_filter_min"                             value=" -0.3" />
    <param name="y_filter_max"                             value="  0.3" />
    <param name="z_filter_min"                             value=" -0.05" />
    <param name="z_filter_max"                             value="  0.05" />
  </node>

</launch>
```
1. <b>x_filter_min/max:</b> the minimum/maximum range of x axis you want to filter out.
   
   <b>y_filter_min/max:</b> the minimum/maximum range of y axis you want to filter out.
   
   These two parameters will work as a filter box with:
   1. <b>length:</b> x_filter_max - x_filter_min
   2. <b>width :</b> y_filter_max - y_filter_min
   3. <b>height:</b> maximum lidar z range

2. <b>z_filter_min/max:</b> an extra ground filter, set as 0.00 to keep the lidar ground points. 

# Performance
1. The pointcloud before the filter:
<img src="https://user-images.githubusercontent.com/89951560/230669292-98c3b99c-c707-457b-92ff-48bf500e8f1e.png" width="800" />

2. The pointcloud after the filter:
<img src="https://user-images.githubusercontent.com/89951560/230669366-ea67d076-5ba7-4bbc-81ca-ad34e1547d42.png" width="800" />





