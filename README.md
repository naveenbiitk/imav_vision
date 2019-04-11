# imav_vision
Object detection and pose estimation for imav-2019 outdoor competition.


How to use 
1. Clone the repository on your workspace 
2. change the subscribing topics in aruco_ros/single.launch
3. catkin build

if erroroccurs modify aruco_ros/object_pos.cpp

node will give position (geometry_msgs::PoseStamped) in ```/detected_object/pose``` - world frame pose of detected rectangle.

 
To use it in simulator paste the following lines in three_firefly.launch in imav_sim. inside any of the quad
```
<node name="input" type="input" pkg="aruco_ros">
        <remap from="colour_image" to="nadir_cam/image_raw" />
    </node>  

    <node pkg="aruco_ros" type="single" name="aruco_single">
        <remap from="aruco_single/camera_info" to="nadir_cam/camera_info" />
        <remap from="aruco_single/threshold_image" to="threshold_image" />
        <param name="image_is_rectified" value="True"/>
        <param name="marker_size"        value="0.1"/>
        <param name="marker_id"          value="582"/>
        <param name="reference_frame"    value=" "/>   
        <param name="camera_frame"       value="stereo_gazebo"/>
        <param name="marker_frame"       value="aruco_marker_frame" />
        <param name="corner_refinement"  value="LINES" />
    </node>

    <node name="object_pos" type="object_pos" pkg="aruco_ros">
        <remap from="odometry" to="ground_truth/pose" />
        <remap from="imu/data" to="imu" />
        <remap from="aruco_single/pose" to="aruco_single/pose" />
        <remap from="object/pose" to="object/pose" />
    </node>
```

Now marker node is detected publishing pose in object/pose 
We will run Median filter for now which will smooth this data and publishes this to command pose of the quad.
Go inside ```aruco_ros/src/ ```  and then excute filter.py
