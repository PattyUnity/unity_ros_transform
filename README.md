# unity_ros_transform

### Getting started
* If running data from a bag file and the tf topics are not the default ones, remap the topic names
  ```sh
  rosbag play -l <rosbag.bag> /merged_desktop/tf:=/tf /merged_desktop/tf_static:=/tf_static
  ```
* Inspect tf tree
  ```sh
  rosrun rqt_tf_tree rqt_tf_tree
  ```
* ros terminal
  ```sh
  roslaunch unity_ros_tf transformers.launch
  ```



### odometry_transformer.py code flow
Output = /global_odom = transform_child_to_parent_frame + z_offset_human + z_offset_floor
1. transform_child_to_parent_frame = Lookup transform from tf tree between the parent_frame and child_frame
2. z_offset_human = set as param in transformer.launch
3. new_floor_reached = check the history of /is_climbing in bool_history[]. If bool_history = [[True]*consecutive_num + [False]*consecutive_num], means the robot has finished climbing. z_offset_floor is extracted.  
4. z_offset_floor = average of robot's odometry's z position (z position of transform_child_to_parent_frame, it is wrt global frame)