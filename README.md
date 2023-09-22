# unity_ros_transform

### Getting started
* If running data from a bag file and the tf topics are not the default ones, remap the topic names
  ```sh
  rosbag play -l <rosbag.bag> /merged_desktop/tf:=/tf /merged_desktop/tf_static:=/tf_static
  ```
  If tf_tree from bag file has time synchronizing issue, try using simulated time
  ```sh
  rosparam set use_sim_time true
  ```
  Then play bag file with --clock
  ```sh
  rosbag play --clock -l -s 1 -u 450 2023-06-14-16-58-28_for\ TCP\ test\ server\ for\ TCP\ test.bag /merged_desktop/tf:=/tf /merged_desktop/tf_static:=/tf_static
  ```
* Inspect tf tree
  ```sh
  rosrun rqt_tf_tree rqt_tf_tree
  ```
* ros terminal
  ```sh
  roslaunch unity_ros_tf transformers.launch
  ```

* git
  Go to the package directory
  ```sh
  git status
  git add .
  git commit -m "Comment"
  git push -f origin main
  ```
  


### odometry_fromodom_transformer.py code flow
Output = /global_odom = /original_odom + transform_child_to_parent_frame + z_offset_human + z_offset_floor
1. transform_child_to_parent_frame = Lookup transform from tf tree between the parent_frame and child_frame
    * parent_frame should be the upstream of the tf_tree, i.e. =  map
    * child_frame should be the frame_id of the /original_odom. Look for it by ```rostopic echo /original_odom```
2. z_offset_human = set as param in transformer.launch
3. z_offset_floor = average of robot's odometry's z position (z position of transform_child_to_parent_frame, it is wrt global frame)

### odometry_transformer.py code flow
Output = /global_odom = transform_child_to_parent_frame + z_offset_human + z_offset_floor
1. transform_child_to_parent_frame = Lookup transform from tf tree between the parent_frame and child_frame
2. z_offset_human = set as param in transformer.launch
<del> 3. new_floor_reached = check the history of /is_climbing in bool_history[]. If bool_history = [ [True]*consecutive_num + [False]*consecutive_num ], means the robot has finished climbing. z_offset_floor is extracted.  </del>
4. z_offset_floor = average of robot's odometry's z position (z position of transform_child_to_parent_frame, it is wrt global frame)

#### old odometry_transformer.py check for new_floor_reached event by listening to /is_climbing bool. 
However, it hasn't been test yet. For now, we are going to use upper floor as Unity height = 0, and the lower floor will have -ve height that is hard-coded. 

