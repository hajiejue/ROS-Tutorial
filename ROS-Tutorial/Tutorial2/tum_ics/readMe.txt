%%%%%%%%%%%%%%%%%%%%%%%%% Exercise 1 %%%%%%%%%%%%%%%
Robot:tiago
Packages:ics_gazebo
Complation:
$catkin_make
$source devel/setup.bash 
roslaunch ics_gazebo tiago.launch world_suffix:=tutorial2




%%%%%%%%%%%%%%%%%%%%%%%%% Exercise 2 %%%%%%%%%%%%%%%
Robot:tiago
Packages:ics_gazebo
Complation:
$catkin_make
$source devel/setup.bash 
roslaunch ics_gazebo tiago.launch world_suffix:=empty
(just see the rviz doumentation)

%%%%%%%%%%%%%%%%%%%%%%%%% Exercise 3 %%%%%%%%%%%%%%%
Robot:tiago
Packages:ics_gazebo,turtle_vis
Complation:
$catkin_make
$source devel/setup.bash 
$roslaunch ics_gazebo tiago.launch world_suffix:=empty or tutorial2
in another terminal
$source devel/setup.bash 
$rosrun turtle_vis turtle_set_position
1,1,0 and input "enter"




%%%%%%%%%%%%%%%%%%%%%%%%% Exercise 4 %%%%%%%%%%%%%%%
Robot:tiago
Packages:controllers_tutorials
Complation:
$catkin_make
$source devel/setup.bash 
$roslaunch ics_gazebo tiago.launch world_suffix:=empty
in another terminal
$source devel/setup.bash 
$rosrun controller_manager controller_manager kill head_controller
$roslaunch controllers_tutorials new_head_controller_tiago.launch







