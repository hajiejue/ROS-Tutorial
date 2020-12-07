open the terminal and input:
source /opt/ros/kinetic/setup.bash
cd ~ROS/workspace/roboCupHome_tutorial_Wudamu or cd "in your file"
source devel/setup.bash
catkin_make
roslaunch turtle_vis TurtleVis.launch
open a new terminal
cd "in your file"
rosservice call /TurtlePose 'input tab':
x:2.0
y:2.0
theta:1.57"
or rosservice call /TurtlePose [2,2,0]
open a new terminal
cd “in your file ”
rosrun turtle_vis turtle_set_position_node
3,3,0




Exercise 3:
1)Call the service and set the position 
Commandline:rosservice call /TurtlePose [2,2,0]
Turtle_set_position(Node2) send the Topic(turtlePose) with message(turtle_vis::send_disered_pose) to Node1 


2)Publischer/Subscriber using asynchronous communication and it is just one direction(Publischer to Subscriber)
  Service/Client using synchronous communication and is it two-way communication(Client to Service and Service to Client)
if we replace the Service for a Subscriber,we will not know ,whether the new position is set up by Node1.








