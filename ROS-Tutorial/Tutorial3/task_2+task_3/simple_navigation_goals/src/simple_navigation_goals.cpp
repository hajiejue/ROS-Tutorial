#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default

  //wait for the action server to come up
       MoveBaseClient ac("move_base", true); 	
               while(!ac.waitForServer(ros::Duration(5.0)))
             	 {
                	ROS_INFO("Waiting for the move_base action server to come up");
              	}
		
      
       while(1)
	{       
	      
		move_base_msgs::MoveBaseGoal target_goal;
  		//we'll send a goal to the robot to move 1 meter forward
  		target_goal.target_pose.header.frame_id = "map";
  		target_goal.target_pose.header.stamp = ros::Time::now();

	  
  	        target_goal.target_pose.pose.position.x =  -2.94;
	  	target_goal.target_pose.pose.position.y = -2.38;
	  	target_goal.target_pose.pose.position.z = 0;
	  	target_goal.target_pose.pose.orientation.x = 0;
	  	target_goal.target_pose.pose.orientation.y = 0; 
	  	target_goal.target_pose.pose.orientation.z = 0.99;
	  	target_goal.target_pose.pose.orientation.w = 0.025;

	 	ROS_INFO("Sending first goal");
	  	ac.sendGoal(target_goal);

	  	ac.waitForResult();

	  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	   		 ROS_INFO("reach the first point");
	 	else
	   		 ROS_INFO("The base failed to reach the first point");



    	        target_goal.target_pose.pose.position.x =  -3.20;
	  	target_goal.target_pose.pose.position.y = -6.32;
	  	target_goal.target_pose.pose.position.z = 0;
	  	target_goal.target_pose.pose.orientation.x = 0;
	  	target_goal.target_pose.pose.orientation.y = 0; 
	  	target_goal.target_pose.pose.orientation.z = 0.998;
	  	target_goal.target_pose.pose.orientation.w = 0.059;
	 	ROS_INFO("Sending second goal");
	  	ac.sendGoal(target_goal);

	  	ac.waitForResult();

	  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	   		 ROS_INFO("reach the second point");
	 	else
	   		 ROS_INFO("The base failed to reach the second point");



  	        target_goal.target_pose.pose.position.x =  -1.44;
	  	target_goal.target_pose.pose.position.y = -12.006;
	  	target_goal.target_pose.pose.position.z = 0;
	  	target_goal.target_pose.pose.orientation.x = 0;
	  	target_goal.target_pose.pose.orientation.y = 0; 
	  	target_goal.target_pose.pose.orientation.z = 0.068;
	  	target_goal.target_pose.pose.orientation.w = 0.9976;

	 	ROS_INFO("Sending third goal");
	  	ac.sendGoal(target_goal);

	  	ac.waitForResult();

	  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	   		 ROS_INFO("reach the third point");
	 	else
	   		 ROS_INFO("The base failed to reach the third point");




    

  		
	}
return 0;
}

