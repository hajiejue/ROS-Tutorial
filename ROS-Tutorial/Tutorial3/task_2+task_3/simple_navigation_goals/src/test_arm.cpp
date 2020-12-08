#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
int movearm_signal = 0;
 while(1)
	{       
		
  		move_base_msgs::MoveBaseGoal target_goal;

  		//we'll send a goal to the robot to move 1 meter forward
  		target_goal.target_pose.header.frame_id = "map";
  		target_goal.target_pose.header.stamp = ros::Time::now();

	  
  	        target_goal.target_pose.pose.position.x =  -3.05;
	  	target_goal.target_pose.pose.position.y = -2.40;
	  	target_goal.target_pose.pose.position.z = 0;
	  	target_goal.target_pose.pose.orientation.x = 0;
	  	target_goal.target_pose.pose.orientation.y = 0; 
	  	target_goal.target_pose.pose.orientation.z = 0.99;
	  	target_goal.target_pose.pose.orientation.w = 0.025;

 		ROS_INFO("Sending goal");
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
	  	target_goal.target_pose.pose.position.y = -11.80;
	  	target_goal.target_pose.pose.position.z = 0;
	  	target_goal.target_pose.pose.orientation.x = 0;
	  	target_goal.target_pose.pose.orientation.y = 0; 
	  	target_goal.target_pose.pose.orientation.z = -0.725;
	  	target_goal.target_pose.pose.orientation.w = -0.688;

	 	ROS_INFO("Sending third goal");
	  	ac.sendGoal(target_goal);

	  	ac.waitForResult();

	  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	   		{
				ROS_INFO("reach the third point");
				movearm_signal =1;
			}
	 	else
	   		 ROS_INFO("The base failed to reach the third point");
		if (movearm_signal==1){ 
	movearm_signal = 0;
// first joint position
  std::map<std::string, double> target_position;

  target_position["torso_lift_joint"] = 0;
  target_position["arm_1_joint"] = 0.2;
  target_position["arm_2_joint"] = -1.34;
  target_position["arm_3_joint"] = -0.2;
  target_position["arm_4_joint"] = 1.94;
  target_position["arm_5_joint"] = -1.57;
  target_position["arm_6_joint"] = 0.03;
  target_position["arm_7_joint"] = 0.00;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_first_plan;
  group_arm_torso.setPlanningTime(5.0);
  bool success_first = bool(group_arm_torso.plan(my_first_plan));

  if ( !success_first )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_first_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start_first = ros::Time::now();
  

  moveit::planning_interface::MoveItErrorCode e_first = group_arm_torso.move();
  if (!bool(e_first))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_first).toSec());

  spinner.stop();
//second joint position
  target_position["torso_lift_joint"] = 0;
  target_position["arm_1_joint"] = 0.2;
  target_position["arm_2_joint"] = -1.34;
  target_position["arm_3_joint"] = -1.97;
  target_position["arm_4_joint"] = 1.94;
  target_position["arm_5_joint"] = -1.57;
  target_position["arm_6_joint"] = 0.03;
  target_position["arm_7_joint"] = 0.00;

  //ros::NodeHandle nh;
  //ros::AsyncSpinner spinner(1);
  spinner.start();

  //std::vector<std::string> torso_arm_joint_names;
  //select group of joints
 // moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_second_plan;
  group_arm_torso.setPlanningTime(5.0);
  bool success_second = bool(group_arm_torso.plan(my_second_plan));

  if ( !success_second )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_second_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start_second = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e_second = group_arm_torso.move();
  if (!bool(e_second))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_second).toSec());

  spinner.stop();

//third joint position

  target_position["torso_lift_joint"] = 0;
  target_position["arm_1_joint"] = 0.2;
  target_position["arm_2_joint"] = -0.44;
  target_position["arm_3_joint"] = -1.97;
  target_position["arm_4_joint"] = 1.94;
  target_position["arm_5_joint"] = -1.57;
  target_position["arm_6_joint"] = 0.03;
  target_position["arm_7_joint"] = 0.00;

  //ros::NodeHandle nh;
  //ros::AsyncSpinner spinner(1);
  spinner.start();

 // std::vector<std::string> torso_arm_joint_names;
  //select group of joints
 // moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

 // torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_third_plan;
  group_arm_torso.setPlanningTime(5.0);
  bool success_third = bool(group_arm_torso.plan(my_third_plan));

  if ( !success_third )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_third_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start_third = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e_third = group_arm_torso.move();
  if (!bool(e_third))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_third).toSec());

//fourth joint position
  spinner.stop();
  target_position["torso_lift_joint"] = 0;
  target_position["arm_1_joint"] = 0.2;
  target_position["arm_2_joint"] = -1.34;
  target_position["arm_3_joint"] = -1.97;
  target_position["arm_4_joint"] = 1.94;
  target_position["arm_5_joint"] = -1.57;
  target_position["arm_6_joint"] = 0.03;
  target_position["arm_7_joint"] = 0.00;

  //ros::NodeHandle nh;
 // ros::AsyncSpinner spinner(1);
  spinner.start();

 // std::vector<std::string> torso_arm_joint_names;
  //select group of joints
 // moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_fourth_plan;
  group_arm_torso.setPlanningTime(5.0);
  bool success_fourth = bool(group_arm_torso.plan(my_fourth_plan));

  if ( !success_fourth )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_fourth_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start_fourth = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e_fourth = group_arm_torso.move();
  if (!bool(e_fourth))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_fourth).toSec());

  spinner.stop();
//fifth joint position
  spinner.stop();
  target_position["torso_lift_joint"] = 0;
  target_position["arm_1_joint"] = 0.2;
  target_position["arm_2_joint"] = -1.34;
  target_position["arm_3_joint"] = -0.2;
  target_position["arm_4_joint"] = 1.94;
  target_position["arm_5_joint"] = -1.57;
  target_position["arm_6_joint"] = 0.03;
  target_position["arm_7_joint"] = 0.00;

  //ros::NodeHandle nh;
 // ros::AsyncSpinner spinner(1);
  spinner.start();

 // std::vector<std::string> torso_arm_joint_names;
  //select group of joints
 // moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_fifth_plan;
  group_arm_torso.setPlanningTime(5.0);
  bool success_fifth = bool(group_arm_torso.plan(my_fifth_plan));

  if ( !success_fifth )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_fifth_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start_fifth = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e_fifth = group_arm_torso.move();
  if (!bool(e_fifth))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_fifth).toSec());

  spinner.stop();
//sixth joint position
  spinner.stop();
  target_position["torso_lift_joint"] = 0;
  target_position["arm_1_joint"] = 0.2;
  target_position["arm_2_joint"] = -1.34;
  target_position["arm_3_joint"] = -0.2;
  target_position["arm_4_joint"] = 1.94;
  target_position["arm_5_joint"] = -1.57;
  target_position["arm_6_joint"] = 1.37;
  target_position["arm_7_joint"] = 0.00;

  //ros::NodeHandle nh;
 // ros::AsyncSpinner spinner(1);
  spinner.start();

 // std::vector<std::string> torso_arm_joint_names;
  //select group of joints
 // moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_sixth_plan;
  group_arm_torso.setPlanningTime(5.0);
  bool success_sixth = bool(group_arm_torso.plan(my_sixth_plan));

  if ( !success_sixth )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_fourth_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start_sixth = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e_sixth = group_arm_torso.move();
  if (!bool(e_sixth))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_sixth).toSec());

  spinner.stop();


 // return EXIT_SUCCESS;
}
}
  return 0;
}

