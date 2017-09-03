/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/GoalManager.hpp"

GoalManager::GoalManager():m_nd("~"){
	
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 5.0;
	goal.target_pose.pose.orientation.w = 1.0;
	
	goals.push_back(goal);

	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 6.0;
	goal.target_pose.pose.orientation.w = 1.0;
	
	goals.push_back(goal);

	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 7.0;
	goal.target_pose.pose.orientation.w = 1.0;
	
	goals.push_back(goal);
}

void GoalManager::sendGoal(){

	for(int i = 0; i < goals.size(); i++){

	   // Tell the action client that we want to spin a thread by default
		MoveBaseClient ac("move_base", true);

       //wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}

		move_base_msgs::MoveBaseGoal goal_to_send = goals.at(i);

		ROS_INFO("Sending goal");

		ac.sendGoal(goal_to_send);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("The base moved successfully. Sending next goal");
			i++;
		}
		else{
			ROS_INFO("The base failed for some reason");
			i--;
		}
	}
	
}