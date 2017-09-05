/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/GoalManager.hpp"

GoalManager::GoalManager():m_nd("~"){

	goal_index = 0;

	GoalManager::sendGoal();
	
	m_sub_TaskFinished = m_nd.subscribe("/task_finished",  1, &GoalManager::taskFinishedClbk, this);
	
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

void GoalManager::run(){
	ros::spin();
}

void GoalManager::automaticSendGoals(){

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

void GoalManager::sendGoal(){

	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

    // wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal_to_send = goals.at(goal_index);

	ROS_INFO("Sending goal");

	ac.sendGoal(goal_to_send);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("The base moved successfully. Sending next goal");
	}
	else{
		ROS_INFO("The base failed for some reason");
	}
}



void GoalManager::taskFinishedClbk(const human_aware_navigation::TaskFinished &tf){

	std::string uri = "Finished Task " + std::to_string(tf.task_id) + ". Moving to the next goal!";

	printer::printGreen(uri);

	goal_index++;

	GoalManager::sendGoal();
}