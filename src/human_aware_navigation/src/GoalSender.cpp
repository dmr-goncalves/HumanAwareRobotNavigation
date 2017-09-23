/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/GoalSender.hpp"

GoalSender::GoalSender():m_nd("~"){

	goal_index = 0;

	m_sub_TaskFinished = m_nd.subscribe("/task_finished",  1, &GoalSender::taskFinishedClbk, this);

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";

	goal.target_pose.header.stamp = Time::now();
	goal.target_pose.pose.position.x = 5.0;
	goal.target_pose.pose.position.y = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = -0.626;
	goal.target_pose.pose.orientation.w = 0.780;
	goals.push_back(goal);

	goal.target_pose.header.stamp = Time::now();
	goal.target_pose.pose.position.x = 0.7;
	goal.target_pose.pose.position.y = -2.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 1.0;
	goal.target_pose.pose.orientation.w = 0.0;
	goals.push_back(goal);

	goal.target_pose.header.stamp = Time::now();
	goal.target_pose.pose.position.x = 2.0;
	goal.target_pose.pose.position.y = 1.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;
	goals.push_back(goal);

	GoalSender::sendGoal();
}

void GoalSender::run(){
	spin();
}

void GoalSender::automaticSendGoals(){

	for(int i = 0; i < goals.size(); i++){

	   // Tell the action client that we want to spin a thread by default
		MoveBaseClient ac("move_base", true);

       //wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
			printer::printYellow("Waiting for the move_base action server to come up");
		}

		move_base_msgs::MoveBaseGoal goal_to_send = goals.at(i);

		printer::printBlue("Sending goal");

		ac.sendGoal(goal_to_send);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			if(i < goals.size() - 1){
				printer::printGreen("The base moved successfully. Sending next goal");
			}else{
				printer::printGreen("Last goal reached successfully!");
			}
		}
		else{
			printer::printRed("The base failed for some reason");
			if(i == 0){
				i = -1;
			}else{
				i--;
			}
		}
	}
}

void GoalSender::sendGoal(){

	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

    // wait for the action server to come up
	while(!ac.waitForServer(Duration(5.0))){
		printer::printYellow("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal_to_send = goals.at(goal_index);

	printer::printBlue("Sending goal");

	ac.sendGoal(goal_to_send);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		if(goal_index < goals.size() - 1){
			printer::printGreen("The base moved successfully.");
		}else{
			printer::printGreen("Last goal reached successfully!");
		}
	}
	else{
		printer::printRed("The base failed for some reason");
	}
}



void GoalSender::taskFinishedClbk(const human_aware_navigation::TaskFinished &tf){

	if(goal_index == goals.size() - 1){
		printer::printCyan("No more goals!");
	}else{

		std::string uri = "Finished Task " + std::to_string(tf.task_id) + ". Moving to the next goal!";

		printer::printGreen(uri);

		goal_index++;

		GoalSender::sendGoal();
	}
}
