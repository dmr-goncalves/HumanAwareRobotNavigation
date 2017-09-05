
#include <ros/ros.h>
#include <ros/package.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <human_aware_navigation/TaskFinished.h>

#include <utilities/printer.hpp>

#include <sstream>
#include <string>     // std::string, std::to_string


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalManager
{
public:
	GoalManager ();
	virtual ~GoalManager (){};

	void run();

private:

	ros::NodeHandle                               m_nd;

	ros::Subscriber								  m_sub_TaskFinished;

	std::vector<move_base_msgs::MoveBaseGoal>     goals;

	int											  goal_index;

	void taskFinishedClbk(const human_aware_navigation::TaskFinished &tf);

	void automaticSendGoals();
	
	void sendGoal();
	
};/* _goal_manager_HPP__ */