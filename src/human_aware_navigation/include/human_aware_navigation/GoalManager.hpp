
#include <ros/ros.h>
#include <ros/package.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalManager
{
public:
	GoalManager ();
	virtual ~GoalManager (){};

	void sendGoal();

private:

	ros::NodeHandle                               m_nd;

	std::vector<move_base_msgs::MoveBaseGoal>     goals;
	
};/* _goal_manager_HPP__ */