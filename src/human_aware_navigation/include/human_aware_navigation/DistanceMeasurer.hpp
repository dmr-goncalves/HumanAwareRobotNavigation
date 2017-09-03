/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <human_aware_navigation/GetPathDistanceDeviation.h>

class DistanceMeasurer
{
public:
	DistanceMeasurer ();
	virtual ~DistanceMeasurer (){};

	void run();

private:

	ros::NodeHandle                               m_nd;
	ros::ServiceServer 							  distanceService;
	ros::Subscriber 							  m_sub_GlobalPlan;
	ros::Subscriber 							  m_sub_RobotPose;

	float 									      total_distance;
	float 									      global_plan_distance;
	float 									      robot_path_distance;

	bool 										  firstGlobalPath;
	
	
	nav_msgs::Path    							  global_plan;
	nav_msgs::Path 								  robot_path;

	void globalPlanClbk(const nav_msgs::Path gp);
	void robotPoseClbk(const nav_msgs::Odometry odom);
	bool getPathDistance(human_aware_navigation::GetPathDistanceDeviation::Request& req, human_aware_navigation::GetPathDistanceDeviation::Response& res);

};/* _distance_measurer_HPP__ */