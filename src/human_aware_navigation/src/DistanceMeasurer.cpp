/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/DistanceMeasurer.hpp"

DistanceMeasurer::DistanceMeasurer(){
	
	distanceService = m_nd.advertiseService("/get_distance", &DistanceMeasurer::getPathDistance, this);
	m_sub_GlobalPlan = m_nd.subscribe("/global_plan", 1, &DistanceMeasurer::globalPlanClbk, this);
	m_sub_RobotPose = m_nd.subscribe("/odometry", 1, &DistanceMeasurer::robotPoseClbk, this);

	total_distance = 0.0;
	global_plan_distance = 0.0;
	robot_path_distance = 0.0;

	firstGlobalPath = true;

	robot_path.header.seq = 0;
	robot_path.header.frame_id = "odom";

}

void DistanceMeasurer::run(){

	spin();

}

bool DistanceMeasurer::getPathDistance(human_aware_navigation::GetPathDistanceDeviation::Request& req, human_aware_navigation::GetPathDistanceDeviation::Response& res)
{

	robot_path_distance = 0.0;

	for(int x = 1; x < robot_path.poses.size(); x++){

		robot_path_distance += sqrt(pow(robot_path.poses.at(x).pose.position.x - robot_path.poses.at(x - 1).pose.position.x ,2) + pow( robot_path.poses.at(x).pose.position.y - robot_path.poses.at(x -1).pose.position.y ,2));

	}		

	// ROS_INFO("Robot Path Distance -> %f", robot_path_distance);
	// ROS_INFO("Global Plan Distance -> %f", global_plan_distance);

	total_distance = robot_path_distance - global_plan_distance;

	res.distance = total_distance;

	// ROS_INFO("Sending back response: [%f]", res.distance);

	return true;
}


void DistanceMeasurer::globalPlanClbk(const nav_msgs::Path gp){
	if(firstGlobalPath){
		global_plan = gp;

		for(int x = 1; x < global_plan.poses.size(); x++){

			global_plan_distance += sqrt(pow(global_plan.poses.at(x).pose.position.x - global_plan.poses.at(x - 1).pose.position.x ,2) + pow( global_plan.poses.at(x).pose.position.y - global_plan.poses.at(x -1).pose.position.y ,2));

		}

		firstGlobalPath = false;
	}
}

void DistanceMeasurer::robotPoseClbk(const nav_msgs::Odometry odom){

	robot_path.header.seq = robot_path.header.seq++;
	robot_path.header.stamp = Time::now();

	geometry_msgs::PoseStamped ps;

	ps.header.seq = 0;
	ps.header.stamp = Time::now();
	ps.header.frame_id = "odom";

	ps.pose = odom.pose.pose;

	robot_path.poses.push_back(ps);
}