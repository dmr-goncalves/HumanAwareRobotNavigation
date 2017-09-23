/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <spencer_tracking_msgs/DetectedPersons.h>

#include <utilities/printer.hpp>

using namespace tf;
using namespace ros;
using namespace std;

class DetectedPeopleTransformBroadcaster
{
public:

	DetectedPeopleTransformBroadcaster ();
	virtual ~DetectedPeopleTransformBroadcaster (){};

	void run();

private:

	ros::NodeHandle                             m_nd;
	ros::Subscriber                             m_sub_DetectedPersons;

	void detectedPeopleClbk(const spencer_tracking_msgs::DetectedPersons DP);

};
