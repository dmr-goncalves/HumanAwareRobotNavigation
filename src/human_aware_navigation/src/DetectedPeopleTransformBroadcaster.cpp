/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/DetectedPeopleTransformBroadcaster.hpp"

DetectedPeopleTransformBroadcaster::DetectedPeopleTransformBroadcaster():m_nd("~"){

	/* Configuration of the Subsribers and Publishers topics and callbacks */
	m_sub_DetectedPersons = m_nd.subscribe("/detected_people", 1, &DetectedPeopleTransformBroadcaster::detectedPeopleClbk, this);
}


void DetectedPeopleTransformBroadcaster::run(){
	spin();
}


void DetectedPeopleTransformBroadcaster::detectedPeopleClbk(const spencer_tracking_msgs::DetectedPersons DP){

	Transform transform;

	string frame_id = "detected_person_1";

	for(int x = 0; x < DP.detections.size(); x++){

		static TransformBroadcaster tf_broadcaster;

		transform.setOrigin(Vector3(DP.detections.at(x).pose.pose.position.z, -DP.detections.at(x).pose.pose.position.x, DP.detections.at(x).pose.pose.position.y));

		Quaternion q;

		quaternionMsgToTF(DP.detections.at(x).pose.pose.orientation, q);

		transform.setRotation(q);

		tf_broadcaster.sendTransform(StampedTransform(transform, Time::now(), "rgbd_front_top_link", frame_id));
	}
}
