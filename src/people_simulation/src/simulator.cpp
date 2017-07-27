/***************************** Made by Duarte Gonçalves *********************************/

#include "simulator_app/simulator.hpp"

simulator::simulator():m_nd("~"){
  //Configurationd of the Subsribers and Publishers topics and callbacks
  m_pub_People = m_nd.advertise<spencer_tracking_msgs::DetectedPersons>("/simulator/spencer/perception/detected_persons", 1);
  x1 = 2;
  y1 = -3.3;
  x2 = 4.4;
  y2 = -3.3;
  x3 = 6.4;
  y3 = -3.3;
}

void simulator::run(){

  ros::Rate loop_rate(0.60);

  while(ros::ok()){

    spencer_tracking_msgs::DetectedPersons DPS;
    spencer_tracking_msgs::DetectedPerson DP1;

    DP1.detection_id = 1;

    DP1.pose.pose.position.x = x1;
    DP1.pose.pose.position.y = y1;
    DP1.pose.pose.position.z = 0;

    DP1.confidence = 1;

    DPS.detections.push_back(DP1);

    spencer_tracking_msgs::DetectedPerson DP2;

    DP2.detection_id = 2;

    DP2.pose.pose.position.x = x2;
    DP2.pose.pose.position.y = y2;
    DP2.pose.pose.position.z = 0;

    DP2.confidence = 1;

    DPS.detections.push_back(DP2);

    spencer_tracking_msgs::DetectedPerson DP3;

    DP3.detection_id = 3;

    DP3.pose.pose.position.x = x3;
    DP3.pose.pose.position.y = y3;
    DP3.pose.pose.position.z = 0;

    DP3.confidence = 1;

    DPS.detections.push_back(DP3);

    DPS.header.frame_id = "people_simulator";
    DPS.header.stamp = ros::Time::now();
    m_pub_People.publish(DPS);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
