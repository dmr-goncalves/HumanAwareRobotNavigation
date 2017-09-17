/***************************** Made by Duarte Gonçalves *********************************/

#include "robot_mapping_app/Mapper.hpp"

Mapper::Mapper():m_nd("~"){
  //Configuration of the Subsribers and Publishers topics and callbacks
  m_pub_Map = m_nd.advertise<nav_msgs::OccupancyGrid>("/map",1);
  m_sub_Scan = m_nd.subscribe("/scan", 1, &Mapper::scanClbk, this);
  m_sub_Odom = m_nd.subscribe( "/odometry/filtered", 1, &Mapper::odomClbk, this );

  //Map dimensions, position and orientation
  mapResolution = 0.05;
  mapWidth = 4000;
  mapHeight = 4000;
  mapOriginPosX = 0.0;
  mapOriginPosY = 0.0;
  mapOriginPosZ = 0.0;
  mapOriginOrX = 0.0;
  mapOriginOrY = 0.0;
  mapOriginOrZ = 0.0;
  mapOriginOrW = 0.0;
}

void Mapper::run(){

  //Populate the OccupancyGrid object with the dimensions, position and orientation
  map.info.resolution = mapResolution;
  map.info.width = mapWidth;
  map.info.height = mapHeight;
  map.info.origin.position.x = mapOriginPosX;
  map.info.origin.position.y = mapOriginPosY;
  map.info.origin.position.z = mapOriginPosZ;
  map.info.origin.orientation.x = mapOriginOrX;
  map.info.origin.orientation.y = mapOriginOrY;
  map.info.origin.orientation.z = mapOriginOrZ;
  map.info.origin.orientation.w = mapOriginOrW;

  //At the beggining all the locations are unknown
  for(unsigned int i=0; i < mapWidth * mapHeight; i++){
    map.data.push_back(-1);
  }
  ros::spin();
}

//Save Odometry Values
void Mapper::odomClbk(const nav_msgs::Odometry Odom){
  odom = Odom;
  robotAngle = odom.pose.pose.orientation.z * M_PI;
}

//Get LaserScan values and build the map
void Mapper::scanClbk(const sensor_msgs::LaserScan scan){

  //Save LaserScan Parameters
  range_min = scan.range_min;
  range_max = scan.range_max;
  angle_min = scan.angle_min;
  angle_increment = scan.angle_increment;

  //Search all measurements
  for(unsigned int w = 0; w < scan.ranges.size(); w++){

    //Current Range
    range = scan.ranges[w];

    //Curruent Robot Position
    xR = odom.pose.pose.position.x;
    yR = odom.pose.pose.position.y;

    //Angle of the measurement
    obstacleAngle = angle_min + w * angle_increment;

    //Position of the measurement
    xO = xR + range * cos(robotAngle + obstacleAngle);
    yO = yR + range * sin(robotAngle + obstacleAngle);

    //Correspondent position in the grid
    gridX = xO / mapResolution + mapWidth / 2;
	  gridY = yO / mapResolution + mapHeight / 2;

    //Populate the OccupancyGrid data vector. A better approach would be to populate it with a probability other than 100%, measuring our belief
	  map.data[gridX + gridY * mapWidth] = 100;

    //Paint white until we reach our range
    for(float f = 0.01; f < range; f += 0.01){

      //Position of the measurement
      xO = xR + f * cos(robotAngle + obstacleAngle);
      yO = yR + f * sin(robotAngle + obstacleAngle);

      //Correspondent position in the grid
  		gridXW = xO / mapResolution + mapWidth / 2;
  		gridYW = yO / mapResolution + mapHeight / 2;

      //We have seen black in this position but now we don't so we probably were mistaken and we clean that position
  		if((gridXW != gridX) || (gridY != gridYW)){
  		  map.data[gridXW + gridYW * mapWidth] = 0;
  	  }
    }
  }
  m_pub_Map.publish(map);
}
