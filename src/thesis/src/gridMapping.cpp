/***************************** Made by Duarte Gonçalves *********************************/

#include "thesis/gridMapping.hpp"

gridMapping::gridMapping():m_nd("~"){
  m_pub_GridMap = m_nd.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);

  gridMap = GridMap({"color"});
  gridMap.setBasicLayers({"color"});

  gridMap.setFrameId("map");

  gridMap.setGeometry(Length(0.5, 0.5), 0.03);

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
  gridMap.getLength().x(), gridMap.getLength().y(),
  gridMap.getSize()(0), gridMap.getSize()(1));

  for (GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
    Index index = *it;

    Eigen::Vector3i colorVector;

    colorVector(0) = 25; //R
    colorVector(1) = 0;   //G
    colorVector(2) = 0;   //B

    float value;

    colorVectorToValue(colorVector, value);
    //ROS_ERROR("Value %lu", value);
    gridMap.at("color", index) = value;
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(gridMap, message);
    m_pub_GridMap.publish(message);
  }
}

void gridMapping::run(){

  ros::Rate rate(0.5);

  while (ros::ok()) {
    Index index2 = Index(1, 1);

    Eigen::Vector3f colorVector;

    float value = gridMap.at("color",index2);

    ROS_ERROR("%f", value);

    colorValueToVector(value, colorVector);

    Eigen::Vector3i colorVectorAnotherBase;

    colorVectorAnotherBase(0) = colorVector(0) * 255;
    colorVectorAnotherBase(1) = colorVector(1) * 255;
    colorVectorAnotherBase(2) = colorVector(2) * 255;

    ROS_ERROR("(%d, %d, %d)", colorVectorAnotherBase(0), colorVectorAnotherBase(1), colorVectorAnotherBase(2));


    rate.sleep();
    ros::spinOnce();
  }
}
