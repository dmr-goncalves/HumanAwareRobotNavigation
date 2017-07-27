/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include "grid_map_core/GridMapMath.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>


using namespace grid_map;

class gridMapping
{
public:

  gridMapping ();
  virtual ~gridMapping (){};

  void run();

private:

  ros::NodeHandle                               m_nd;
  ros::Publisher                                m_pub_GridMap;
  GridMap                                       gridMap;

};
/* _girdMapping_HPP__ */
