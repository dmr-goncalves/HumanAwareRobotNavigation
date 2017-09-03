/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <string>     // std::string, std::to_string
#include <math.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <human_aware_navigation/DetectedPeople.h>
#include <human_aware_navigation/DetectedPerson.h>
#include <human_aware_navigation/DetectedStations.h>
#include <human_aware_navigation/DetectedStation.h>

#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>

#include <tinyxml2.h>

#include <utilities/printer.hpp>

using namespace grid_map;
using namespace tinyxml2;

typedef struct label{
  double side;
  std::string type;
  Eigen::Vector3i color;
  double weight;
} SLabel;

typedef struct triple
{
  int  first;
  std::string second;
  double third;
} STriple;


class PeopleStationMerger
{
public:

  PeopleStationMerger ();
  virtual ~PeopleStationMerger (){};

  void run();

private:

  ros::NodeHandle                               m_nd;
  ros::Publisher                                m_pub_People, m_pub_GridMap, m_pub_Stations;
  image_transport::Publisher                    m_pub_Image;
  ros::Subscriber                               m_sub_DetectedPersons, m_sub_TrackedPersons, m_sub_Odom;
  ros::Subscriber                               m_sub_Map, m_sub_AMCL;
  
  sensor_msgs::Image                            colorImg;

  GridMap                                       gridMap;
  nav_msgs::OccupancyGrid                       map;

  bool                                          firstTime;

  geometry_msgs::PoseWithCovarianceStamped      robotLocalization;

  human_aware_navigation::DetectedPeople        dppl;
  human_aware_navigation::DetectedStations      DS;

  SLabel label;
  std::vector<SLabel>                           existingLabels;

  double left, right, down, up;

  void detectedPeopleClbk(const spencer_tracking_msgs::DetectedPersons DP);
  void trackedPeopleClbk(const spencer_tracking_msgs::TrackedPersons TP);
  void mapClbk(const nav_msgs::OccupancyGrid &updatedMap);
  void amclClbk(const geometry_msgs::PoseWithCovarianceStamped &localization);
  
  double getVelocity(double finalPosition, double initialPosition, double finalTime, double initialTime);
  STriple getWeight(float x, float y);
  void gridMapConstruction();
  void findStations();
  void getLabels();
};
/* _human_aware_navigation_HPP__ */
