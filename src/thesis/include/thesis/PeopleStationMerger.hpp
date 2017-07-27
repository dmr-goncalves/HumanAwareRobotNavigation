/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <string>     // std::string, std::to_string
#include <math.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <thesis/DetectedPeople.h>
#include <thesis/DetectedPerson.h>
#include <thesis/DetectedStations.h>
#include <thesis/DetectedStation.h>

#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>

#include <tinyxml2.h>

using namespace grid_map;
using namespace tinyxml2;

typedef struct color{
  std::string name;
  Eigen::Vector3i color;
  double weight;
} SColor;

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
  
  ros::Time                                     initialTime, auxiliarTimeToNewPersons;

  sensor_msgs::Image                            colorImg;

  GridMap                                       gridMap;
  nav_msgs::OccupancyGrid                       map;

  bool                                          firstTime;

  geometry_msgs::PoseWithCovarianceStamped      robotLocalization;

  thesis::DetectedPeople                        dppl;
  thesis::DetectedStations                      DS;
  
  double HorizontalLeft, HorizontalRight, VerticalDown, VerticalUp;

  SColor color;
  std::vector<SColor> existingColors;

  void detectedPeopleClbk(const spencer_tracking_msgs::DetectedPersons DP);
  void trackedPeopleClbk(const spencer_tracking_msgs::TrackedPersons TP);
  void mapClbk(const nav_msgs::OccupancyGrid &updatedMap);
  void amclClbk(const geometry_msgs::PoseWithCovarianceStamped &localization);
  
  double getVelocity(double finalPosition, double initialPosition, double finalTime, double initialTime);
  std::pair<double,std::string> getWeight(float x, float y);
  void gridMapConstruction();
  void findStations();
  void getColors();
};
/* _thesis_HPP__ */
