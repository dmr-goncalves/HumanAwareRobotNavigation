/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>

#include <spencer_tracking_msgs/DetectedPersons.h>

class simulator
{
public:

  simulator ();
  virtual ~simulator (){};

  void run();

private:

  ros::NodeHandle                               m_nd;
  ros::Publisher                                m_pub_People;
  float                                         x1;
  float                                         y1;
  float                                         z1;
  int                                           x2;
  int                                           y2;
  int                                           x3;
  int                                           y3;

};
/* _Simulator_HPP__ */
