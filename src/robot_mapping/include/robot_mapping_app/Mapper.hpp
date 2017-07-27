/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>

    class Mapper
    {
        public:

            Mapper ();
            virtual ~Mapper (){};

            void run();

        private:


            ros::NodeHandle                               m_nd;
            ros::Publisher                                m_pub_Map; // /myMap
            ros::Subscriber                               m_sub_Odom; // /odom
            ros::Subscriber                               m_sub_Scan; // /scan
            nav_msgs::OccupancyGrid                       map;
            nav_msgs::Odometry                            odom; //Save odometry values
            float                                         mapResolution;
            int                                           mapWidth;
            int                                           mapHeight;
            double                                        mapOriginPosX;
            double                                        mapOriginPosY;
            double                                        mapOriginPosZ;
            double                                        mapOriginOrX;
            double                                        mapOriginOrY;
            double                                        mapOriginOrZ;
            double                                        mapOriginOrW;
            int                                           gridX;
            int                                           gridY;
            int                                           gridXW;
            int                                           gridYW;
            double                                        range;
            double                                        range_min;
            double                                        range_max;
            double                                        angle_increment;
            double                                        angle_min;
            double                                        robotAngle;
            double                                        xR;
            double                                        yR;
            double                                        xO;
            double                                        yO;
            double                                        obstacleAngle;
            
            void odomClbk(const nav_msgs::Odometry Odom);
            void scanClbk(const sensor_msgs::LaserScan scan);
    };
 /* _Mapper_HPP__ */
