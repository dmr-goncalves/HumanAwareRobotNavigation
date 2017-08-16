/***************************** Made by Duarte Gonçalves *********************************/
#ifndef STATION_LAYER_H_
#define STATION_LAYER_H_

#include <ros/ros.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <dynamic_reconfigure/server.h>
#include <stations_layer/StationLayerConfig.h>

#include <thesis/DetectedStations.h>

#include <math.h>

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

namespace stations_layer_namespace{

  class StationLayer : public costmap_2d::Layer{

  public:
    StationLayer();

    virtual void onInitialize();
    virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  private:
    void stationCallback(const thesis::DetectedStations& stations_);

    ros::Subscriber stations_sub_;
    thesis::DetectedStations stations_list_;

    void configure(stations_layer::StationLayerConfig &config, uint32_t level);

    double cutoff_, amplitude_, covar_, factor_;
    dynamic_reconfigure::Server<stations_layer::StationLayerConfig>* server_;
    dynamic_reconfigure::Server<stations_layer::StationLayerConfig>::CallbackType f_;

    bool first_time_;    
  };
}

#endif
