/***************************** Made by Duarte Gonçalves *********************************/

#include <stations_layer/StationLayer.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(stations_layer_namespace::StationLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
  double dx = x-x0, dy = y-y0;
  double h = sqrt(dx*dx+dy*dy);
  double angle = atan2(dy,dx);
  double mx = cos(angle-skew) * h;
  double my = sin(angle-skew) * h;
  double f1 = pow(mx, 2.0)/(2.0 * varx),
  f2 = pow(my, 2.0)/(2.0 * vary);
  return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var){
  return sqrt(-2*var * log(cutoff/A) );
}

namespace stations_layer_namespace
{

  StationLayer::StationLayer() {}

  void StationLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    stations_sub_ = nh.subscribe("/stations", 1, &StationLayer::stationCallback, this);

    server_ = new dynamic_reconfigure::Server<stations_layer::StationLayerConfig>(nh);
    f_ = boost::bind(&StationLayer::configure, this, _1, _2);
    server_->setCallback(f_);

    first_time_ = true;
  }

  void StationLayer::stationCallback(const thesis::DetectedStations& stations) {
    stations_list_ = stations;
  }

  void StationLayer::configure(stations_layer::StationLayerConfig &config, uint32_t level)
  {
    cutoff_ = config.cutoff;
    amplitude_ = config.amplitude;
    covar_ = config.covariance;
    factor_ = config.factor;
    enabled_ = config.enabled;
  }

  /*Defines the area that will be updated. We calculate the point we want to change and then expand the min/max bounds to be sure it includes the new point */
  void StationLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y){

    for(int z = 0; z < stations_list_.stations.size(); ++z){
      thesis::DetectedStation station = stations_list_.stations.at(z);

      double mag = station.magnitude;
      double factor = 1.0 + mag * factor_;
      double point = get_radius(cutoff_, amplitude_, covar_ * factor );

      *min_x = std::min(*min_x, station.center.x - point);
      *min_y = std::min(*min_y, station.center.y - point);
      *max_x = std::max(*max_x, station.center.x + point);
      *max_y = std::max(*max_y, station.center.y + point);

    }
  }

  /* First, we calculate which grid cell our point is in using worldToMap. Then we set the cost of that cell. */
  void StationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {

    if(!enabled_) return;

    if( stations_list_.stations.size() == 0 )
      return;
    if( cutoff_ >= amplitude_)
      return;

    std::list<thesis::DetectedStation>::iterator p_it; //Iterator for detected persons lsit

    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();
    int i = 0;

    for(i = 0; i < stations_list_.stations.size(); i++){
      thesis::DetectedStation station = stations_list_.stations.at(i);

      double angle = station.angle; // Detected Person Orientation
      double mag = station.magnitude; // A line of mag magnitude, i.e, length

      // Change factor_ variable accordingly to the task
      // to increase the gaussians length

      int prob = 1.0;

      double factor = prob + mag * factor_; //factor_ -> Factor to scale the velocity
      double base = get_radius(cutoff_, amplitude_, covar_);
      double point = get_radius(cutoff_, amplitude_, covar_ * factor );

      unsigned int width = std::max(1, int( (base + point) / res )),
      height = std::max(1, int( (base + point) / res ));

      double cx = station.center.x, cy = station.center.y; // Detected Person Center Coordinates

      double ox, oy;

      if(sin(angle)>0)
        oy = cy - base;
      else
        oy = cy + (point-base) * sin(angle) - base;

      if(cos(angle)>=0)
        ox = cx - base;
      else
        ox = cx + (point-base) * cos(angle) - base;

      int dx, dy; //Store our grid cell coordinate
      costmap->worldToMapNoBounds(ox, oy, dx, dy); //Calculate which grid cell our point is in

      int start_x = 0, start_y=0, end_x=width, end_y = height;
      if(dx < 0)
        start_x = -dx;
      else if(dx + width > costmap->getSizeInCellsX())
        end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx);

      if((int)(start_x+dx) < min_i)
        start_x = min_i - dx;
      if((int)(end_x+dx) > max_i)
        end_x = max_i - dx;

      if(dy < 0)
        start_y = -dy;
      else if(dy + height > costmap->getSizeInCellsY())
        end_y = std::max(0, (int) costmap->getSizeInCellsY() - dy);

      if((int)(start_y+dy) < min_j)
        start_y = min_j - dy;
      if((int)(end_y+dy) > max_j)
        end_y = max_j - dy;

      double bx = ox + res / 2,
      by = oy + res / 2;
      for(int i=start_x;i<end_x;i++){
        for(int j=start_y;j<end_y;j++){
          unsigned char old_cost = costmap->getCost(i+dx, j+dy);
          if(old_cost == costmap_2d::NO_INFORMATION)
            continue;

          double x = bx+i*res, y = by+j*res;
          double ma = atan2(y-cy,x-cx);
          double diff = angles::shortest_angular_distance(angle, ma);

          double a;
          if(fabs(diff)<M_PI/2)
            a = gaussian(x,y,cx,cy,amplitude_,covar_*factor,covar_,angle);
          else
            a = gaussian(x,y,cx,cy,amplitude_,covar_, covar_,0);

          if(a < cutoff_)
            continue;
          
          unsigned char cvalue = (unsigned char) a;
          costmap->setCost(i+dx, j+dy, std::max(cvalue, old_cost)); //Set cost to the desired position
        }
      }
    }
  }
};
