/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/PeopleStationMerger.hpp"


int main( int argc, char** argv )
{
  ros::init( argc, argv, "PeopleStationMerger_Node");

  PeopleStationMerger PSM;

  PSM.run();

  return 0;
}
