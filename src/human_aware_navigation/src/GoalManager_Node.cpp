/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/GoalManager.hpp"


int main( int argc, char** argv )
{
  ros::init( argc, argv, "GoalManager");

  GoalManager GM;

  GM.run();

  return 0;
}
