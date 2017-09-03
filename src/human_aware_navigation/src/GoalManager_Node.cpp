/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/GoalManager.hpp"


int main( int argc, char** argv )
{
  ros::init( argc, argv, "GoalManager_Node");

  GoalManager GM;

  GM.sendGoal();

  return 0;
}
