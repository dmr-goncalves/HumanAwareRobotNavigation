/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/GoalSender.hpp"


int main( int argc, char** argv )
{
  init( argc, argv, "GoalSender");

  GoalSender GM;

  GM.run();

  return 0;
}
