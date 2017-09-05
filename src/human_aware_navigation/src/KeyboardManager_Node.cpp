/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/KeyboardManager.hpp"


int main( int argc, char** argv )
{
  ros::init( argc, argv, "KeyboardManager");

  KeyboardManager KM;

  KM.run();

  return 0;
}
