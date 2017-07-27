/***************************** Made by Duarte Gonçalves *********************************/

#include "thesis/gridMapping.hpp"


int main( int argc, char** argv )
{
  ros::init( argc, argv, "gridMapping_Node");

  gridMapping gm;

  gm.run();

  return 0;
}
