/***************************** Made by Duarte Gonçalves *********************************/

#include "simulator_app/simulator.hpp"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "simulator_node");

  simulator sim;

  sim.run();

  return 0;
}
