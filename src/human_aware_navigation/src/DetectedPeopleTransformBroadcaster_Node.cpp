/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/DetectedPeopleTransformBroadcaster.hpp"


int main( int argc, char** argv )
{
  init( argc, argv, "DetectedPeopleTransformBroadcaster");

  DetectedPeopleTransformBroadcaster DPT;

  DPT.run();

  return 0;
}
