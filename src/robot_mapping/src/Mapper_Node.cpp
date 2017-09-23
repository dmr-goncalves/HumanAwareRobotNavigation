/***************************** Made by Duarte Gonçalves *********************************/

#include "robot_mapping_app/Mapper.hpp"


int main( int argc, char** argv )
{
    ros::init( argc, argv, "Mapper_Node" );

    Mapper m;

    m.run();

    return 0;
}
