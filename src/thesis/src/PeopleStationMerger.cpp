/***************************** Made by Duarte Gonçalves *********************************/

#include "thesis/PeopleStationMerger.hpp"

PeopleStationMerger::PeopleStationMerger():m_nd("~"){

  /* Configuration of the Subsribers and Publishers topics and callbacks */
  m_sub_DetectedPersons = m_nd.subscribe("/spencer/perception/detected_persons", 1, &PeopleStationMerger::detectedPeopleClbk, this);
  m_sub_TrackedPersons = m_nd.subscribe("/spencer/perception/tracked_persons", 1, &PeopleStationMerger::trackedPeopleClbk, this);
  m_sub_Map = m_nd.subscribe("/map", 1, &PeopleStationMerger::mapClbk, this);
  m_sub_AMCL = m_nd.subscribe("/amcl_pose", 1, &PeopleStationMerger::amclClbk, this);

  m_pub_People = m_nd.advertise<thesis::DetectedPeople>("/people", 1, true);
  m_pub_GridMap = m_nd.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  m_pub_Stations = m_nd.advertise<thesis::DetectedStations>("/stations", 1, true);

  image_transport::ImageTransport it(m_nd);
  m_pub_Image = it.advertise("/labeledMapImage", 1);


  /* Configuration of auxiliar variables */
  initialTime = ros::Time::now();
  auxiliarTimeToNewPersons = ros::Time::now();
  firstTime = true;

  /* Configuration of grid_map variable */
  gridMap = grid_map::GridMap({"color"});

  /* Frame id attribution */
  dppl.header.frame_id = "people";

  /* Configuration of stations orientations */
  VerticalDown = M_PI;
  VerticalUp = 0;
  HorizontalLeft = 0.5 * M_PI ;
  HorizontalRight = 1.5 * M_PI;

  PeopleStationMerger::getColors();
}

void PeopleStationMerger::run(){
  ros::spin();
}

void PeopleStationMerger::detectedPeopleClbk(const spencer_tracking_msgs::DetectedPersons DP){

  thesis::DetectedPeople dpplAux;
  dpplAux.header.frame_id = "people";

  for(int x = 0; x < DP.detections.size(); x++){

    std::stringstream ss;//create a stringstream

    ss << DP.detections.at(x).detection_id;//add number to the stream

    thesis::DetectedPerson dp;

    dp.name = ss.str();
    dp.position = DP.detections.at(x).pose.pose.position;
    dp.confidence = DP.detections.at(x).confidence;

    std::pair<double,std::string> dpPair = getWeight(-dp.position.x, -dp.position.y);
    
    dp.weight = dpPair.first;
    dp.workstation = dpPair.second;

    dpplAux.people.push_back(dp);
  }

  dpplAux.header.stamp = DP.header.stamp;
  dppl = dpplAux;
  m_pub_People.publish(dppl);
}

void PeopleStationMerger::trackedPeopleClbk(const spencer_tracking_msgs::TrackedPersons TP){
  for(int x = 0; x < TP.tracks.size(); x++){
    if(TP.tracks.at(x).is_matched && !TP.tracks.at(x).is_occluded){
      for(int  c = 0; c < dppl.people.size(); c++){
        std::stringstream ss;//create a stringstream
        ss << TP.tracks.at(x).detection_id;
        if(dppl.people.at(c).name == ss.str()){ //To update people velocities and positions
          dppl.people.at(c).velocity.x = TP.tracks.at(x).twist.twist.linear.x;
          dppl.people.at(c).velocity.y = TP.tracks.at(x).twist.twist.linear.y;
          dppl.people.at(c).velocity.z = TP.tracks.at(x).twist.twist.linear.z;
          dppl.people.at(c).position = TP.tracks.at(x).pose.pose.position;
          c = dppl.people.size();
        }
      }
    }
  }
  m_pub_People.publish(dppl);
}

std::pair<double,std::string> PeopleStationMerger::getWeight(float x, float y){

  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(gridMap, message);
  float mapResolution = message.info.resolution;

  int crosswalkIndex = -1.0;

  geometry_msgs::Pose mapOrigin = message.info.pose;

  mapOrigin.position.x = message.info.pose.position.x - (message.info.length_x / 2);
  mapOrigin.position.y = message.info.pose.position.y - (message.info.length_y / 2);

  int gridX = (x - mapOrigin.position.x) / mapResolution;
  int gridY = (y - mapOrigin.position.y) / mapResolution;

  Index index = Index(gridX, gridY);

  Position center;

  std::pair<double, std::string> resultPair;

  gridMap.getPosition(index, center);
  double radius = 0.6;
  double peopleCrosswalkRadius = 2.0;

  for(int i = 0; i < existingColors.size(); i++){
    if(!existingColors.at(i).name.compare("crosswalk")){
     crosswalkIndex = i;
   }
 }

 if(crosswalkIndex > -1){
   for (grid_map::CircleIterator peopleCrosswalkIterator(gridMap, center, peopleCrosswalkRadius); !peopleCrosswalkIterator.isPastEnd(); ++peopleCrosswalkIterator) {

    float valuePeopleCrosswalkSearch = gridMap.at("color", *peopleCrosswalkIterator);

    Eigen::Vector3f colorVectorPeopleCrosswalk; 

    colorValueToVector(valuePeopleCrosswalkSearch, colorVectorPeopleCrosswalk);

    Eigen::Vector3i colorVectorPeopleCrosswalkAnotherBase;

    colorVectorPeopleCrosswalkAnotherBase(0) = colorVectorPeopleCrosswalk(0) * 255;
    colorVectorPeopleCrosswalkAnotherBase(1) = colorVectorPeopleCrosswalk(1) * 255;
    colorVectorPeopleCrosswalkAnotherBase(2) = colorVectorPeopleCrosswalk(2) * 255;

  if(colorVectorPeopleCrosswalkAnotherBase == existingColors.at(crosswalkIndex).color){ //Check if we found the crosswalk*/

    //To iterate in a given radius around the person
    for (grid_map::CircleIterator colorIterator(gridMap, center, radius); !colorIterator.isPastEnd(); ++colorIterator) {

      float colorValue = gridMap.at("color", *colorIterator);

      Eigen::Vector3f colorVector;

      colorValueToVector(colorValue, colorVector);

      Eigen::Vector3i colorVectorAnotherBase;

      colorVectorAnotherBase(0) = colorVector(0) * 255;
      colorVectorAnotherBase(1) = colorVector(1) * 255;
      colorVectorAnotherBase(2) = colorVector(2) * 255;

      for(int x = 0; x < existingColors.size(); x++){

        if(colorVectorAnotherBase == existingColors.at(x).color && x != crosswalkIndex){
          resultPair.first = existingColors.at(x).weight;
          resultPair.second = existingColors.at(x).name;
          return resultPair;
        }
      }

      if(colorIterator.isPastEnd()){
        resultPair.first = 0.0;
        resultPair.second = "none";
        return resultPair;
      }
    }
  }else{
    if(peopleCrosswalkIterator.isPastEnd()){
      resultPair.first = 0.0;        
      resultPair.second = "none";
      return resultPair;
    }
  }
}
}else{
 resultPair.first = 0.0;        
 resultPair.second = "none";
 return resultPair; 
}
}



void PeopleStationMerger::gridMapConstruction(){

  std::string uri = ros::package::getPath("thesis") + "/misc/myLabeledMapWithCrosswalk.png";

  cv::Mat image = cv::imread(uri.c_str(), CV_LOAD_IMAGE_COLOR );

  if(!image.data )// Check for invalid input
  {
    std::cout <<  "Could not open or find the image" << std::endl ;
  }

  if (image.empty())
  {
    std::cout << "Error : Image cannot be loaded..!!" << std::endl;
  }

  cv::cvtColor(image, image, CV_BGR2RGB);

  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, image);

  img_bridge.toImageMsg(colorImg); // from cv_bridge to sensor_msgs::Image

  colorImg.header.stamp = ros::Time::now();

  m_pub_Image.publish(colorImg);

  gridMap.setGeometry(Length(map.info.height * map.info.resolution, map.info.width * map.info.resolution), map.info.resolution);

  GridMapRosConverter::addColorLayerFromImage(colorImg, "color", gridMap);

  gridMap.setFrameId("gridmap");

  gridMap.setBasicLayers({"color"});

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    gridMap.getLength().x(), gridMap.getLength().y(), gridMap.getSize()(0), gridMap.getSize()(1));

  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(gridMap, message);

  PeopleStationMerger::findStations();
  m_pub_GridMap.publish(message);
}

void PeopleStationMerger::mapClbk(const nav_msgs::OccupancyGrid &udpatedMap){
  if(firstTime){
    map = udpatedMap;
    PeopleStationMerger::gridMapConstruction(); //Build gridmap
    firstTime = false;
  }
}

void PeopleStationMerger::amclClbk(const geometry_msgs::PoseWithCovarianceStamped &localization){
  robotLocalization = localization;
}

double PeopleStationMerger::getVelocity(double finalPosition, double initialPosition, double finalTime, double initialTime){
  return (finalPosition - initialPosition) / (finalTime - initialTime);
}

void PeopleStationMerger::findStations(){

  thesis::DetectedStations DS;

  DS.header.stamp = ros::Time::now();
  DS.header.frame_id = "stations";

  XMLDocument doc;

  std::string uri = ros::package::getPath("thesis") + "/misc/stations.xml";

  doc.LoadFile(uri.c_str());

  XMLElement* stations_location_library = doc.FirstChildElement("stations_location_library");

  for(XMLElement* e = stations_location_library->FirstChildElement("station"); e != NULL; e = e->NextSiblingElement("station") ){

    thesis::DetectedStation ds;

    const char* color = e->Attribute("category");

    std::string color_(color);

    if(color_ == "red"){
      ds.name = "red";
      ds.magnitude = 20;
    }else if(color_ == "yellow"){
      ds.name = "yellow";
      ds.magnitude = 15;
    }else if(color_ == "blue"){
      ds.name = "blue";
      ds.magnitude = 10;
    }else if(color_ == "green"){
      ds.name = "green";
      ds.magnitude = 5;
    }

    e->FirstChildElement("center")->FirstChildElement("x")->QueryDoubleText(&ds.center.x);
    e->FirstChildElement("center")->FirstChildElement("y")->QueryDoubleText(&ds.center.y);
    e->FirstChildElement("center")->FirstChildElement("z")->QueryDoubleText(&ds.center.z);

    const char* orientation = e->FirstChildElement("peopleWalk")->GetText();

    std::string orientation_(orientation);

    if(orientation_ == "left"){
      ds.angle = HorizontalLeft;
    }else if(orientation_ == "right"){
      ds.angle = HorizontalRight;
    }else if(orientation_ == "up"){
      ds.angle = VerticalUp;
    }else if(orientation_ == "down"){
      ds.angle = VerticalDown;
    }
    
    DS.stations.push_back(ds);
  }

  m_pub_Stations.publish(DS);
}

void PeopleStationMerger::getColors(){

  XMLDocument doc;

  std::string uri = ros::package::getPath("thesis") + "/misc/colors.xml";

  doc.LoadFile(uri.c_str());

  XMLElement* colors_library = doc.FirstChildElement("colors_library");

  for(XMLElement* e = colors_library->FirstChildElement("color"); e != NULL; e = e->NextSiblingElement("color") ){
    const char* name = e->Attribute("category");

    Eigen::Vector3i cl;
    double r,g,b, wght;

    e->FirstChildElement("R")->QueryDoubleText(&r);
    e->FirstChildElement("G")->QueryDoubleText(&g);
    e->FirstChildElement("B")->QueryDoubleText(&b);
    e->FirstChildElement("weight")->QueryDoubleText(&wght);

    cl(0) = r;
    cl(1) = g;
    cl(2) = b;

    color.name = name;
    color.color = cl;
    color.weight = wght;

    existingColors.push_back(color);
  }
}