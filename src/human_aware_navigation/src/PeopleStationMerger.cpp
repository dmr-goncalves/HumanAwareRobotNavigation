/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/PeopleStationMerger.hpp"

PeopleStationMerger::PeopleStationMerger():m_nd("~"){

  /* Configuration of the Subsribers and Publishers topics and callbacks */
  m_sub_DetectedPersons = m_nd.subscribe("/detected_people", 1, &PeopleStationMerger::detectedPeopleClbk, this);
  //m_sub_TrackedPersons = m_nd.subscribe("/tracked_people", 1, &PeopleStationMerger::trackedPeopleClbk, this);
  m_sub_Map = m_nd.subscribe("/map", 1, &PeopleStationMerger::mapClbk, this);
  m_sub_AMCL = m_nd.subscribe("/amcl_pose", 1, &PeopleStationMerger::amclClbk, this);

  m_pub_People = m_nd.advertise<human_aware_navigation::DetectedPeople>("/people", 1, true);
  m_pub_GridMap = m_nd.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  m_pub_Stations = m_nd.advertise<human_aware_navigation::DetectedStations>("/stations", 1, true);
  m_pub_resultPose = m_nd.advertise<geometry_msgs::PoseWithCovarianceStamped>("/person_result_pose", 1, true);

  image_transport::ImageTransport it(m_nd);
  m_pub_Image = it.advertise("/labeledMapImage", 1);

  /* Configuration of auxiliar variables */
  firstTime = true;

  /* Configuration of grid_map variable */
  gridMap = grid_map::GridMap({"color"});

  /* Frame id attribution */
  dppl.header.frame_id = "people";

  /* Configuration of custom orientations */
  up = 0;
  down = M_PI;
  left = 0.5 * M_PI ;
  right = 1.5 * M_PI;

  PeopleStationMerger::getLabels();

  foundLabel = false;
  foundCrosswalk = false;

}

void PeopleStationMerger::run(){
  ros::spin();
}

void PeopleStationMerger::detectedPeopleClbk(const spencer_tracking_msgs::DetectedPersons DP){

  human_aware_navigation::DetectedPeople dpplAux;
  dpplAux.header.frame_id = "people";

  TransformListener tf_listener;

  for(int x = 0; x < DP.detections.size(); x++){

    StampedTransform transform;

    ros::Duration(10.0).sleep();

    stringstream ss;//create a stringstream

    ss << DP.detections.at(x).detection_id;//add number to the stream

    human_aware_navigation::DetectedPerson dp;

    dp.name = ss.str();

    string frame_id = "detected_person_1";

    try{
      tf_listener.lookupTransform("map", frame_id, ros::Time(0), transform);

      dp.position.x = transform.getOrigin().x();
      dp.position.y = transform.getOrigin().y();
      dp.position.z = transform.getOrigin().z();

      geometry_msgs::Quaternion q;

      q.x = 0;
      q.y = 0;
      q.z = 0;
      q.w = 1;

      dp.orientation = q;

      dp.confidence = DP.detections.at(x).confidence;

      geometry_msgs::PoseWithCovarianceStamped ps;

      ps.header.frame_id = "persons_pose";


      ps.pose.pose.position = dp.position;
      ps.pose.pose.orientation = dp.orientation;

      m_pub_resultPose.publish(ps);

      STriple dpTriple = getWeight(-dp.position.x, -dp.position.y);

      dp.weight = dpTriple.first;
      dp.workstation = dpTriple.second;
      dp.angle = dpTriple.third;

      dpplAux.people.push_back(dp);

    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  dpplAux.header.stamp = DP.header.stamp;
  dppl = dpplAux;
  m_pub_People.publish(dppl);
}

/*void PeopleStationMerger::trackedPeopleClbk(const spencer_tracking_msgs::TrackedPersons TP){
  for(int x = 0; x < TP.tracks.size(); x++){
    if(TP.tracks.at(x).is_matched && !TP.tracks.at(x).is_occluded){
      for(int  c = 0; c < dppl.people.size(); c++){
        stringstream ss;//create a stringstream
        ss << TP.tracks.at(x).detection_id;
        if(dppl.people.at(c).name == ss.str()){ //To update people velocities and positions

          dppl.people.at(c).velocity.x = TP.tracks.at(x).twist.twist.linear.x;
          dppl.people.at(c).velocity.y = TP.tracks.at(x).twist.twist.linear.y;
          dppl.people.at(c).velocity.z = TP.tracks.at(x).twist.twist.linear.z;
          dppl.people.at(c).position.x = TP.tracks.at(x).pose.pose.position.z;
          dppl.people.at(c).position.y = TP.tracks.at(x).pose.pose.position.y;
          dppl.people.at(c).position.z = TP.tracks.at(x).pose.pose.position.x;


          STriple dpTriple = getWeight(-dppl.people.at(c).position.x, -dppl.people.at(c).position.y);

          dppl.people.at(c).weight = dpTriple.first;
          dppl.people.at(c).workstation = dpTriple.second;
          dppl.people.at(c).angle = dpTriple.third;

          c = dppl.people.size();
        }
      }
    }
  }
  m_pub_People.publish(dppl);
}*/

STriple PeopleStationMerger::getWeight(float x, float y){

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

  Position personsCenter;

  STriple resultTriple;

  gridMap.getPosition(index, personsCenter);

  double radius = 1.0;
  double peopleCrosswalkRadius = 15.0;

  if(existingLabels.size() == 0){
    resultTriple.first = 0.0;
    resultTriple.second = "No labels";
    resultTriple.third = 0.0;
    printer::printRed("No labels!");
    return resultTriple;
  }

  for(int i = 0; i < existingLabels.size(); i++){
    if(!existingLabels.at(i).type.compare("crosswalk")){
      crosswalkIndex = i;
    }
  }

  if(crosswalkIndex == -1){
    resultTriple.first = 0.0;
    resultTriple.second = "No crosswalk";
    resultTriple.third = 0.0;
    printer::printRed("No crosswalk!");
    return resultTriple;
  }


  if(existingLabels.size() > 1){

    for (grid_map::CircleIterator peopleCrosswalkIterator(gridMap, personsCenter, peopleCrosswalkRadius); !peopleCrosswalkIterator.isPastEnd(); ++peopleCrosswalkIterator) {

      float valuePeopleCrosswalkSearch = gridMap.at("color", *peopleCrosswalkIterator);

      Eigen::Vector3f colorVectorPeopleCrosswalk;

      colorValueToVector(valuePeopleCrosswalkSearch, colorVectorPeopleCrosswalk);

      Eigen::Vector3i colorVectorPeopleCrosswalkAnotherBase;

      colorVectorPeopleCrosswalkAnotherBase(0) = colorVectorPeopleCrosswalk(0) * 255;
      colorVectorPeopleCrosswalkAnotherBase(1) = colorVectorPeopleCrosswalk(1) * 255;
      colorVectorPeopleCrosswalkAnotherBase(2) = colorVectorPeopleCrosswalk(2) * 255;

      if(colorVectorPeopleCrosswalkAnotherBase == existingLabels.at(crosswalkIndex).color){ //Check if we found the crosswalk*/

      foundCrosswalk = true;

        //To iterate in a given radius around the person
      for (grid_map::CircleIterator colorIterator(gridMap, personsCenter, radius); !colorIterator.isPastEnd(); ++colorIterator) {

        float colorValue = gridMap.at("color", *colorIterator);

        Eigen::Vector3f colorVector;

        colorValueToVector(colorValue, colorVector);

        Eigen::Vector3i colorVectorAnotherBase;

        colorVectorAnotherBase(0) = colorVector(0) * 255;
        colorVectorAnotherBase(1) = colorVector(1) * 255;
        colorVectorAnotherBase(2) = colorVector(2) * 255;

        for(int x = 0; x < existingLabels.size(); x++){

          if(colorVectorAnotherBase == existingLabels.at(x).color && x != crosswalkIndex){

            foundLabel = true;

            resultTriple.first = existingLabels.at(x).weight;
            resultTriple.second = existingLabels.at(x).type;
            resultTriple.third = existingLabels.at(x).side;
            printer::printRed("Found with label -> " + existingLabels.at(x).type);
            return resultTriple;
          }
        }
      }
    }
  }
}else{
  resultTriple.first = 0.0;
  resultTriple.second = "No labels other than the crosswalk";
  resultTriple.third = 0.0;
  printer::printRed("No labels other that the crosswalk!");
  return resultTriple;
}

if(!foundCrosswalk){
  resultTriple.first = 0.0;
  resultTriple.second = "No crosswalk found near the person";
  resultTriple.third = 0.0;
  printer::printRed("No crosswalk found near the person");
  return resultTriple;
}

if(!foundLabel && foundCrosswalk){
  resultTriple.first = 0.0;
  resultTriple.second = "No labels found near the person";
  resultTriple.third = 0.0;
  printer::printRed("No labels found near the person");
  return resultTriple;
}
}

void PeopleStationMerger::gridMapConstruction(){

  string uri = ros::package::getPath("human_aware_navigation") + "/misc/ricsLabeledMap1.png";

  cv::Mat image = cv::imread(uri.c_str(), CV_LOAD_IMAGE_COLOR );

  if(!image.data )// Check for invalid input
  {
    printer::printRed("Could not open or find the image");
    ros::shutdown();
  }

  if (image.empty())
  {
    printer::printRed("Error : Image cannot be loaded..!!");
    ros::shutdown();
  }

  cv::cvtColor(image, image, CV_BGR2RGB);

  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, image);

  img_bridge.toImageMsg(colorImg); // from cv_bridge to sensor_msgs::Image

  colorImg.header.stamp = ros::Time::now();

  m_pub_Image.publish(colorImg);

  GridMapRosConverter::fromOccupancyGrid(map, "map", gridMap);

  GridMapRosConverter::addColorLayerFromImage(colorImg, "color", gridMap);

  gridMap.setFrameId("gridmap");

  gridMap.setBasicLayers({"color"});

  string str = "Created grid_map with size " + to_string(gridMap.getLength().x()) + " x " + to_string(gridMap.getLength().y()) + "m (" + to_string(gridMap.getSize()(0)) + " x " + to_string(gridMap.getSize()(1)) + " cels).";

  printer::printGreen(str);

  grid_map_msgs::GridMap message;

  GridMapRosConverter::toMessage(gridMap, message);

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

  human_aware_navigation::DetectedStations DS;

  DS.header.stamp = ros::Time::now();
  DS.header.frame_id = "stations";

  XMLDocument stationsDoc;

  string stationsURI = ros::package::getPath("human_aware_navigation") + "/misc/stationsExample.xml";

  if(!stationsDoc.LoadFile(stationsURI.c_str())){

    XMLElement* stations = stationsDoc.FirstChildElement("stations");

    for(XMLElement* e = stations->FirstChildElement("station"); e != NULL; e = e->NextSiblingElement("station")){

      human_aware_navigation::DetectedStation ds;

      const char* type = e->Attribute("type");

      string type_(type);

      if(type_ == "red"){
        ds.name = "red";
        ds.magnitude = 20;
      }else if(type_ == "yellow"){
        ds.name = "yellow";
        ds.magnitude = 15;
      }else if(type_ == "blue"){
        ds.name = "blue";
        ds.magnitude = 10;
      }else if(type_ == "green"){
        ds.name = "green";
        ds.magnitude = 5;
      }

      e->FirstChildElement("center")->FirstChildElement("x")->QueryDoubleText(&ds.center.x);
      e->FirstChildElement("center")->FirstChildElement("y")->QueryDoubleText(&ds.center.y);
      e->FirstChildElement("center")->FirstChildElement("z")->QueryDoubleText(&ds.center.z);

      const char* orientation = e->FirstChildElement("crosswalk")->GetText();

      string orientation_(orientation);

      if(orientation_ == "left"){
        ds.angle = left;
      }else if(orientation_ == "right"){
        ds.angle = right;
      }else if(orientation_ == "up"){
        ds.angle = up;
      }else if(orientation_ == "down"){
        ds.angle = down;
      }

      DS.stations.push_back(ds);
    }

    m_pub_Stations.publish(DS);
  }else{
    printer::printRed("Could not open or find stations.xml file");
    ros::shutdown();
  }
}

void PeopleStationMerger::getLabels(){

  XMLDocument labelsDoc;


  string labelsURI = ros::package::getPath("human_aware_navigation") + "/misc/labels.xml";

  if(!labelsDoc.LoadFile(labelsURI.c_str())){

    printer::printGreen("Reading labels.xml file");

    XMLElement* labels = labelsDoc.FirstChildElement("labels");

    for(XMLElement* e = labels->FirstChildElement("label"); e != NULL; e = e->NextSiblingElement("label")){

      const char* type = e->Attribute("type");

      Eigen::Vector3i cl;
      double r,g,b, wght;
      string side = "";

      e->FirstChildElement("R")->QueryDoubleText(&r);
      e->FirstChildElement("G")->QueryDoubleText(&g);
      e->FirstChildElement("B")->QueryDoubleText(&b);

      if(e->FirstChildElement("weight") != NULL){
        e->FirstChildElement("weight")->QueryDoubleText(&wght);
      }else{
        wght = 0.0;
      }

      if(e->FirstChildElement("side") != NULL){
        side = e->FirstChildElement("side")->GetText();
      }

      if(side == "left"){
        label.side = left;
      }else if(side == "right"){
        label.side = right;
      }else if(side == "up"){
        label.side = up;
      }else if(side == "down"){
        label.side = down;
      }else if(side == ""){
        label.side = 0.0;
      }

      cl(0) = r;
      cl(1) = g;
      cl(2) = b;

      label.type = type;
      label.color = cl;
      label.weight = wght;

      existingLabels.push_back(label);
    }
  }else{
    printer::printRed("Could not open or find labels.xml file");
    ros::shutdown();
  }
}
