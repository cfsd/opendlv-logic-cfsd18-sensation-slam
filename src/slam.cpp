/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>

#include "slam.hpp"
#include "WGS84toCartesian.hpp"

/*Initializes the slam module by initializing all relevant member variables and reading commandline parameters*/
Slam::Slam(std::map<std::string, std::string> commandlineArguments,cluon::OD4Session &a_od4) :
  od4(a_od4)
, m_optimizer()
, m_lastTimeStamp()
, m_lastCvTimeStamp()
, m_coneCollector()
, m_lastObjectId()
, m_coneMutex()
, m_sensorMutex()
, m_mapMutex()
, m_optimizerMutex()
, m_yawMutex()
, m_groundSpeedMutex()
, m_stateMachineMutex()
, m_odometryData()
, m_gpsReference()
, m_map()
, m_keyframeTimeStamp(cluon::time::now())
, m_newFrame()
, m_sendPose()
, m_sendMutex()
, m_cvCones()
{
  setupOptimizer();
  setUp(commandlineArguments);
  m_coneCollector = Eigen::MatrixXd::Zero(4,100);
  m_lastObjectId = 0;
  m_odometryData << 0,0,0;
  m_sendPose << 0,0,0;
  m_newFrame = true;
}
/*Sets up the g2o optimizer used for loop closure*/
void Slam::setupOptimizer(){

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  m_optimizer.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  //m_optimizer.setVerbose(true);
}
/*Reads the split pose and converts it to cartesian coordinates, not used since Geolocation is used instead*/
void Slam::nextSplitPose(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  if(data.dataType() == opendlv::proxy::GeodeticWgs84Reading::ID()){
    auto position = cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(std::move(data));

    double longitude = position.longitude();
    double latitude = position.latitude();

    //toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position)

    std::array<double,2> WGS84ReadingTemp;

    WGS84ReadingTemp[0] = latitude;
    WGS84ReadingTemp[1] = longitude;

    std::array<double,2> WGS84Reading = wgs84::toCartesian(m_gpsReference, WGS84ReadingTemp); 
    //opendlv::data::environment::WGS84Coordinate gpsCurrent = opendlv::data::environment::WGS84Coordinate(latitude, longitude);
    //opendlv::data::environment::Point3 gpsTransform = m_gpsReference.transform(gpsCurrent);

    m_odometryData(0) =  WGS84Reading[0];
    m_odometryData(1) =  WGS84Reading[1];
  }
  else if(data.dataType() == opendlv::proxy::GeodeticHeadingReading::ID()){
    auto message = cluon::extractMessage<opendlv::proxy::GeodeticHeadingReading>(std::move(data));
    double heading = message.northHeading();
    heading = heading-PI;
    heading = (heading > PI)?(heading-2*PI):(heading);
    heading = (heading < -PI)?(heading+2*PI):(heading);
    m_odometryData(2) = heading;
    //std::cout << "head: " << heading << std::endl;
  }
}
/*Extracts and converts the current pose to x,y,heading in local coordinates and stores it in odometryData*/
void Slam::nextPose(cluon::data::Envelope data){
    //#########################Recieve Odometry##################################
  
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  m_geolocationReceivedTime = data.sampleTimeStamp();
  auto odometry = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(data));

  double longitude = odometry.longitude();
  double latitude = odometry.latitude();

  //toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position)
  if(m_gpsCoords){

    std::array<double,2> WGS84ReadingTemp;

    WGS84ReadingTemp[0] = latitude;
    WGS84ReadingTemp[1] = longitude;

    std::array<double,2> WGS84Reading = wgs84::toCartesian(m_gpsReference, WGS84ReadingTemp); 
    //opendlv::data::environment::WGS84Coordinate gpsCurrent = opendlv::data::environment::WGS84Coordinate(latitude, longitude);
    //opendlv::data::environment::Point3 gpsTransform = m_gpsReference.transform(gpsCurrent);

    m_odometryData << WGS84Reading[0],
                      WGS84Reading[1],
                      odometry.heading();
    double heading = odometry.heading()-PI;
    heading = (heading > PI)?(heading-2*PI):(heading);
    heading = (heading < -PI)?(heading+2*PI):(heading);
    m_odometryData(2) = heading;
  }
  else{
    m_odometryData << longitude+m_xOffset,
                      latitude+m_yOffset,
                      odometry.heading()+m_headingOffset;
  }  //std::cout << "head: " << odometry.heading() << std::endl;                   
}

/*Reads the current yawrate and stores it with a timestamp*/
void Slam::nextYawRate(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockYaw(m_yawMutex);
  auto yawRate = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(data));
  m_yawRate = yawRate.angularVelocityZ();
   m_yawReceivedTime = data.sampleTimeStamp();
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}
void Slam::nextGroundSpeed(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
  auto groundSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(data));
  m_groundSpeed = groundSpeed.groundSpeed();
   m_groundSpeedReceivedTime = data.sampleTimeStamp();
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}
/*Receives the combined frame from the collector class and unpacks it into an Eigen matrix containing the current frame.
This is the method that initializes the rest of the slam class*/
void Slam::recieveCombinedMessage(cluon::data::TimeStamp currentFrameTime,std::map<int,ConePackage> currentFrame){
  m_lastTimeStamp = currentFrameTime;

  if(isKeyframe()){
    Eigen::MatrixXd cones = Eigen::MatrixXd::Zero(4,currentFrame.size());
    std::map<int,ConePackage>::iterator it;
    int coneIndex = 0;
    it =currentFrame.begin();
    while(it != currentFrame.end()){
      auto direction = std::get<0>(it->second);
      auto distance = std::get<1>(it->second);
      auto type = std::get<2>(it->second);
      double azimuth = direction.azimuthAngle();
      if(fabs(azimuth)<90){//We will only receive cones in front of the car
        cones(0,coneIndex) = azimuth;
        cones(1,coneIndex) = direction.zenithAngle();
        cones(2,coneIndex) = distance.distance();
        cones(3,coneIndex) = (type.type()<=4)?(type.type()):(0);
        coneIndex++;
      }
      it++;
    }
    //cones = cones.leftCols(coneIndex);
    
      performSLAM(cones);
    
  }
}

/*This method unpacks the frame from the CvCollector which contains cones from the detectcone module and stores it as CVcones
CVcones are used as a fallback if the localization fails after the first lap.*/
void Slam::recieveCombinedCvMessage(cluon::data::TimeStamp currentFrameTime,std::map<int,ConePackage> currentFrame){
  m_lastCvTimeStamp = currentFrameTime;
  Eigen::MatrixXd cones = Eigen::MatrixXd::Zero(4,currentFrame.size());
  std::map<int,ConePackage>::iterator it;
  int coneIndex = 0;
  it =currentFrame.begin();
  while(it != currentFrame.end()){
    auto direction = std::get<0>(it->second);
    auto distance = std::get<1>(it->second);
    auto type = std::get<2>(it->second);
    cones(0,coneIndex) = direction.azimuthAngle();
    cones(1,coneIndex) = direction.zenithAngle();
    cones(2,coneIndex) = distance.distance();
    cones(3,coneIndex) = (type.type()<=4)?(type.type()):(0);
    coneIndex++;
    it++;
  } 
  m_cvCones.setCvCones(cones);
  m_cvCones.setTimeStamp(currentFrameTime);

  //SendCvCones(m_cvCones.getCvCones());
}
/*Old method that was to be used to determine which cone frame to use as keyframe
The current policy is to use a simple timing policy to insert a keyframe every m_timeBetweenKeyframes ms*/
bool Slam::isKeyframe(){
  cluon::data::TimeStamp startTime = cluon::time::now();
  double timeElapsed = fabs(static_cast<double>(cluon::time::deltaInMicroseconds(m_keyframeTimeStamp,startTime)))/1000;
  std::cout << "Time ellapsed is: " << timeElapsed << std::endl;
  if(timeElapsed>m_timeBetweenKeyframes){//Keyframe candidate is based on time difference from last keyframe
    m_keyframeTimeStamp = startTime;
    return true;
  }
  return false;
}

/*Main method for the SLAM module, receives cones as input and processes them by either adding them to the map during the first lap,
or uses them for localization in the map for the subsequent laps. During the second lap the message methods are called from here.
The loop closing and subsequent map filtering is also initialized from this method*/
void Slam::performSLAM(Eigen::MatrixXd cones){
  
  std::lock_guard<std::mutex> lockStateMachine(m_stateMachineMutex);
  if(!m_readyStateMachine || !m_readyState) //If the module isn't initialized we shouldn't do SLAM
  {
    return;
  }
    Eigen::Vector3d pose;
  {// Read the current pose into a variable and add it to the list
    std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
    pose = m_odometryData;
    m_poses.push_back(pose);
  }
  //IF it is the first lap, add the cones to the map
  if(!m_loopClosingComplete){
    {
      std::lock_guard<std::mutex> lockMap(m_mapMutex);
      createConnections(cones,pose); //Builds the map
    }

    uint32_t currentEndCone = m_coneList.size() - 1; 
    uint32_t coneDiff = currentEndCone - m_coneRef;
    //Do an essential graph optimization every 10th cone
    if(coneDiff >= 10 && !m_loopClosingComplete){
      std::lock_guard<std::mutex> lockMap(m_mapMutex);
      optimizeEssentialGraph(currentEndCone-coneDiff, currentEndCone);
      m_coneRef = currentEndCone; 
    }
  }

  m_poseId++;

  //Check if there is enough loopclosing candidates
  if(!m_loopClosingComplete){
    if(m_currentConeDiff > m_lapSize || m_validMapIterator > 0){
      std::lock_guard<std::mutex> lockMap(m_mapMutex);
      isMapValid(pose);
      if(m_mapIsValid){
        std::cout << "Full BA ..." << std::endl;
        fullBA();

        std::cout << "Full BA Done ..." << std::endl;
      }else if(m_validMapIterator > 19){//Map was not good enough for loop closure, map is reset and the car has to go one more lap
        m_currentConeIndex = 0;
        m_poseId = 1000;
        m_optimizer.clear();
        m_poses.clear();
        m_coneList.clear();
        m_map.clear();
        m_essentialMap.clear();
        m_validMapIterator = 0;

        std::cout << "Loop Closing Uncertain, Map Reset ..." << std::endl;

      }
    }
  }
  //Map preprocessing to filter out invalid cones and assign colours
  if(m_filterMap){

    std::cout << "Filter Map ..." << std::endl;
    std::lock_guard<std::mutex> lockMap(m_mapMutex);
    std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
    filterMap();

    std::cout << "Update Map ..." << std::endl;
    updateMap(0,m_coneList.size(),true);
    m_filterMap = false;
    m_currentConeIndex = 0;
  }
  /*Localization logic after loop closure, first we attempt to match the currently seen cones to the cones in the map
  using the initial position from the UKF, after that we optimize the localization using a local g2o optimizer, then we check the
  localization for concistency and robustness and if it succeeds we send out the cones and pose to the other modules*/
  if(m_loopClosingComplete){
    std::cout << "Localizing ..." << std::endl;
    std::vector<std::pair<int,Eigen::Vector3d>> matchedCones = matchCones(cones,pose); 
    if(localizable(matchedCones)){
      std::cout << "matched " << matchedCones.size() << " cones" << std::endl;
      localizer(matchedCones,pose);
      if(checkLocalization()){
        sendPose();
        sendCones();
        return;
      }
    }
    //If the localization fails we send out a local frame to make sure that it is consistent
    m_sendPose = m_odometryData;
    //sendCones();
    std::vector<Cone> conesCv = m_cvCones.getCvCones(); 
    SendCvCones(conesCv, conesCv.size(),m_lastCvTimeStamp );
  }
}

/*This method attempts to match cones in the local frame to the global map, It returns a vector of pairs with a matched ID 
and the global observation. The method initially uses the provided pose to search for a global matcha and if the average error of the cones
is less than 1 meter the match is accepted. If the average error is greater than 1 metre the optimal heading is searched for using a loop.
The optimal heading is used to find the best match in the map*/
std::vector<std::pair<int,Eigen::Vector3d>> Slam::matchCones(Eigen::MatrixXd cones,Eigen::Vector3d &pose){
  //Find surrounding cone indexes of 20 meters
  //double initPose = pose(2);
  std::vector<uint32_t> inMapIndex;
  for(uint32_t i = 0; i < m_map.size(); i++){//The relevant cones in the map are extracted
    opendlv::logic::perception::ObjectDistance objDistance = m_map[i].getDistance(pose);
    if(objDistance.distance() < 20){
      inMapIndex.push_back(i);
    }
  }
  uint32_t conesThatFit = 0;
  std::pair<double,std::vector<uint32_t>> scoredMatch = evaluatePose(cones,pose,inMapIndex,conesThatFit); //The initial pose is scored to find a match
  if(std::get<0>(scoredMatch)/cones.cols()<1){
    return filterMatch(cones,pose,scoredMatch);
  }
  double angle = pose(2)-PI/2;
  double angleMax = pose(2)+PI/2;
  double degrees = 2;
  double angleStep = 0.01745*degrees;
  std::vector<std::pair<double,std::vector<uint32_t>>> matchVector;
  uint32_t lastConeFitter = 0;
  double bestHeading = 0;
  double bestSumError = 100000;
  uint32_t minDistIdx = 0;
  int counter = 0;
  //A sweep over the angle is performed, the angle with the most cones matched and the best score is selected as the optimal angle
  for(double k = angle; k < angleMax; k = k + angleStep){
    pose(2) = k;
    conesThatFit = 0;
    bool foundFullMatch = false;
    scoredMatch = evaluatePose(cones,pose,inMapIndex,conesThatFit);
    matchVector.push_back(scoredMatch);
    double sumOfAllErrors = std::get<0>(scoredMatch);
    bool betterSum = false;
    if(sumOfAllErrors < bestSumError){
      betterSum = true;
    }
    //std::cout << "Fitted Cones: " << conesThatFits << std::endl;
    //First we attempt to find angle that can match all cones
    if(conesThatFit == cones.cols() && !foundFullMatch){
      std::cout << "new Best Heading: " << k << std::endl;
      std::cout << "best Error: " << sumOfAllErrors << std::endl;
      bestHeading = k;
      lastConeFitter = conesThatFit;
      bestSumError = sumOfAllErrors;
      minDistIdx = counter;
      foundFullMatch = true;
    //Then we look at the score for the heading
    }else if(conesThatFit > 1 && conesThatFit >= lastConeFitter && betterSum){
      std::cout << "new Best Heading: " << k << std::endl;
      std::cout << "best Error: " << sumOfAllErrors << std::endl;
      bestHeading = k;
      minDistIdx = counter;
      lastConeFitter = conesThatFit;
      bestSumError = sumOfAllErrors;
    }
    counter++;                  
  }
  bestHeading = (bestHeading > PI)?(bestHeading-2*PI):(bestHeading);
  bestHeading = (bestHeading < -PI)?(bestHeading+2*PI):(bestHeading);
  pose(2) = bestHeading;
  std::vector<std::pair<int,Eigen::Vector3d>> matchedCones = filterMatch(cones,pose,matchVector[minDistIdx]);
  return matchedCones;
}
/*A score and the best match to the global map is calculated for a given pose, the score and the match to the global map is returned.
The score is formulated as the sum of all positional errors when comparing the local observations to the global map*/
std::pair<double,std::vector<uint32_t>> Slam::evaluatePose(Eigen::MatrixXd cones, Eigen::Vector3d pose, std::vector<uint32_t> inMapIndex, uint32_t &fitCones){
  double sumOfAllErrors = 0;
  std::vector<double> coneErrors;
  std::vector<uint32_t> matchedCone;
  for(uint32_t i = 0; i < cones.cols(); i++){
    Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(i));
    double minimumError = 100000;
    uint32_t minIndex = 100;
    for(uint32_t j = 0; j < inMapIndex.size(); j++){
      double errorDistance = std::sqrt( (globalCone(0)-m_map[inMapIndex[j]].getOptX())*(globalCone(0)-m_map[inMapIndex[j]].getOptX()) + (globalCone(1)-m_map[inMapIndex[j]].getOptY())*(globalCone(1)-m_map[inMapIndex[j]].getOptY()) );
      if(errorDistance < minimumError){
        minIndex = inMapIndex[j];
        minimumError = errorDistance;
      }
    }//Map
    sumOfAllErrors += minimumError;
    coneErrors.push_back(minimumError);        
    matchedCone.push_back(minIndex);
  }//Local Frame  
  fitCones = 0;
  for(uint32_t l = 0; l < coneErrors.size(); l++){
    if(coneErrors[l] < 0.5){
      fitCones++;
    }
  }
  std::pair<double,std::vector<uint32_t>> scoredMatches = std::make_pair(sumOfAllErrors,matchedCone);
  return scoredMatches;
}
/*Checks the best matched cones and compares them to a distance threshold to determine if the match is good enough.
The returned match contains the ID of the matched cone and the current observation it was matched to*/
std::vector<std::pair<int,Eigen::Vector3d>> Slam::filterMatch(Eigen::MatrixXd cones, Eigen::Vector3d pose,std::pair<double,std::vector<uint32_t>> matchedCones){
  std::vector<uint32_t> matchedIndices = std::get<1>(matchedCones);
  std::vector<std::pair<int,Eigen::Vector3d>> matchedConeVector;
  //double errorSum = std::get<0>(matchedCones);
  /*
  if(cones.cols() == 2 && errorSum/cones.cols() > 0.3){
    return matchedConeVector;
  }
  if(cones.cols() == 3 && errorSum/cones.cols() > 0.4){
    return matchedConeVector;
  }
  if(cones.cols() > 3 && errorSum/cones.cols() > 0.5){
    return matchedConeVector;
  }*/
  for(int i = 0; i<cones.cols();i++){
    Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(i));
    Eigen::Vector3d localCone = Spherical2Cartesian(cones(0,i),cones(1,i),cones(2,i));
    double distance = std::sqrt( (globalCone(0)-m_map[matchedIndices[i]].getOptX())*(globalCone(0)-m_map[matchedIndices[i]].getOptX()) + (globalCone(1)-m_map[matchedIndices[i]].getOptY())*(globalCone(1)-m_map[matchedIndices[i]].getOptY()) );
    if(distance<m_newConeThreshold){
      std::pair<int,Eigen::Vector3d> match = std::make_pair(matchedIndices[i],localCone);
      matchedConeVector.push_back(match);
    }
  }
  m_currentConeIndex = updateCurrentCone(pose,m_currentConeIndex,m_map.size()-1);
  return matchedConeVector;
}
/*Checks for a new current cone, the current cone is defined as the closest cone to the car.
It is used to determine which cones to send to other modules after a Loop closure*/
int Slam::updateCurrentCone(Eigen::Vector3d pose,uint32_t currentConeIndex, uint32_t remainingIter){
  /*currentConeIndex=(currentConeIndex<m_map.size())?(currentConeIndex):(currentConeIndex-m_map.size());
  Cone currentCone = m_map[remainingIter];
  remainingIter = remainingIter-1;
  auto distance = currentCone.getDistance(pose);
  auto direction = currentCone.getDirection(pose);
  if(remainingIter == 0){
    //currentConeIndex += 1;
    if(currentConeIndex > m_map.size()){
      currentConeIndex = currentConeIndex - m_map.size();
    }
    return currentConeIndex;
  }
  if(distance.distance() < 5.0f && fabs(direction.azimuthAngle())>80.0f){
    currentConeIndex = updateCurrentCone(pose,currentConeIndex,remainingIter);
  }
  return remainingIter;*/
  remainingIter = remainingIter;
  opendlv::logic::perception::ObjectDistance lastDistance;
  lastDistance.distance(1000);
  for(uint32_t i = 0; i < m_map.size(); i++){
    Cone currentCone = m_map[i];
    auto distance = currentCone.getDistance(pose);
    auto direction = currentCone.getDirection(pose);
    if(distance.distance() < 3.0f){
      if(distance.distance() < lastDistance.distance()){
        lastDistance.distance(distance.distance());
        currentConeIndex = i;
      }
    } 
  }

  return currentConeIndex;
}
/*Method for checking if the cones detected and matched are good enough to use for localization,
This method can be improved*/
bool Slam::localizable(std::vector<std::pair<int,Eigen::Vector3d>> matchedCones){
  return matchedCones.size()>1;
}

/*Performs a local optimization to optimize the current pose and localize the car. Inspired by the tracking method in ORB-SLAM.
This method is currently only used after loop closure but an improvement would be to use it in the first lap in some way.
By doing localization in the first lap the map buildnign should be improved*/
void Slam::localizer(std::vector<std::pair<int,Eigen::Vector3d>> matchedCones, Eigen::Vector3d pose){
  g2o::SparseOptimizer localGraph;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  localGraph.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  //localGraph.setVerbose(true);

  std::lock_guard<std::mutex> lockMap(m_mapMutex);
  //Create a local graph
  bool performedLocalization = false;
  if(matchedCones.size() > 1 ){  
    //Create graph
    //Add pose vertex
    g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
    poseVertex->setId(1000);
    poseVertex->setEstimate(pose);
    localGraph.addVertex(poseVertex);
    //Add cone vertex
    Eigen::Vector2d coneMeanXY;
    for(uint32_t i = 0; i < matchedCones.size(); i++){
      Eigen::Vector3d localObs = std::get<1>(matchedCones[i]);
      int index = std::get<0>(matchedCones[i]);
      g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;
      coneMeanXY << m_map[index].getOptX(),m_map[index].getOptY();
      g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
      coneVertex->setId(i);
      coneVertex->setEstimate(coneMeanXY);
      coneVertex->setFixed(true);      
      localGraph.addVertex(coneVertex);

      //Add edge between pose and cone i
      Eigen::Vector2d xyMeasurement;
      coneMeasurement->vertices()[0] = localGraph.vertex(1000);
      coneMeasurement->vertices()[1] = localGraph.vertex(i);
      xyMeasurement << localObs(0),localObs(1);
      coneMeasurement->setMeasurement(xyMeasurement);

      Eigen::Matrix2d informationMatrix;
      informationMatrix << 1/0.1,0,
                              0,1/0.1;
      coneMeasurement->setInformation(informationMatrix);
      localGraph.addEdge(coneMeasurement);
      performedLocalization = true;
    }
  }
  //Perform the localization and optimize the pose
  if(performedLocalization){
    localGraph.initializeOptimization();
    localGraph.optimize(10); 
    g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(localGraph.vertex(1000));
    g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
    Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
    std::lock_guard<std::mutex> lockSend(m_sendMutex);
    m_sendPose << updatedPose(0),updatedPose(1),updatedPose(2);
  }else{//localization failed, use the initial pose instead
    std::lock_guard<std::mutex> lockSend(m_sendMutex);
    m_sendPose << pose(0),pose(1),pose(2);
  } 
}
/*Checks the localization and determines if it is good enough, the offset is used to attempt to minimize any drift by the UKF and minimize the error,
Preferably the offset shouldn't be necessary but it was an emergency measure to improve robustness in the localization*/
bool Slam::checkLocalization(){
  double xOffset = m_sendPose(0) - m_odometryData(0);
  double yOffset = m_sendPose(1) - m_odometryData(1);
  double headingOffset = m_sendPose(2) - m_odometryData(2);
  std::cout << "xoffset: " << xOffset << "yoffset: " << yOffset << "headingoffset: " << headingOffset << std::endl;
  bool goodOffset = fabs(xOffset)<m_offsetDistance && fabs(yOffset)<m_offsetDistance && fabs(headingOffset)<m_offsetHeading;
  if(goodOffset){
    m_xOffset = xOffset+m_xOffset;
    m_yOffset = yOffset+m_yOffset;
    m_headingOffset = headingOffset+m_headingOffset;
  }
  return goodOffset;
}

/*This creates the map during the first lap, it first tries to match the locally observed cones to the global map
and then adds any unmatched cones to the map, the observations of matched cones are also added to the graph via the cone class*/
void Slam::createConnections(Eigen::MatrixXd cones, Eigen::Vector3d pose){
  int currentConeIndex = m_currentConeIndex;
  //Check current cone list and find same cone, if true add pose connection and cone observation
  bool firstCone = false;
  if(m_coneList.size() == 0){//first cone to map
    Eigen::Vector3d localCone = Spherical2Cartesian(cones(0,0), cones(1,0),cones(2,0));
    Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(0));
    Cone cone = Cone(globalCone(0),globalCone(1),(int)globalCone(2),m_coneList.size()); //Temp id, think of system later
    cone.addObservation(localCone,globalCone,m_poseId,m_currentConeIndex);

    m_coneList.push_back(cone);
    firstCone = true;
  }

  double minDistance = 100;
  for(uint32_t i = 0; i<cones.cols(); i++){//Iterate through local cone objects
    if(fabs(cones(0,i))<0.00001 || fabs(cones(1,i))<0.00001 || fabs(cones(2,i))<0.00001) //Error check
    {
      continue;
    }
    double distanceToCar = cones(2,i);
    Eigen::Vector3d localCone = Spherical2Cartesian(cones(0,i), cones(1,i),cones(2,i));
    Eigen::Vector3d globalCone = coneToGlobal(pose, cones.col(i)); //Make local cone into global coordinate frame
    uint32_t j = 0;
    bool coneFound = false;
    //Loops through the map to match the cones
    while(!coneFound && j<m_coneList.size() && !m_loopClosing && !firstCone){
      if(fabs(m_coneList[j].getType() - cones(3,i))<0.0001 && !std::isnan(globalCone(0)) && !std::isnan(globalCone(1))){ //Check is same classification

        Cone globalConeObject = Cone(globalCone(0), globalCone(1),0,2000);
        double distance = distanceBetweenCones(m_coneList[j],globalConeObject);

        if(distance<m_newConeThreshold){ //NewConeThreshold is the accepted distance for a new cone candidate
          coneFound = true;
          m_coneList[j].addObservation(localCone, globalCone, m_poseId,m_currentConeIndex);
          
          if(distanceToCar<minDistance && distanceToCar<m_coneMappingThreshold){//Update current cone to know where in the map we are
            currentConeIndex = j;
            minDistance = distanceToCar;
          }
        }
      }
      j++;
    }
    //Adds the current cone to map because no match was found so it was a new cone
    if(distanceToCar < m_coneMappingThreshold && !coneFound && !m_loopClosing && !firstCone){
      //std::cout << "Trying to add cone" << std::endl;
      if(!std::isnan(globalCone(0)) && !std::isnan(globalCone(1))){
        Cone cone = Cone(globalCone(0),globalCone(1),(int)globalCone(2),m_coneList.size()); //Temp id, think of system later
        cone.addObservation(localCone, globalCone,m_poseId,m_currentConeIndex);
        m_coneList.push_back(cone);  
      }
    }
  }
  m_currentConeDiff = m_currentConeIndex - currentConeIndex;
  m_currentConeIndex = currentConeIndex;
}

//Creates a graph, used for loop closing
void Slam::createFullGraph(){
  //Add all poses to graph
  addPosesToGraph();
  //For each cone in conelist add poses

  addConesToGraph();
}
/*Creates and optimizes an Essential graph, an essential graph is a subset of the map consisting of ten cones and their poses,
It is done to improve map concistency by optimizing the map in small bunches*/
void Slam::optimizeEssentialGraph(uint32_t graphIndexStart, uint32_t graphIndexEnd){

  //Initialize graph
  g2o::SparseOptimizer essentialGraph;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > slamBlockSolver;
  typedef g2o::LinearSolverEigen<slamBlockSolver::PoseMatrixType> slamLinearSolver;
  
  auto linearSolver = g2o::make_unique<slamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  
  g2o::OptimizationAlgorithmGaussNewton* algorithmType = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<slamBlockSolver>(std::move(linearSolver)));
  essentialGraph.setAlgorithm(algorithmType); //Set optimizing method to Gauss Newton
  //essentialGraph.setVerbose(true);

  std::vector<int> posesToGraph;
  //Find cones of conespan and extract poses
  for(uint32_t i = graphIndexStart; i < graphIndexEnd+1; i++){

    std::vector<int> currentConnectedPoses = m_coneList[i].getConnectedPoses();

    for(uint32_t j = 0; j < currentConnectedPoses.size(); j++){

      posesToGraph.push_back(currentConnectedPoses[j]);
    }

  }
  if(posesToGraph.size() > 0){

    uint32_t max = *std::max_element(posesToGraph.begin(), posesToGraph.end());
    uint32_t min = *std::min_element(posesToGraph.begin(), posesToGraph.end());

    //add poses to graph based on min and max
    for(uint32_t k = min; k < max+1; k++){

      //Add vertex
      g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
      poseVertex->setId(k);
      poseVertex->setEstimate(m_poses[k-1000]);

      essentialGraph.addVertex(poseVertex);

      //Add edge
      if(k > min){
        g2o::EdgeSE2* odometryEdge = new g2o::EdgeSE2;
        odometryEdge->vertices()[0] = essentialGraph.vertex(k-1);
        odometryEdge->vertices()[1] = essentialGraph.vertex(k);
        g2o::VertexSE2* prevVertex = static_cast<g2o::VertexSE2*>(essentialGraph.vertex(k-1));
        g2o::SE2 prevPose = prevVertex->estimate();
        g2o::SE2 currentPose = g2o::SE2(m_poses[k-1000](0), m_poses[k-1000](1), m_poses[k-1000](2));
        g2o::SE2 measurement = prevPose.inverse()*currentPose;
        odometryEdge->setMeasurement(measurement);
        odometryEdge->setInformation(Eigen::Matrix3d::Identity()*1/0.5); //Actual covariance should be configured
        essentialGraph.addEdge(odometryEdge);
      }
    }

    //Connect cones to poses
    for(uint32_t i = graphIndexStart; i < graphIndexEnd; i++){

        Eigen::Vector2d coneMeanXY;
        if(!m_coneList[i].isOptimized()){
          m_coneList[i].calculateMean();
          coneMeanXY << m_coneList[i].getMeanX(),m_coneList[i].getMeanY();
        }else{
          coneMeanXY << m_coneList[i].getOptX(),m_coneList[i].getOptY();
        }
      
        g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
        coneVertex->setId(m_coneList[i].getId());
        coneVertex->setEstimate(coneMeanXY);
        essentialGraph.addVertex(coneVertex);

        //Connect cones to POses
        g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;
        std::vector<int> connectedPoses = m_coneList[i].getConnectedPoses();

      for(uint32_t j = 0; j < connectedPoses.size(); j++){
          Eigen::Vector2d xyMeasurement;
          xyMeasurement = getConeToPoseMeasurement(i,j);
          //std::cout << "x: " << xyMeasurement(0) << " y: " << xyMeasurement(1) << std::endl; 
          coneMeasurement->vertices()[0] = essentialGraph.vertex(connectedPoses[j]);
          coneMeasurement->vertices()[1] = essentialGraph.vertex(m_coneList[i].getId());
          coneMeasurement->setMeasurement(xyMeasurement);

          Eigen::Vector2d covXY = m_coneList[i].getCovariance();
          Eigen::Matrix2d informationMatrix;
          informationMatrix << 1/covXY(0),0,
                              0,1/covXY(1);
          coneMeasurement->setInformation(informationMatrix); //Placeholder value
          //std::cout << "cX: " << covXY(0) << " cY: " << covXY(1) << std::endl;
          essentialGraph.addEdge(coneMeasurement);

        }
      }


    g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(essentialGraph.vertex(min));
    firstRobotPose->setFixed(true);

    /*g2o::VertexPointXY* firstCone = dynamic_cast<g2o::VertexPointXY*>(essentialGraph.vertex(graphIndexStart));
    firstCone->setFixed(true);*/
    //std::cout << "Optimizing" << std::endl;
    essentialGraph.initializeOptimization();
    essentialGraph.optimize(10); //Add config for amount of iterations??
    //std::cout << "Optimizing done." << std::endl;

    updateFromEssential(min, max, graphIndexStart,graphIndexEnd, essentialGraph);
    updateMap(graphIndexStart,graphIndexEnd,false);
  }
}
/*Updates the list of cones and the pose vector after the essential graph optimization to provide optimized pose and cone positions*/
void Slam::updateFromEssential(uint32_t poseStart, uint32_t poseEnd,uint32_t coneStart,uint32_t coneEnd, g2o::SparseOptimizer &essentialGraph){

  //Update pose vector

  for(uint32_t i = poseStart; i < poseEnd; i++){
    g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(essentialGraph.vertex(i));
    g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
    Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
    m_poses[i-1000] << updatedPose(0),updatedPose(1),updatedPose(2);
  }
  //Set optimized cone positions

   Eigen::Vector2d updatedConeXY;
   g2o::VertexPointXY* updatedConeVertex;

   for(uint32_t i = coneStart; i < coneEnd; i++){
    updatedConeVertex = static_cast<g2o::VertexPointXY*>(essentialGraph.vertex(i));
    updatedConeXY = updatedConeVertex->estimate();
    m_coneList[i].setOptX(updatedConeXY(0));
    m_coneList[i].setOptY(updatedConeXY(1));
    m_coneList[i].setOptimized();    
   }
}
/*NOT USED, extracts the latest pose*/
Eigen::Vector3d Slam::updatePoseFromGraph(){

  g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(m_optimizer.vertex(m_poseId-1));
  g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
  Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
  return updatedPose;
}
/*Adds the poses to the graph used for loop closing*/
void Slam::addPosesToGraph(){

  for(uint32_t i = 0; i < m_poses.size(); i++){
    g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
    poseVertex->setId(i+1000);
    poseVertex->setEstimate(m_poses[i]);

    m_optimizer.addVertex(poseVertex);

    addOdometryMeasurement(m_poses[i],i);

    std::vector<int> poseVector;
    m_connectivityGraph.push_back(poseVector);
  }
}

/*Adds an odometry measurement between each pose, at the moment the odomerty measurement is just the difference between the poses.
However to improve the SLAM the odometry measurement can use the yaw and groundspeed from the IMU instead*/
void Slam::addOdometryMeasurement(Eigen::Vector3d pose, uint32_t i){
  if(i > 0){
    g2o::EdgeSE2* odometryEdge = new g2o::EdgeSE2;

    odometryEdge->vertices()[0] = m_optimizer.vertex(i+999);
    odometryEdge->vertices()[1] = m_optimizer.vertex(i+1000);
    g2o::VertexSE2* prevVertex = static_cast<g2o::VertexSE2*>(m_optimizer.vertex(i+999));
    g2o::SE2 prevPose = prevVertex->estimate();
    g2o::SE2 currentPose = g2o::SE2(pose(0), pose(1), pose(2));
    g2o::SE2 measurement = prevPose.inverse()*currentPose;
    odometryEdge->setMeasurement(measurement);
    odometryEdge->setInformation(Eigen::Matrix3d::Identity()*1/0.5); //Actual covariance should be configured
    m_optimizer.addEdge(odometryEdge);
  }
}
/*This performs the full optimization of the map, referred to as a loop closure. After the loop closure all the cone and pose
positions are updated according to the optimized result. This is one of the key methods in the SLAM module*/
void Slam::fullBA(){


  createFullGraph();

  g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(m_optimizer.vertex(1000));
  firstRobotPose->setFixed(true);

  /*g2o::VertexSE2* secondRobotPose = dynamic_cast<g2o::VertexSE2*>(m_optimizer.vertex(1001));
  secondRobotPose->setFixed(true);*/

  /*g2o::VertexPointXY* firstCone = dynamic_cast<g2o::VertexPointXY*>(m_optimizer.vertex(m_currentConeIndex));
  firstCone->setFixed(true);*/

  /*g2o::VertexPointXY* secondCone = dynamic_cast<g2o::VertexPointXY*>(m_optimizer.vertex(1));
  secondCone->setFixed(true);*/


  //m_optimizer.setVerbose(true);
  //std::cout << "Optimizing" << std::endl;
  //This is an internal check in g2o to ensure proper optimization
  if(m_optimizer.verifyInformationMatrices()){
    m_optimizer.initializeOptimization();
    m_optimizer.optimize(10);
    std::cout << "Optimization Done ..." << std::endl;
    m_loopClosingComplete = true;
    m_filterMap = true;
  }else{
    m_poseId = 1000;
    m_currentConeIndex = 0;  
    m_optimizer.clear();
    m_map.clear();
    m_essentialMap.clear();
    m_coneList.clear();
    m_poses.clear();
    std::cout << "Optimization not feasable, rebuilding graph ..." << std::endl;
  }
  
  //Add config for amount of iterations??
  //std::cout << "Optimizing done." << std::endl;


  Eigen::Vector2d updatedConeXY;
  g2o::VertexPointXY* updatedConeVertex;

  for(uint32_t j = 0; j < m_coneList.size(); j++){//Iterate and replace old map landmarks with new updated ones
    updatedConeVertex = static_cast<g2o::VertexPointXY*>(m_optimizer.vertex(j));
    updatedConeXY = updatedConeVertex->estimate();
    m_coneList[j].setOptX(updatedConeXY(0));
    m_coneList[j].setOptY(updatedConeXY(1));
  }
  
  for(uint32_t i = 0; i < m_poses.size(); i++){//Replace the poses as well
    g2o::VertexSE2* updatedPoseVertex = static_cast<g2o::VertexSE2*>(m_optimizer.vertex(i+1000));
    g2o::SE2 updatedPoseSE2 = updatedPoseVertex->estimate();
    Eigen::Vector3d updatedPose = updatedPoseSE2.toVector();
    m_poses[i] << updatedPose(0),updatedPose(1),updatedPose(2);
  }
  //updateMap(0,m_coneList.size(),true);

}


/*Conversion method to convert a cone from the local car coordinate system to the global coordinate system*/
Eigen::Vector3d Slam::coneToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd cones){
  Eigen::Vector3d cone = Spherical2Cartesian(cones(0), cones(1), cones(2));
  //convert from local to global coordsystem
  double newX = cone(0)*cos(pose(2))-cone(1)*sin(pose(2));
  double newY = cone(0)*sin(pose(2))+cone(1)*cos(pose(2));
  cone(0) = newX+pose(0);
  cone(1) = newY+pose(1);
  cone(2) = cones(3);
  return cone;
}
/*Transforms the cone coordinates from the Lidar frame to the COG frame using trigonometry (cosine theorem and such)
The angle and distance is transformed since the angle and distance to a cone differs from the Lidar and COG perspective*/
Eigen::Vector2d Slam::transformConeToCoG(double angle, double distance){
  const double lidarDistToCoG = 1.5;
  double sign = angle/std::fabs(angle);
  angle = PI - std::fabs(angle*DEG2RAD); 
  double distanceNew = std::sqrt(lidarDistToCoG*lidarDistToCoG + distance*distance - 2*lidarDistToCoG*distance*std::cos(angle));
  double angleNew = std::asin((std::sin(angle)*distance)/distanceNew )*RAD2DEG; 
  Eigen::Vector2d transformed;
  transformed << angleNew*sign,distanceNew;

  return transformed;
}
/*This adds cones to the global graph used for loop closure, it is called when the loop is closed*/
void Slam::addConesToGraph(){

  for(uint32_t i = 0; i < m_coneList.size(); i++){
    Eigen::Vector2d coneMeanXY;
    if(!m_coneList[i].isOptimized()){
      m_coneList[i].calculateMean();
      coneMeanXY << m_coneList[i].getMeanX(),m_coneList[i].getMeanY();
    }else{
      coneMeanXY << m_coneList[i].getOptX(),m_coneList[i].getOptY();
    }
    g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY;
    coneVertex->setId(m_coneList[i].getId());
    coneVertex->setEstimate(coneMeanXY);
    m_optimizer.addVertex(coneVertex);
    addConeMeasurements(i);
    }
}
/*Add the observations of cones from poses (the edges) to the global map. Used to build the global graph for loop closure*/
void Slam::addConeMeasurements(int i){

  g2o::EdgeSE2PointXY* coneMeasurement = new g2o::EdgeSE2PointXY;
  std::vector<int> connectedPoses = m_coneList[i].getConnectedPoses();

  for(uint32_t j = 0; j < connectedPoses.size(); j++){
  Eigen::Vector2d xyMeasurement;
  xyMeasurement = getConeToPoseMeasurement(i,j);
  coneMeasurement->vertices()[0] = m_optimizer.vertex(connectedPoses[j]);
  coneMeasurement->vertices()[1] = m_optimizer.vertex(m_coneList[i].getId());
  coneMeasurement->setMeasurement(xyMeasurement);

  Eigen::Vector2d covXY = m_coneList[i].getCovariance();
  Eigen::Matrix2d informationMatrix;
  informationMatrix << 1/covXY(0),0,
                       0,1/covXY(1);
  coneMeasurement->setInformation(informationMatrix); //Placeholder value
  
  m_optimizer.addEdge(coneMeasurement);
  }
  //m_connectivityGraph[m_poseId-1001].push_back(cone.getId());
}
/*Gets the measurements between and individual pose (id j) and cone (id i) to use to construct the edge of the graph*/
Eigen::Vector2d Slam::getConeToPoseMeasurement(int i,int j){
  Eigen::Vector2d cone;
  cone = m_coneList[i].getLocalConeObservation(j); 
  Eigen::Vector2d measurement;
  measurement << cone(0), cone(1);  
  return measurement;
}
/*Gets the local measurement between a cone and a pose, NOT USED since the method was moved to the cone class*/
Eigen::Vector2d Slam::getLocalConeToPoseMeasurement(Eigen::Vector3d pose, Eigen::Vector2d cone){
  
  Eigen::Vector2d measurement;
  measurement << cone(0)-pose(0), cone(1)-pose(1);  
  return measurement;
}
/*Converts the cone coordinates from spherical (OpenDLV message) in the local (Lidar) frame to cartesian coordinates in the COG frame*/
Eigen::Vector3d Slam::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  //azimuth = (azimuth > PI)?(azimuth-2*PI):(azimuth);
  //azimuth = (azimuth < -PI)?(azimuth+2*PI):(azimuth);
  Eigen::Vector2d transformedCone = transformConeToCoG(azimuth,distance); //Transforms cone coordinates from Lidar to COG frame
  azimuth = transformedCone(0);
  distance = transformedCone(1);
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Eigen::Vector3d::Zero();
  recievedPoint << xData,
                   yData,
                   zData;
  return recievedPoint;
}
/*Conversion to cartesian without conversion to the COG frame*/
Eigen::Vector3d Slam::Spherical2CartesianNoCoG(double azimuth, double zenimuth, double distance)
{

  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Eigen::Vector3d::Zero();
  recievedPoint << xData,
                   yData,
                   zData;
  return recievedPoint;
}
/*Conversion to spherical coordinates from cartesian to send cones from the SLAM module*/
Eigen::Vector3d Slam::Cartesian2Spherical(double x, double y, double z)
{
  double distance = sqrt(x*x+y*y+z*z);
  double azimuthAngle = atan2(y,x)*static_cast<double>(RAD2DEG);
  double zenithAngle = 0;
  
  Eigen::Vector3d transformedPoints;

  transformedPoints << azimuthAngle,zenithAngle,distance;

  return transformedPoints;

}
/*Sends the relevant cones from the global map, converts first to a local frame and then to the message types before sending
via the od4session. The code here matches the local cvCones with the cones from the map to ensure consistency. This is a messy
method and should be redone*/
void Slam::sendCones()
{
  Eigen::Vector3d pose;
  {
    std::lock_guard<std::mutex> lockSend(m_sendMutex); 
    pose = m_sendPose;
  }//mapmutex too
  std::lock_guard<std::mutex> lockMap(m_mapMutex);
  //std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  std::vector<Cone> localFrame = m_cvCones.getCvCones();

  cluon::data::TimeStamp sampleTime = m_lastTimeStamp;
  //SendCvCones(localFrame,m_conesPerPacket,sampleTime);
  std::vector<uint32_t> globalIndex;
  
  for(uint32_t i = 0; i< m_conesPerPacket;i++){
    bool isMatch = false; //Iterate through the cones ahead of time the path planning recieves
    int index = (m_currentConeIndex+i<m_map.size())?(m_currentConeIndex+i):(m_currentConeIndex+i-m_map.size()); //Check if more cones is sent than there exists
    opendlv::logic::perception::ObjectDirection directionMsg = m_map[index].getDirection(pose); //Extract cone direction
    opendlv::logic::perception::ObjectDistance distanceMsg = m_map[index].getDistance(pose); //Extract cone distance

    Eigen::Vector3d currentGlobalCone = Spherical2CartesianNoCoG(static_cast<double>(directionMsg.azimuthAngle()),0.0,static_cast<double>(distanceMsg.distance()));
    Cone currentMapCone = Cone(currentGlobalCone(0),currentGlobalCone(1),0,0);
    for(uint32_t j = 0; j < localFrame.size(); j++){//Checks the global map for matches to the local frmae
      //ToGlobal
        if(distanceBetweenCones(currentMapCone,localFrame[j]) < 2 && localFrame[j].getType() != 0 ){
          isMatch = true;
          
        }
    }    

    if(!isMatch){//Cones that weren't in the local frame are added to the global indices
      globalIndex.push_back(index);
    }
  }

  uint32_t packetSize = globalIndex.size() + localFrame.size();
  int i2 = 0;
  for(uint32_t i = 0; i < packetSize; i++){

          uint32_t index = packetSize-1-i;
        if(i < localFrame.size()){//IF it is a local frame

          Eigen::Vector3d sphericalPoints = Cartesian2Spherical(localFrame[i].getX(),localFrame[i].getY(),0);
          opendlv::logic::perception::ObjectDirection coneDirection;
          coneDirection.objectId(index);
          coneDirection.azimuthAngle(static_cast<float>(sphericalPoints(0)));  //Negative to convert to car frame from LIDAR
          coneDirection.zenithAngle(static_cast<float>(sphericalPoints(1)));
          od4.send(coneDirection,sampleTime,m_senderStamp);

          opendlv::logic::perception::ObjectDistance coneDistance;
          coneDistance.objectId(index);
          coneDistance.distance(static_cast<float>(sphericalPoints(2)));
          od4.send(coneDistance,sampleTime,m_senderStamp);

          opendlv::logic::perception::ObjectType coneType;
          coneType.objectId(index);
          coneType.type(localFrame[i].getType());
          od4.send(coneType,sampleTime,m_senderStamp);

        }else{ //If it is from the global frame

          int i3 = globalIndex[i2];
          opendlv::logic::perception::ObjectDirection directionMsg = m_map[i3].getDirection(pose); //Extract cone direction
          opendlv::logic::perception::ObjectDistance distanceMsg = m_map[i3].getDistance(pose); //Extract cone distance
          directionMsg.objectId(index);
          od4.send(directionMsg,sampleTime,m_senderStamp);
          distanceMsg.objectId(index);
          od4.send(distanceMsg,sampleTime,m_senderStamp);
          opendlv::logic::perception::ObjectType typeMsg;
          typeMsg.type(m_map[i3].getType()); //Extract cone type
          typeMsg.objectId(index);
          od4.send(typeMsg,sampleTime,m_senderStamp);
          i2++;

        }
  }      
}
/*Sends the optimized pose from the localization, can be used as feedback to state estimation*/
void Slam::sendPose(){
  opendlv::logic::sensation::Geolocation poseMessage;
  std::lock_guard<std::mutex> lockSend(m_sendMutex); 
  poseMessage.longitude(m_sendPose(0)-m_xOffset);
  poseMessage.latitude(m_sendPose(1)-m_yOffset);
  poseMessage.heading(static_cast<float>(m_sendPose(2)-m_headingOffset));
  cluon::data::TimeStamp sampleTime = m_geolocationReceivedTime;
  od4.send(poseMessage, sampleTime ,m_senderStamp);
}

/* If the localization fails we just feed the cones from detectcone to the path planner*/
void Slam::SendCvCones(std::vector<Cone> cones,uint32_t conesToSend,cluon::data::TimeStamp sampleTimeIn){
  std::cout << "Sending out cvCones .." << std::endl;
  cluon::data::TimeStamp sampleTime = sampleTimeIn;
  for(uint32_t i = 0; i < conesToSend; i++){
    uint32_t index = conesToSend-1-i;
    Eigen::Vector3d sphericalPoints = Cartesian2Spherical(cones[index].getX(),cones[index].getY(),0);
    opendlv::logic::perception::ObjectDirection coneDirection;
    coneDirection.objectId(index);
    coneDirection.azimuthAngle(static_cast<float>(sphericalPoints(0)));  //Negative to convert to car frame from LIDAR
    coneDirection.zenithAngle(static_cast<float>(sphericalPoints(1)));
    od4.send(coneDirection,sampleTime,m_senderStamp);

    opendlv::logic::perception::ObjectDistance coneDistance;
    coneDistance.objectId(index);
    coneDistance.distance(static_cast<float>(sphericalPoints(2)));
    od4.send(coneDistance,sampleTime,m_senderStamp);

    opendlv::logic::perception::ObjectType coneType;
    coneType.objectId(index);
    coneType.type(cones[index].getType());
    od4.send(coneType,sampleTime,m_senderStamp);
  }
}
/*Calculates distance between two cones to determine if they are the same cones*/
double Slam::distanceBetweenCones(Cone c1, Cone c2){
  c1.calculateMean();
  c2.calculateMean();
  double distance = std::sqrt( (c1.getMeanX()-c2.getMeanX())*(c1.getMeanX()-c2.getMeanX()) + (c1.getMeanY()-c2.getMeanY())*(c1.getMeanY()-c2.getMeanY()) );
  return distance;
}
/*Comparison with optimized cones instead of mean observation*/
double Slam::distanceBetweenConesOpt(Cone c1, Cone c2){
  double distance = std::sqrt( (c1.getOptX()-c2.getMeanX())*(c1.getOptX()-c2.getMeanX()) + (c1.getOptY()-c2.getMeanY())*(c1.getOptY()-c2.getMeanY()) );
  return distance;
}

/*Updates the global map after an optimization, the essentialMap is mainly used for plotting*/
void Slam::updateMap(uint32_t start, uint32_t end, bool updateToGlobal){
  for(uint32_t i = start; i < end; i++){

    if(updateToGlobal && m_coneList[i].isValid()){
      m_map.push_back(m_coneList[i]);
    }else{
      m_essentialMap.push_back(m_coneList[i]);
    }
  }
}
/*Filters the map on a set of criteria such as duplicates and proximity of cones to the car from the closest pose, Also assigns colors 
to the cones based on azimuth angle*/
void Slam::filterMap(){

  //Filter on mean and optimized value
  /*for(uint32_t i = 0; i < m_coneList.size(); i++){
    double distance = distanceBetweenConesOpt(m_coneList[i],m_coneList[i]);
    if(distance > m_newConeThreshold){
      m_coneList[i].setValidState(false);

    }
  }*/



  //Check for duplicates
  for(uint32_t i = 0; i < m_coneList.size(); i++){
    for(uint32_t j = 0; j < m_coneList.size(); j++){
      if(i != j){
        double distance = std::sqrt( (m_coneList[i].getOptX() - m_coneList[j].getOptX() )*(m_coneList[i].getOptX() - m_coneList[j].getOptX()) + (m_coneList[i].getOptY() - m_coneList[j].getOptY())*(m_coneList[i].getOptY() - m_coneList[j].getOptY()) );

        if(distance < m_newConeThreshold && m_coneList[i].isValid() && m_coneList[j].isValid()){
          m_coneList[j].setValidState(false);
        }
      } 
    }
  }  


  //Check closest pose didstance
  for(uint32_t i = 0; i < m_coneList.size(); i++){
    double closestPoseDistance = 10000;
    uint32_t closestPoseId = 0;
    for(uint32_t j = 0; j < m_poses.size(); j++){
      double distance = std::sqrt( (m_poses[j](0)-m_coneList[i].getOptX())*(m_poses[j](0)-m_coneList[i].getOptX()) + (m_poses[j](1)-m_coneList[i].getOptY())*(m_poses[j](1)-m_coneList[i].getOptY()) );
      if(distance < closestPoseDistance){
        closestPoseDistance = distance;
        closestPoseId = j;
      }

    }
    if(closestPoseDistance > 4 || m_coneList[i].getObservations()<2){
      m_coneList[i].setValidState(false);
    }
    //assigns color tp the cone list*/
    else{
      auto direction = m_coneList[i].getDirection(m_poses[closestPoseId]);
      if(direction.azimuthAngle()>0){
        m_coneList[i].setType(1);
      }
      else{
        m_coneList[i].setType(2);
      }
      
    }

  }


  //Evaluate if the map is good enough.


}
/*Assigns the parameters from the command line to member variables*/
void Slam::setUp(std::map<std::string, std::string> configuration)
{

  m_timeDiffMilliseconds = static_cast<uint32_t>(std::stoi(configuration["gatheringTimeMs"]));
  m_newConeThreshold = static_cast<double>(std::stod(configuration["sameConeThreshold"]));
  m_gpsReference[0] = static_cast<double>(std::stod(configuration["refLatitude"]));
  m_gpsReference[1] = static_cast<double>(std::stod(configuration["refLongitude"]));
  m_timeBetweenKeyframes = static_cast<double>(std::stod(configuration["timeBetweenKeyframes"]));
  m_coneMappingThreshold = static_cast<double>(std::stod(configuration["coneMappingThreshold"]));
  m_conesPerPacket = static_cast<int>(std::stoi(configuration["conesPerPacket"]));
  m_offsetDistance = std::stod(configuration["offsetDistanceThreshold"]);
  m_offsetHeading = std::stod(configuration["offsetHeadingThreshold"]);
  std::cout << m_offsetDistance << m_offsetHeading << std::endl;
  m_lapSize = std::stoi(configuration["lapSize"]);
  //std::cout << "Cones per packet" << m_conesPerPacket << std::endl;
  m_senderStamp = static_cast<int>(std::stoi(configuration["id"]));
}

void Slam::initializeModule(){
  //local Gps Vars
  double lastOdoX = 100000;
  double lastOdoY = 100000;
  int validGpsMeasurements = 0;
  bool gpsReadyState = false;

  //Local IMU vars
  bool imuReadyState = false;
  float lastVel = 100000;
  float lastHead = 100000;
  int validVelMeasurements = 0;
  int validHeadMeasurements = 0;
  while(!m_readyState){
    bool sleep = true;
    auto start = std::chrono::system_clock::now();

    while(sleep)
    {
      auto now = std::chrono::system_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);

      if(elapsed.count() > 50*1000){
        //std::cout << "Timed out" << std::endl;
        sleep = false;
      }
    }
    //GPS

    std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
    if(!gpsReadyState){

      if( std::fabs(m_odometryData(0) - lastOdoX) > 0.001 && std::fabs(m_odometryData(1) - lastOdoY) > 0.001){
        if(m_odometryData(0) < 200 && m_odometryData(1) < 200){
          lastOdoX = m_odometryData(0);
          lastOdoY = m_odometryData(1);
          validGpsMeasurements++;
        }
      }else{}

      if(validGpsMeasurements > 5){
        gpsReadyState = true;
        std::cout << "GPS Ready .." << std::endl;
      }
    }//GPS end  
    //IMU
    if(!imuReadyState){

      std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
      if(std::fabs(m_groundSpeed - lastVel) > 0.001){ 
        lastVel = m_groundSpeed;  
        validVelMeasurements++;
      }
      if(std::fabs(m_odometryData(2) - lastHead) > 0.001){
        lastHead = static_cast<float>(m_odometryData(2));  
        validHeadMeasurements++;
      }
      if(validVelMeasurements > 5 && validHeadMeasurements > 5){
        imuReadyState = true;
        std::cout << "IMU Ready .." << std::endl;
      }
    }

    if(gpsReadyState && imuReadyState){
      m_readyState = true;
      std::cout << "Slam ready check done !" << std::endl;  
    }
  }//While
  
  

}

void Slam::isMapValid(Eigen::Vector3d pose){

    uint32_t startPoseSet = 30;
    //Check pose

    Eigen::Vector2d firstLocalObs =  m_coneList[m_currentConeIndex].getLocalConeObservation(0);
    Eigen::Vector2d lastLocalObs =  m_coneList[m_currentConeIndex].getLocalConeObservation(m_coneList[m_currentConeIndex].getObservations()-2);
    Eigen::Vector2d loopClosingObs =  m_coneList[m_currentConeIndex].getLocalConeObservation(m_coneList[m_currentConeIndex].getObservations()-1);
    double azLocal = std::atan2(lastLocalObs(1),lastLocalObs(0));
    double azLoop = std::atan2(loopClosingObs(1),loopClosingObs(0));
    
    if(std::signbit(azLocal) == std::signbit(azLoop)){
      for(uint32_t i = 0; i < startPoseSet; i++){

        double distance = std::sqrt( (m_poses[i](0)-pose(0))*(m_poses[i](0)-pose(0)) + (m_poses[i](1)-pose(1))*(m_poses[i](1)-pose(1)) );
        if(distance < 1 ){
          double localObsDistance = std::sqrt( (firstLocalObs(0)-loopClosingObs(0))*(firstLocalObs(0)-loopClosingObs(0)) + (firstLocalObs(1)-loopClosingObs(1))*(firstLocalObs(1)-loopClosingObs(1)) );
          if(localObsDistance < 1){
            m_mapIsValid = true;
          }
        }
      }
    }
    if(!m_mapIsValid){
      m_validMapIterator++;

    }
}

void Slam::setStateMachineStatus(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockStateMachine(m_stateMachineMutex);
  auto machineStatus = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(data));
  int state = machineStatus.state();
  if(state == 2){
    m_readyStateMachine = true;
  }
/*Getters and methods used for the local pangolin UI*/
}
bool Slam::getModuleState(){

  return m_readyState;

}
std::vector<Eigen::Vector3d> Slam::drawPoses(){
  std::lock_guard<std::mutex> lockSensor(m_sensorMutex);
  return m_poses;
}

std::vector<Cone> Slam::drawCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_map;
}
std::vector<Cone> Slam::drawRawCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_coneList;
}
std::vector<Cone> Slam::drawLocalOptimizedCones(){
  std::lock_guard<std::mutex> lock(m_mapMutex);
  return m_essentialMap;
}
Eigen::Vector3d Slam::drawCurrentPose(){
  if(m_loopClosingComplete){
    std::lock_guard<std::mutex> lock(m_sendMutex);
    return m_sendPose;
  }
  else{
    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return m_odometryData;
  }
}
Eigen::Vector3d Slam::drawCurrentUKFPose(){

    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return m_odometryData;
  
}
std::vector<std::vector<int>> Slam::drawGraph(){
  std::lock_guard<std::mutex> lock1(m_mapMutex);
  std::lock_guard<std::mutex> lock2(m_sensorMutex);
  return m_connectivityGraph;
 
}
/*Used for extracting the map to a text file for debugging and analysis*/
void Slam::writeToPoseAndMapFile()
{
  std::string filepathMap;
  filepathMap = "./map.txt";
	
		std::ofstream f;
    	f.open(filepathMap.c_str());
		for(uint32_t i = 0; i<m_map.size(); i++){

				f << std::setprecision(9) << m_map[i].getX() << "\t" << m_map[i].getY() << std::endl;
		}
		f.close();
		std::cout << "map with " << m_map.size() << " points saved" << std::endl;


    std::string filepathPose;
    filepathPose = "./pose.txt";
		std::ofstream p;
    p.open(filepathPose.c_str());
		for(uint32_t i = 0; i<m_poses.size(); i++){

				p << std::setprecision(9) << m_poses[i](0) << "\t" << m_poses[i](1) << "\t" << m_poses[i](2) << std::endl;
		}
		p.close();

}
/*Gets the current map size to report the amount of cones seen, used for CAN logging by FSG*/
uint16_t Slam::getMapSize(){

  std::lock_guard<std::mutex> lockMap(m_mapMutex);
  return static_cast<uint16_t>(m_coneList.size());
}
void Slam::tearDown()
{
}
Slam::~Slam()
{
}

//switchstatereading 1411 actual cones, 1412 total cones CID 219
